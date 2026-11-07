/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * wiiu-wake-esp32: wake a Wii U from an ESP32 by acting as a paired Wii Remote.
 *
 * Talks raw HCI to the Bluetooth controller through VHCI. There is no Bluetooth host stack, only the small
 * parts of L2CAP, SDP and the Wii Remote protocol that pairing needs. It serves a small web page (index.html)
 * and an HTTP API. See README.md for usage.
 *
 * Waking: page the console and, as soon as the connection is up, disconnect with reason 0x15 (remote device
 * terminated connection due to power off). No authentication and no L2CAP from our side. A real Wii Remote
 * was observed to do the same when its power button is pressed.
 *
 * Verifying: after a wake the firmware connects like a controller, authenticates and requests the HID control
 * channel. The console's Bluetooth module accepts the connection in standby too, but answers on L2CAP only
 * once the console is running. On an answer it disconnects with reason 0x13, which does not wake anything.
 *
 * Pairing: the console queries SDP and then sets the controller up. The firmware answers only the HID reports
 * this requires and never sends button or data reports.
 *
 * Protocol references: Bluetooth Core Specification (HCI, L2CAP) and the WiiBrew "Wiimote" page, including
 * its section on the power button and disconnect reason 0x15
 * (PIN = host BD_ADDR reversed, class of device 0x002504, name "Nintendo RVL-CNT-01", HID PSMs 0x11/0x13,
 * reports 0x20 status, 0x21 read data, 0x22 ack).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"

static const char *TAG = "wake";

#define SYNC_SECONDS 120
#define SYNC_REFRESH_US (18 * 1000000LL)
/* Verification after a wake: the console answers L2CAP only once it is running, not in standby. */
#define VERIFY_DELAY_US (25 * 1000000LL)   /* time for the console to boot before the first check */
#define VERIFY_TIMEOUT_US (60 * 1000000LL) /* give up this long after the wake */
#define VERIFY_RETRY_US (5 * 1000000LL)    /* pause between checks */
#define PROBE_TIMEOUT_US (10 * 1000000LL)  /* longer than the page timeout of 5.12 s */
#define READ_MAX 512 /* limit for one HID memory read, each 16 bytes cost a report */
#define NVS_NS "wake"

/* ------------------------------------------------------------ state -- */
typedef struct { /* stored as is in NVS, keep the layout */
    uint8_t valid;
    uint8_t addr[6]; /* HCI byte order (LSB first) */
    uint8_t key[16];
    uint8_t has_key;
} console_t;

typedef struct {
    uint16_t local, remote;
    bool cfg_in_done, cfg_out_done;
} chan_t;

static console_t s_con;
static uint8_t s_local[6];

static bool s_syncing;
static int64_t s_sync_until;
static int64_t s_sync_refresh;
static const char *s_last_sync = "none";

static uint16_t s_handle = 0xFFFF; /* ACL handle of the console */
static bool s_waking;  /* wake connection being set up or closed */
static bool s_probing; /* verification connection being set up or closed */
typedef enum { WAKE_IDLE, WAKE_WAKING, WAKE_VERIFYING, WAKE_ON, WAKE_UNCONFIRMED } wake_state_t;
static wake_state_t s_wake_state;
static int64_t s_verify_next;
static int64_t s_verify_deadline;
static int64_t s_probe_until;
static uint32_t s_last_wake_ms;
static char s_last_wake_result[16] = "none";

static chan_t s_ch[3]; /* 0 = 0x11 control, 1 = 0x13 interrupt, 2 = 0x01 SDP */
static bool s_hid_announced;
static uint8_t s_sig_id = 1;
static uint8_t s_leds;

static char s_ip[16] = "-";
static char s_token[65];

static QueueHandle_t s_rxq;
static SemaphoreHandle_t s_txlock;
static SemaphoreHandle_t s_cmd_sem; /* Command Complete/Status during init */
static SemaphoreHandle_t s_lock;    /* guards the state shared by rx_task, UART and HTTP */

static void set_scan(uint8_t mode);

/* ------------------------------------------------------------ helpers -- */
static void addr_str(const uint8_t *a, char *out)
{
    sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X", a[5], a[4], a[3], a[2], a[1], a[0]);
}

static void nvs_put_blob(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, key, data, len);
    nvs_commit(h);
    nvs_close(h);
}

static bool nvs_get_exact(const char *key, void *out, size_t len)
{
    nvs_handle_t h;
    size_t n = len;
    bool ok = false;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        ok = nvs_get_blob(h, key, out, &n) == ESP_OK && n == len;
        nvs_close(h);
    }
    return ok;
}

static void nvs_put_str_key(const char *key, const char *val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (val) nvs_set_str(h, key, val);
    else nvs_erase_key(h, key);
    nvs_commit(h);
    nvs_close(h);
}

static bool nvs_get_str_key(const char *key, char *out, size_t cap)
{
    nvs_handle_t h;
    size_t n = cap;
    bool ok = false;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        ok = nvs_get_str(h, key, out, &n) == ESP_OK;
        nvs_close(h);
    }
    return ok;
}

/* ------------------------------------------------------------ VHCI -- */
typedef struct {
    uint16_t len;
    uint8_t *data;
} pkt_t;

static void vhci_send_available(void) {}

static int vhci_recv(uint8_t *data, uint16_t len)
{
    pkt_t p = { .len = len, .data = malloc(len) };
    if (!p.data) return 0;
    memcpy(p.data, data, len);
    if (xQueueSend(s_rxq, &p, 0) != pdTRUE) free(p.data);
    return 0;
}

static const esp_vhci_host_callback_t s_vhci_cb = {
    .notify_host_send_available = vhci_send_available,
    .notify_host_recv = vhci_recv,
};

static void hci_send_raw(const uint8_t *buf, uint16_t len)
{
    xSemaphoreTake(s_txlock, portMAX_DELAY);
    for (int i = 0; i < 200 && !esp_vhci_host_check_send_available(); i++) vTaskDelay(1);
    esp_vhci_host_send_packet((uint8_t *)buf, len);
    xSemaphoreGive(s_txlock);
}

static void hci_cmd(uint16_t opcode, const uint8_t *params, uint8_t plen)
{
    uint8_t buf[260];
    buf[0] = 0x01;
    buf[1] = opcode & 0xFF;
    buf[2] = opcode >> 8;
    buf[3] = plen;
    if (plen) memcpy(buf + 4, params, plen);
    hci_send_raw(buf, 4 + plen);
}

/* Init only: send a command and wait for Command Complete/Status */
static void hci_cmd_wait(uint16_t opcode, const uint8_t *params, uint8_t plen)
{
    xSemaphoreTake(s_cmd_sem, 0);
    hci_cmd(opcode, params, plen);
    if (xSemaphoreTake(s_cmd_sem, pdMS_TO_TICKS(1000)) != pdTRUE) ESP_LOGW(TAG, "timeout opcode 0x%04x", opcode);
}

static void l2cap_send(uint16_t cid, const uint8_t *payload, uint16_t plen)
{
    if (s_handle == 0xFFFF) return;
    uint8_t buf[300];
    uint16_t acllen = plen + 4;
    buf[0] = 0x02;
    buf[1] = s_handle & 0xFF;
    buf[2] = ((s_handle >> 8) & 0x0F) | 0x20; /* PB=10 */
    buf[3] = acllen & 0xFF;
    buf[4] = acllen >> 8;
    buf[5] = plen & 0xFF;
    buf[6] = plen >> 8;
    buf[7] = cid & 0xFF;
    buf[8] = cid >> 8;
    memcpy(buf + 9, payload, plen);
    hci_send_raw(buf, 9 + plen);
}

static void sig_send(uint8_t code, uint8_t id, const uint8_t *data, uint16_t dlen)
{
    uint8_t p[64];
    p[0] = code;
    p[1] = id;
    p[2] = dlen & 0xFF;
    p[3] = dlen >> 8;
    memcpy(p + 4, data, dlen);
    l2cap_send(0x0001, p, 4 + dlen);
}

static void disconnect_link(uint8_t reason)
{
    if (s_handle == 0xFFFF) return;
    uint8_t d[3] = { (uint8_t)s_handle, (uint8_t)(s_handle >> 8), reason };
    hci_cmd(0x0406, d, 3);
}

/* ------------------------------------------------------------ sync -- */
static void sync_end(const char *why)
{
    if (!s_syncing) return;
    s_syncing = false;
    s_last_sync = why;
    set_scan(0x00);
    ESP_LOGI(TAG, "EVT sync closed (%s)", why);
}

/* ------------------------------------------------------------ HID -- */
static chan_t *chan_by_local(uint16_t cid)
{
    for (int i = 0; i < 3; i++) if (s_ch[i].local == cid) return &s_ch[i];
    return NULL;
}

static bool hid_ready(void)
{
    return s_ch[1].remote && s_ch[1].cfg_in_done && s_ch[1].cfg_out_done;
}

static void hid_send(const uint8_t *rep, uint16_t len)
{
    if (!hid_ready()) return;
    uint8_t p[32];
    p[0] = 0xA1;
    memcpy(p + 1, rep, len);
    l2cap_send(s_ch[1].remote, p, 1 + len);
}

static void hid_ack(uint8_t rid, uint8_t err)
{
    uint8_t r[5] = { 0x22, 0, 0, rid, err };
    hid_send(r, sizeof(r));
}

/* Memory read: EEPROM calibration at 0x16..0x1F gets rest values, everything else zeros.
 * 0xA4xxxx (extension) and 0xA6xxxx (MotionPlus) answer with error 7, nothing attached.
 * The console does not finish setting the controller up without this. */
static void hid_read(const uint8_t *d)
{
    uint8_t space = d[0];
    uint32_t addr = (d[1] << 16) | (d[2] << 8) | d[3];
    uint16_t size = (d[4] << 8) | d[5];
    if (size > READ_MAX) size = READ_MAX;
    if ((space & 0x04) && (((addr & 0xFF0000) == 0xA40000) || ((addr & 0xFF0000) == 0xA60000))) {
        uint8_t r[22] = { 0x21, 0, 0, 0x07, (uint8_t)(addr >> 8), (uint8_t)addr };
        hid_send(r, sizeof(r));
        return;
    }
    static const uint8_t cal[10] = { 0x80, 0x80, 0x80, 0x00, 0x9A, 0x9A, 0x9A, 0x00, 0x40, 0x00 };
    uint32_t off = 0;
    while (off < size) {
        uint8_t chunk = (size - off) > 16 ? 16 : (uint8_t)(size - off);
        uint32_t a = addr + off;
        uint8_t r[22];
        memset(r, 0, sizeof(r));
        r[0] = 0x21;
        r[3] = (uint8_t)((chunk - 1) << 4);
        r[4] = (a >> 8) & 0xFF;
        r[5] = a & 0xFF;
        for (int i = 0; i < chunk; i++) {
            uint32_t x = a + i;
            if (!(space & 0x04) && x >= 0x16 && x < 0x20) r[6 + i] = cal[x - 0x16];
        }
        hid_send(r, sizeof(r));
        off += chunk;
    }
}

static void hid_output(const uint8_t *p, uint16_t n)
{
    if (n < 2) return;
    /* p[0] = 0xA2 (interrupt channel) or 0x52 (control channel, legacy form) */
    uint8_t rid = p[1];
    const uint8_t *d = p + 2;
    uint16_t dn = n - 2;
    bool want_ack = dn > 0 && (d[0] & 0x02);
    switch (rid) {
    case 0x11: /* LEDs */
        if (dn) s_leds = d[0];
        ESP_LOGI(TAG, "EVT leds 0x%x", s_leds >> 4);
        if (want_ack) hid_ack(rid, 0);
        break;
    case 0x15: { /* Status */
        uint8_t r[7] = { 0x20, 0, 0, (uint8_t)(s_leds & 0xF0), 0, 0, 0xC0 };
        hid_send(r, sizeof(r));
        break;
    }
    case 0x16: hid_ack(rid, 0); break; /* Write */
    case 0x17: if (dn >= 6) hid_read(d); break;
    default: /* e.g. 0x12 report mode, 0x13/0x1a IR camera */
        if (want_ack) hid_ack(rid, 0);
        break;
    }
}

/* --------------------------------------------------------------- SDP -- */
/* Responses of a real Wii Remote, taken unchanged from sdp.c in WiimoteEmulator by rnconrad
 * (https://github.com/rnconrad/WiimoteEmulator), also used by bluubomb by GaryOderNichts
 * (https://github.com/GaryOderNichts/bluubomb). These tables are byte-identical to both projects, which carry
 * no license, and are included with credit. They are not covered by a license grant from those projects.
 * The console sends ServiceSearch (0x02) -> resp0, then ServiceAttribute (0x04) with continuation
 * -> resp1..resp4. The response is selected by continuation state, falling back to the transaction ID. */
static const uint8_t sdp_resp0[14] = {
    0x03, 0x00, 0x00, 0x00, 0x09, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00
};
static const uint8_t sdp_resp1[128] = {
    0x05, 0x00, 0x01, 0x00, 0x7B, 0x00, 0x76, 0x36, 0x01, 0xCC, 0x09, 0x00, 0x00, 0x0A, 0x00, 0x01,
    0x00, 0x00, 0x09, 0x00, 0x01, 0x35, 0x03, 0x19, 0x11, 0x24, 0x09, 0x00, 0x04, 0x35, 0x0D, 0x35,
    0x06, 0x19, 0x01, 0x00, 0x09, 0x00, 0x11, 0x35, 0x03, 0x19, 0x00, 0x11, 0x09, 0x00, 0x05, 0x35,
    0x03, 0x19, 0x10, 0x02, 0x09, 0x00, 0x06, 0x35, 0x09, 0x09, 0x65, 0x6E, 0x09, 0x00, 0x6A, 0x09,
    0x01, 0x00, 0x09, 0x00, 0x09, 0x35, 0x08, 0x35, 0x06, 0x19, 0x11, 0x24, 0x09, 0x01, 0x00, 0x09,
    0x00, 0x0D, 0x35, 0x0F, 0x35, 0x0D, 0x35, 0x06, 0x19, 0x01, 0x00, 0x09, 0x00, 0x13, 0x35, 0x03,
    0x19, 0x00, 0x11, 0x09, 0x01, 0x00, 0x25, 0x13, 0x4E, 0x69, 0x6E, 0x74, 0x65, 0x6E, 0x64, 0x6F,
    0x20, 0x52, 0x56, 0x4C, 0x2D, 0x43, 0x4E, 0x54, 0x2D, 0x30, 0x31, 0x09, 0x01, 0x02, 0x00, 0x76
};
static const uint8_t sdp_resp2[128] = {
    0x05, 0x00, 0x02, 0x00, 0x7B, 0x00, 0x76, 0x01, 0x25, 0x13, 0x4E, 0x69, 0x6E, 0x74, 0x65, 0x6E,
    0x64, 0x6F, 0x20, 0x52, 0x56, 0x4C, 0x2D, 0x43, 0x4E, 0x54, 0x2D, 0x30, 0x31, 0x09, 0x01, 0x02,
    0x25, 0x08, 0x4E, 0x69, 0x6E, 0x74, 0x65, 0x6E, 0x64, 0x6F, 0x09, 0x02, 0x00, 0x09, 0x01, 0x00,
    0x09, 0x02, 0x01, 0x09, 0x01, 0x11, 0x09, 0x02, 0x02, 0x08, 0x04, 0x09, 0x02, 0x03, 0x08, 0x33,
    0x09, 0x02, 0x04, 0x28, 0x00, 0x09, 0x02, 0x05, 0x28, 0x01, 0x09, 0x02, 0x06, 0x35, 0xDF, 0x35,
    0xDD, 0x08, 0x22, 0x25, 0xD9, 0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x10, 0x15, 0x00, 0x26,
    0xFF, 0x00, 0x75, 0x08, 0x95, 0x01, 0x06, 0x00, 0xFF, 0x09, 0x01, 0x91, 0x00, 0x85, 0x11, 0x95,
    0x01, 0x09, 0x01, 0x91, 0x00, 0x85, 0x12, 0x95, 0x02, 0x09, 0x01, 0x91, 0x00, 0x02, 0x00, 0xEC
};
static const uint8_t sdp_resp3[128] = {
    0x05, 0x00, 0x03, 0x00, 0x7B, 0x00, 0x76, 0x85, 0x13, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00, 0x85,
    0x14, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00, 0x85, 0x15, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00, 0x85,
    0x16, 0x95, 0x15, 0x09, 0x01, 0x91, 0x00, 0x85, 0x17, 0x95, 0x06, 0x09, 0x01, 0x91, 0x00, 0x85,
    0x18, 0x95, 0x15, 0x09, 0x01, 0x91, 0x00, 0x85, 0x19, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00, 0x85,
    0x1A, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00, 0x85, 0x20, 0x95, 0x06, 0x09, 0x01, 0x81, 0x00, 0x85,
    0x21, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00, 0x85, 0x22, 0x95, 0x04, 0x09, 0x01, 0x81, 0x00, 0x85,
    0x30, 0x95, 0x02, 0x09, 0x01, 0x81, 0x00, 0x85, 0x31, 0x95, 0x05, 0x09, 0x01, 0x81, 0x00, 0x85,
    0x32, 0x95, 0x0A, 0x09, 0x01, 0x81, 0x00, 0x85, 0x33, 0x95, 0x11, 0x09, 0x01, 0x02, 0x01, 0x62
};
static const uint8_t sdp_resp4[117] = {
    0x05, 0x00, 0x04, 0x00, 0x70, 0x00, 0x6D, 0x81, 0x00, 0x85, 0x34, 0x95, 0x15, 0x09, 0x01, 0x81,
    0x00, 0x85, 0x35, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00, 0x85, 0x36, 0x95, 0x15, 0x09, 0x01, 0x81,
    0x00, 0x85, 0x37, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00, 0x85, 0x3D, 0x95, 0x15, 0x09, 0x01, 0x81,
    0x00, 0x85, 0x3E, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00, 0x85, 0x3F, 0x95, 0x15, 0x09, 0x01, 0x81,
    0x00, 0xC0, 0x09, 0x02, 0x07, 0x35, 0x08, 0x35, 0x06, 0x09, 0x04, 0x09, 0x09, 0x01, 0x00, 0x09,
    0x02, 0x08, 0x28, 0x00, 0x09, 0x02, 0x09, 0x28, 0x01, 0x09, 0x02, 0x0A, 0x28, 0x01, 0x09, 0x02,
    0x0B, 0x09, 0x01, 0x00, 0x09, 0x02, 0x0C, 0x09, 0x0C, 0x80, 0x09, 0x02, 0x0D, 0x28, 0x00, 0x09,
    0x02, 0x0E, 0x28, 0x00, 0x00
};
static const uint8_t *const sdp_resp[5] = {sdp_resp0, sdp_resp1, sdp_resp2, sdp_resp3, sdp_resp4};
static const uint16_t sdp_resp_len[5] = {sizeof sdp_resp0, sizeof sdp_resp1, sizeof sdp_resp2, sizeof sdp_resp3, sizeof sdp_resp4};

static void sdp_rx(const uint8_t *p, uint16_t n)
{
    if (n < 5 || !s_ch[2].remote) return;
    uint8_t pdu = p[0];
    int idx = -1;
    if (pdu == 0x02) {
        idx = 0;
    } else if (pdu == 0x04) {
        /* the continuation state is at the end: [len][bytes] */
        if (p[n - 1] == 0x00) idx = 1;
        else if (n >= 3 && p[n - 3] == 0x02) {
            uint16_t c = (uint16_t)(p[n - 2] << 8) | p[n - 1];
            if (c == 0x0076) idx = 2;
            else if (c == 0x00EC) idx = 3;
            else if (c == 0x0162) idx = 4;
        }
        if (idx < 0) { int t = p[2]; if (t >= 1 && t <= 4) idx = t; }
    }
    if (idx < 0) {            /* unknown: ErrorResponse 0x0003, invalid syntax */
        uint8_t r[7] = {0x01, p[1], p[2], 0x00, 0x02, 0x00, 0x03};
        ESP_LOGW(TAG, "SDP pdu 0x%02x unbekannt -> error", pdu);
        l2cap_send(s_ch[2].remote, r, sizeof r);
        return;
    }
    uint8_t r[128];
    memcpy(r, sdp_resp[idx], sdp_resp_len[idx]);
    r[1] = p[1]; r[2] = p[2]; /* echo the transaction ID */
    l2cap_send(s_ch[2].remote, r, sdp_resp_len[idx]);
}

/* ------------------------------------------------------------ L2CAP -- */
static void l2cap_cfg_req(chan_t *c)
{
    uint8_t d[4] = { (uint8_t)c->remote, (uint8_t)(c->remote >> 8), 0, 0 };
    sig_send(0x04, s_sig_id++, d, 4);
}

static void chan_check(void)
{
    if (!s_hid_announced && hid_ready() && s_ch[0].cfg_in_done && s_ch[0].cfg_out_done) {
        s_hid_announced = true;
        ESP_LOGI(TAG, "EVT hid channels open");
        sync_end("paired");
    }
}

/* Responder side only: during pairing the console opens all channels. */
static void l2cap_signal(const uint8_t *p, uint16_t n)
{
    while (n >= 4) {
        uint8_t code = p[0], id = p[1];
        uint16_t len = p[2] | (p[3] << 8);
        const uint8_t *d = p + 4;
        if (len > n - 4) return; /* truncated or malformed */
        switch (code) {
        case 0x02: { /* Connection Request */
            if (len < 4) break;
            uint16_t psm = d[0] | (d[1] << 8), scid = d[2] | (d[3] << 8);
            int i = psm == 0x11 ? 0 : psm == 0x13 ? 1 : psm == 0x01 ? 2 : -1;
            uint8_t r[8];
            uint16_t dcid = i >= 0 ? s_ch[i].local : 0;
            r[0] = dcid; r[1] = dcid >> 8; r[2] = scid; r[3] = scid >> 8;
            r[4] = i >= 0 ? 0 : 2; r[5] = 0; r[6] = 0; r[7] = 0; /* 2 = PSM not supported */
            sig_send(0x03, id, r, 8);
            if (i >= 0) { s_ch[i].remote = scid; l2cap_cfg_req(&s_ch[i]); }
            break;
        }
        case 0x03: { /* Connection Response: only verification opens a channel from our side */
            if (len < 6) break;
            uint16_t res = d[4] | (d[5] << 8);
            if (!s_probing || res == 1) break; /* 1 = pending */
            if (res == 0) {
                ESP_LOGI(TAG, "EVT wake: console is on");
                s_wake_state = WAKE_ON;
            }
            disconnect_link(0x13);
            break;
        }
        case 0x04: { /* Configuration Request: accept everything */
            if (len < 2) break;
            uint16_t dcid = d[0] | (d[1] << 8);
            chan_t *c = chan_by_local(dcid);
            uint8_t r[6] = { 0, 0, 0, 0, 0, 0 };
            if (c) { r[0] = c->remote; r[1] = c->remote >> 8; c->cfg_in_done = true; }
            sig_send(0x05, id, r, 6);
            chan_check();
            break;
        }
        case 0x05: { /* Configuration Response */
            if (len < 6) break;
            uint16_t scid = d[0] | (d[1] << 8), res = d[4] | (d[5] << 8);
            chan_t *c = chan_by_local(scid);
            if (c && res == 0) c->cfg_out_done = true;
            chan_check();
            break;
        }
        case 0x06: { /* Disconnection Request */
            if (len < 4) break;
            uint16_t dcid = d[0] | (d[1] << 8);
            uint8_t r[4] = { d[0], d[1], d[2], d[3] };
            chan_t *c = chan_by_local(dcid);
            if (c) { c->remote = 0; c->cfg_in_done = c->cfg_out_done = false; }
            sig_send(0x07, id, r, 4);
            break;
        }
        case 0x0A: { /* Information Request: not supported */
            if (len < 2) break;
            uint8_t r[4] = { d[0], d[1], 1, 0 };
            sig_send(0x0B, id, r, 4);
            break;
        }
        default: break;
        }
        p += 4 + len;
        n -= 4 + len;
    }
}

static void reset_link(void)
{
    s_handle = 0xFFFF;
    s_waking = false;
    s_probing = false;
    s_hid_announced = false;
    memset(s_ch, 0, sizeof(s_ch));
    s_ch[0].local = 0x0040;
    s_ch[1].local = 0x0041;
    s_ch[2].local = 0x0042;
}

/* ------------------------------------------------------------ HCI events -- */
static void set_scan(uint8_t mode)
{
    hci_cmd(0x0C1A, &mode, 1);
}

/* Smallest parameter length of the events handled below */
static uint8_t event_min_len(uint8_t code)
{
    switch (code) {
    case 0x03: return 11; /* Connection Complete */
    case 0x04: return 10; /* Connection Request */
    case 0x05: return 4;  /* Disconnection Complete */
    case 0x06: return 3;  /* Authentication Complete */
    case 0x0E: return 4;  /* Command Complete */
    case 0x0F: return 4;  /* Command Status */
    case 0x16: return 6;  /* PIN Code Request */
    case 0x17: return 6;  /* Link Key Request */
    case 0x18: return 23; /* Link Key Notification */
    default: return 0;
    }
}

static void on_event(const uint8_t *e, uint16_t n)
{
    uint8_t code = e[0];
    uint8_t plen = e[1];
    const uint8_t *p = e + 2;
    if (n < 2 + plen || plen < event_min_len(code)) return;
    char as[18];
    switch (code) {
    case 0x0E: /* Command Complete */
        {
            uint16_t op = p[1] | (p[2] << 8);
            if (op == 0x1009 && p[3] == 0 && plen >= 10) memcpy(s_local, p + 4, 6);
            if (p[3]) ESP_LOGW(TAG, "cmd 0x%04x status 0x%02x", op, p[3]);
        }
        xSemaphoreGive(s_cmd_sem);
        break;
    case 0x0F: /* Command Status */
        if (p[0]) {
            uint16_t op = p[2] | (p[3] << 8);
            ESP_LOGW(TAG, "cmd status 0x%02x opcode 0x%04x", p[0], op);
            if (op == 0x0405 && s_waking) {
                s_waking = false;
                s_wake_state = WAKE_IDLE;
                snprintf(s_last_wake_result, sizeof(s_last_wake_result), "failed_0x%02x", p[0]);
            } else if (op == 0x0405 && s_probing) {
                s_probing = false;
                s_verify_next = esp_timer_get_time() + VERIFY_RETRY_US;
            }
        }
        xSemaphoreGive(s_cmd_sem);
        break;
    case 0x04: { /* Connection Request: accept only during the sync window */
        uint8_t r[7];
        memcpy(r, p, 6);
        addr_str(p, as);
        ESP_LOGI(TAG, "EVT conn request from %s", as);
        if (s_handle == 0xFFFF && s_syncing) {
            r[6] = 0x01; /* stay peripheral */
            hci_cmd(0x0409, r, 7);
        } else {
            r[6] = 0x0F; /* unacceptable BD_ADDR */
            hci_cmd(0x040A, r, 7);
        }
        break;
    }
    case 0x03: /* Connection Complete */
        addr_str(p + 3, as);
        ESP_LOGI(TAG, "EVT connection complete status 0x%02x handle 0x%03x %s", p[0], p[1] | (p[2] << 8), as);
        if (p[0] == 0) {
            bool waking = s_waking, probing = s_probing;
            reset_link();
            s_waking = waking;
            s_probing = probing;
            s_handle = (p[1] | (p[2] << 8)) & 0x0FFF;
            if (s_waking) {
                uint8_t d[3] = { (uint8_t)s_handle, (uint8_t)(s_handle >> 8), 0x15 };
                ESP_LOGI(TAG, "EVT wake: connected, sending power-off disconnect");
                hci_cmd(0x0406, d, 3);
            } else if (s_probing) {
                uint8_t h[2] = { (uint8_t)s_handle, (uint8_t)(s_handle >> 8) };
                hci_cmd(0x0411, h, 2); /* Authentication_Requested */
            }
        } else if (s_waking) {
            ESP_LOGW(TAG, "EVT wake failed status 0x%02x", p[0]);
            snprintf(s_last_wake_result, sizeof(s_last_wake_result), "failed_0x%02x", p[0]);
            s_waking = false;
            s_wake_state = WAKE_IDLE;
        } else if (s_probing) {
            s_probing = false;
            s_verify_next = esp_timer_get_time() + VERIFY_RETRY_US;
        }
        break;
    case 0x06: /* Authentication Complete */
        if (s_probing) {
            if (p[0] == 0) {
                uint8_t d[4] = { 0x11, 0x00, (uint8_t)s_ch[0].local, (uint8_t)(s_ch[0].local >> 8) };
                sig_send(0x02, s_sig_id++, d, 4); /* L2CAP Connection Request, PSM 0x11 */
            } else {
                disconnect_link(0x13);
            }
        }
        break;
    case 0x05: /* Disconnection Complete */
        ESP_LOGI(TAG, "EVT disconnected status 0x%02x reason 0x%02x", p[0], p[3]);
        if (s_waking) {
            ESP_LOGI(TAG, "EVT wake: done (reason 0x%02x)", p[3]);
            strcpy(s_last_wake_result, "done");
            int64_t now = esp_timer_get_time();
            s_wake_state = WAKE_VERIFYING;
            s_verify_next = now + VERIFY_DELAY_US;
            s_verify_deadline = now + VERIFY_TIMEOUT_US;
        } else if (s_probing) {
            s_verify_next = esp_timer_get_time() + VERIFY_RETRY_US;
        }
        reset_link();
        break;
    case 0x16: { /* PIN Code Request: the PIN is the host address reversed, which is its HCI byte order */
        uint8_t r[23];
        memset(r, 0, sizeof(r));
        memcpy(r, p, 6);
        r[6] = 6;
        memcpy(r + 7, p, 6);
        hci_cmd(0x040D, r, 23);
        break;
    }
    case 0x17: /* Link Key Request */
        if (s_con.valid && s_con.has_key && !memcmp(p, s_con.addr, 6)) {
            uint8_t r[22];
            memcpy(r, p, 6);
            memcpy(r + 6, s_con.key, 16);
            hci_cmd(0x040B, r, 22);
        } else {
            hci_cmd(0x040C, p, 6);
        }
        break;
    case 0x18: /* Link Key Notification */
        addr_str(p, as);
        memcpy(s_con.addr, p, 6);
        memcpy(s_con.key, p + 6, 16);
        s_con.valid = 1;
        s_con.has_key = 1;
        nvs_put_blob("con", &s_con, sizeof(s_con));
        ESP_LOGI(TAG, "EVT link key stored for console %s (type %u)", as, p[22]);
        sync_end("paired");
        break;
    default:
        break;
    }
}

static void on_acl(const uint8_t *a, uint16_t n)
{
    if (n < 8) return;
    uint16_t l2len = a[4] | (a[5] << 8);
    uint16_t cid = a[6] | (a[7] << 8);
    const uint8_t *pl = a + 8;
    if (l2len > n - 8) l2len = n - 8; /* no fragmentation expected */
    if (cid == 0x0001) l2cap_signal(pl, l2len);
    else if (cid == s_ch[2].local) sdp_rx(pl, l2len);
    else if (cid == s_ch[0].local || cid == s_ch[1].local) hid_output(pl, l2len);
}

static void rx_task(void *arg)
{
    pkt_t p;
    while (1) {
        if (xQueueReceive(s_rxq, &p, portMAX_DELAY) != pdTRUE) continue;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (p.data[0] == 0x04 && p.len >= 3) on_event(p.data + 1, p.len - 1);
        else if (p.data[0] == 0x02 && p.len >= 5) on_acl(p.data + 1, p.len - 1);
        xSemaphoreGive(s_lock);
        free(p.data);
    }
}

/* ------------------------------------------------------------ actions -- */
/* All of these must be called with s_lock held */
static bool wake_busy(void)
{
    return s_wake_state == WAKE_WAKING || s_wake_state == WAKE_VERIFYING;
}

static const char *wake_state_name(void)
{
    switch (s_wake_state) {
    case WAKE_WAKING: return "waking";
    case WAKE_VERIFYING: return "verifying";
    case WAKE_ON: return "on";
    case WAKE_UNCONFIRMED: return "unconfirmed";
    default: return "idle";
    }
}

static void create_connection(void)
{
    uint8_t r[13];
    memcpy(r, s_con.addr, 6);
    r[6] = 0x18; r[7] = 0xCC; /* DM1 DH1 DM3 DH3 DM5 DH5 */
    r[8] = 0x01;              /* R1 */
    r[9] = 0;
    r[10] = 0; r[11] = 0;
    r[12] = 0x01;             /* allow role switch */
    hci_cmd(0x0405, r, 13);
}

/* Runs the verification: connects like a controller and checks that the console answers on L2CAP. */
static void wake_tick(int64_t now)
{
    if (s_probing && now > s_probe_until) {
        s_probe_until = now + PROBE_TIMEOUT_US;
        if (s_handle != 0xFFFF) {
            disconnect_link(0x13);
        } else {
            s_probing = false;
            s_verify_next = now + VERIFY_RETRY_US;
        }
    }
    if (s_wake_state != WAKE_VERIFYING || s_probing || now < s_verify_next) return;
    if (now >= s_verify_deadline) {
        ESP_LOGW(TAG, "EVT wake: console did not answer, not confirmed");
        s_wake_state = WAKE_UNCONFIRMED;
        return;
    }
    ESP_LOGI(TAG, "EVT wake: checking console");
    s_probing = true;
    s_probe_until = now + PROBE_TIMEOUT_US;
    create_connection();
}

static bool do_sync(void)
{
    if (wake_busy()) return false;
    int64_t now = esp_timer_get_time();
    s_syncing = true;
    s_last_sync = "open";
    s_sync_until = now + SYNC_SECONDS * 1000000LL;
    s_sync_refresh = now + SYNC_REFRESH_US;
    set_scan(0x03);
    ESP_LOGI(TAG, "EVT sync open (%d s)", SYNC_SECONDS);
    return true;
}

/* Returns 0 when started, -1 when no console is paired, -2 when already connected or waking */
static int do_wake(void)
{
    if (!s_con.valid) return -1;
    if (s_handle != 0xFFFF || s_syncing || wake_busy()) return -2;
    s_waking = true;
    s_wake_state = WAKE_WAKING;
    s_last_wake_ms = (uint32_t)(esp_timer_get_time() / 1000);
    strcpy(s_last_wake_result, "paging");
    create_connection();
    ESP_LOGI(TAG, "EVT wake: paging console");
    return 0;
}

static bool do_forget(void)
{
    if (wake_busy()) return false;
    memset(&s_con, 0, sizeof(s_con));
    nvs_put_blob("con", &s_con, sizeof(s_con));
    ESP_LOGI(TAG, "EVT console forgotten");
    return true;
}

static void info_strs(char *a, char *c)
{
    addr_str(s_local, a);
    strcpy(c, "-");
    if (s_con.valid) addr_str(s_con.addr, c);
}

static void do_info(void)
{
    char a[18], c[18];
    info_strs(a, c);
    ESP_LOGI(TAG, "INFO esp_bdaddr=%s console=%s link_key=%d connected=%d syncing=%d ip=%s wake_state=%s last_wake=%s@%lums",
             a, c, s_con.has_key, s_handle != 0xFFFF, s_syncing, s_ip, wake_state_name(), s_last_wake_result, (unsigned long)s_last_wake_ms);
}

/* ------------------------------------------------------------ HTTP -- */
static bool http_auth(httpd_req_t *req)
{
    if (!s_token[0]) return true;
    char hdr[80];
    char want[80];
    snprintf(want, sizeof(want), "Bearer %s", s_token);
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) == ESP_OK && !strcmp(hdr, want)) return true;
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"error\":\"unauthorized\"}");
    return false;
}

static esp_err_t http_json(httpd_req_t *req, const char *status, const char *body)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t h_wake(httpd_req_t *req)
{
    if (!http_auth(req)) return ESP_OK;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int r = do_wake();
    xSemaphoreGive(s_lock);
    if (r == -1) return http_json(req, "409 Conflict", "{\"error\":\"no console paired\"}");
    if (r == -2) return http_json(req, "409 Conflict", "{\"error\":\"busy\"}");
    return http_json(req, "202 Accepted", "{\"started\":true}");
}

static esp_err_t h_sync(httpd_req_t *req)
{
    if (!http_auth(req)) return ESP_OK;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ok = do_sync();
    xSemaphoreGive(s_lock);
    if (!ok) return http_json(req, "409 Conflict", "{\"error\":\"busy\"}");
    return http_json(req, "202 Accepted", "{\"syncing\":true}");
}

static esp_err_t h_forget(httpd_req_t *req)
{
    if (!http_auth(req)) return ESP_OK;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ok = do_forget();
    xSemaphoreGive(s_lock);
    if (!ok) return http_json(req, "409 Conflict", "{\"error\":\"busy\"}");
    return http_json(req, "200 OK", "{\"paired\":false}");
}

extern const char index_html_start[] asm("_binary_index_html_start");

/* The page holds no data of its own, so it needs no token. */
static esp_err_t h_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_sendstr(req, index_html_start);
}

static esp_err_t h_info(httpd_req_t *req)
{
    if (!http_auth(req)) return ESP_OK;
    char a[18], c[18], body[448];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    info_strs(a, c);
    int64_t now = esp_timer_get_time();
    long sync_left = s_syncing ? (long)((s_sync_until - now) / 1000000) : 0;
    wifi_ap_record_t ap;
    int rssi = esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
    snprintf(body, sizeof(body),
             "{\"bdaddr\":\"%s\",\"console\":\"%s\",\"paired\":%s,\"connected\":%s,\"syncing\":%s,\"sync_left_s\":%ld,\"last_sync\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"uptime_ms\":%lu,\"wake_state\":\"%s\","
             "\"last_wake\":{\"time_ms\":%lu,\"result\":\"%s\"}}",
             a, c, s_con.has_key ? "true" : "false", s_handle != 0xFFFF ? "true" : "false",
             s_syncing ? "true" : "false", sync_left, s_last_sync, s_ip, rssi, (unsigned long)(now / 1000), wake_state_name(),
             (unsigned long)s_last_wake_ms, s_last_wake_result);
    xSemaphoreGive(s_lock);
    return http_json(req, "200 OK", body);
}

static void http_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 6144;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "http server failed to start");
        return;
    }
    const httpd_uri_t uris[] = {
        { .uri = "/", .method = HTTP_GET, .handler = h_index },
        { .uri = "/wake", .method = HTTP_POST, .handler = h_wake },
        { .uri = "/sync", .method = HTTP_POST, .handler = h_sync },
        { .uri = "/forget", .method = HTTP_POST, .handler = h_forget },
        { .uri = "/info", .method = HTTP_GET, .handler = h_info },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) httpd_register_uri_handler(server, &uris[i]);
}

/* ------------------------------------------------------------ Wi-Fi -- */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (strcmp(s_ip, "-")) ESP_LOGW(TAG, "EVT wifi disconnected, reconnecting");
        strcpy(s_ip, "-");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "EVT wifi connected, ip %s", s_ip);
    }
}

static void wifi_start(void)
{
    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    if (!nvs_get_str_key("wifi_ssid", (char *)wc.sta.ssid, sizeof(wc.sta.ssid))) {
        ESP_LOGI(TAG, "no wifi configured, http off (uart: wifi <ssid> <password>)");
        return;
    }
    nvs_get_str_key("wifi_pass", (char *)wc.sta.password, sizeof(wc.sta.password));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, "wiiu-wake-esp32");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    http_start();
}

/* ------------------------------------------------------------ UART -- */
static void restart_now(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

/* Takes the next argument from *p in place: either a plain word, or text in double quotes where a backslash
 * escapes the next character. Returns NULL if there is none or a quote is not closed. */
static char *next_arg(char **p)
{
    char *in = *p;
    while (*in == ' ') in++;
    if (!*in) return NULL;
    char *start = in;
    if (*in != '"') {
        while (*in && *in != ' ') in++;
        if (*in) *in++ = 0;
        *p = in;
        return start;
    }
    char *out = ++in;
    start = out;
    while (*in && *in != '"') {
        if (*in == '\\' && in[1]) in++;
        *out++ = *in++;
    }
    if (*in != '"') return NULL;
    *out = 0;
    *p = in + 1;
    return start;
}

static void cmd_wifi(char *arg)
{
    char *ssid = next_arg(&arg);
    char *pass = ssid ? next_arg(&arg) : NULL;
    if (!ssid || !pass || !*ssid) {
        ESP_LOGW(TAG, "use: wifi \"<ssid>\" \"<password>\"");
        return;
    }
    if (strlen(ssid) > 32 || strlen(pass) > 63) {
        ESP_LOGW(TAG, "ssid or password too long");
        return;
    }
    nvs_put_str_key("wifi_ssid", ssid);
    nvs_put_str_key("wifi_pass", pass);
    ESP_LOGI(TAG, "EVT wifi saved, restarting");
    restart_now();
}

static void cmd_token(const char *arg)
{
    if (!strcmp(arg, "clear")) {
        s_token[0] = 0;
        nvs_put_str_key("token", NULL);
        ESP_LOGI(TAG, "EVT token cleared, http is open");
    } else if (strlen(arg) >= sizeof(s_token)) {
        ESP_LOGW(TAG, "token too long");
    } else {
        strcpy(s_token, arg);
        nvs_put_str_key("token", s_token);
        ESP_LOGI(TAG, "EVT token set");
    }
}

static void do_cmd(char *c)
{
    if (!strcmp(c, "sync")) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool ok = do_sync();
        xSemaphoreGive(s_lock);
        if (!ok) ESP_LOGW(TAG, "busy, a wake is in progress");
    } else if (!strcmp(c, "wake")) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int r = do_wake();
        xSemaphoreGive(s_lock);
        if (r == -1) ESP_LOGW(TAG, "no console stored, sync first");
        if (r == -2) ESP_LOGW(TAG, "busy");
    } else if (!strcmp(c, "info")) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        do_info();
        xSemaphoreGive(s_lock);
    } else if (!strcmp(c, "forget")) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool ok = do_forget();
        xSemaphoreGive(s_lock);
        if (!ok) ESP_LOGW(TAG, "busy, a wake is in progress");
    } else if (!strncmp(c, "wifi ", 5)) {
        cmd_wifi(c + 5);
    } else if (!strncmp(c, "token ", 6)) {
        cmd_token(c + 6);
    } else if (*c) {
        ESP_LOGW(TAG, "unknown cmd (sync|wake|info|forget|wifi|token)");
    }
}

/* ------------------------------------------------------------ main -- */
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    if (!nvs_get_exact("con", &s_con, sizeof(s_con))) memset(&s_con, 0, sizeof(s_con));
    nvs_get_str_key("token", s_token, sizeof(s_token));
    reset_link();

    s_rxq = xQueueCreate(32, sizeof(pkt_t));
    s_txlock = xSemaphoreCreateMutex();
    s_cmd_sem = xSemaphoreCreateBinary();
    s_lock = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_vhci_host_register_callback(&s_vhci_cb));
    xTaskCreatePinnedToCore(rx_task, "hci_rx", 6144, NULL, 6, NULL, 0);

    hci_cmd_wait(0x0C03, NULL, 0); /* Reset */
    hci_cmd_wait(0x1009, NULL, 0); /* Read_BD_ADDR */
    uint8_t name[248] = { 0 };
    strcpy((char *)name, "Nintendo RVL-CNT-01");
    hci_cmd_wait(0x0C13, name, 248);
    uint8_t cod[3] = { 0x04, 0x25, 0x00 }; /* 0x002504 */
    hci_cmd_wait(0x0C24, cod, 3);
    uint8_t iac[7] = { 2, 0x00, 0x8B, 0x9E, 0x33, 0x8B, 0x9E }; /* LIAC, GIAC */
    hci_cmd_wait(0x0C3A, iac, 7);
    uint8_t zero = 0;
    hci_cmd_wait(0x0C20, &zero, 1); /* Authentication_Enable off */
    uint8_t pto[2] = { 0x00, 0x20 }; /* Page Timeout 0x2000 * 0.625 ms = 5.12 s */
    hci_cmd_wait(0x0C18, pto, 2);
    uint8_t lp[2] = { 0x05, 0x00 }; /* Default Link Policy: Role Switch + Sniff */
    hci_cmd_wait(0x080F, lp, 2);
    uint8_t scan = 0x00; /* off until sync */
    hci_cmd_wait(0x0C1A, &scan, 1);

    wifi_start();
    do_info();
    ESP_LOGI(TAG, "ready. cmds: sync | wake | info | forget | wifi <ssid> <password> | token <t>|clear");

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));
    char line[120];
    size_t n = 0;
    while (true) {
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(50)) == 1) {
            if (ch == '\r' || ch == '\n') {
                line[n] = 0;
                do_cmd(line);
                n = 0;
            } else if (n < sizeof(line) - 1) {
                line[n++] = (char)ch;
            }
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int64_t now = esp_timer_get_time();
        if (s_syncing && now > s_sync_until) {
            sync_end("timeout");
        } else if (s_syncing && now > s_sync_refresh) { /* re-arm scanning periodically during the window */
            s_sync_refresh = now + SYNC_REFRESH_US;
            set_scan(0x03);
        }
        wake_tick(now);
        xSemaphoreGive(s_lock);
    }
}
