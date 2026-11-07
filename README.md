# wiiu-wake-esp32

Wake a Wii U from standby with an ESP32, from a web page, over HTTP or over a serial port.

The ESP32 pairs with the console like a Wii Remote and then wakes it the way a real remote does when you press its power button. Afterwards it checks that the console came up. It does nothing else: it cannot act as a controller and sends no button input.

## Hardware

An original ESP32 (for example an ESP32-WROOM-32 dev board) with 4 MB of flash. Other variants such as the ESP32-S3 or ESP32-C3 do not have Bluetooth Classic and will not work.

## Building

Docker is the recommended way to build. Run this in the project folder:

PowerShell (Windows):

```powershell
docker run --rm -v ${PWD}:/project -w /project espressif/idf:v5.4.4 idf.py build
```

bash (Linux, macOS):

```bash
docker run --rm -v "$PWD":/project -w /project espressif/idf:v5.4.4 idf.py build
```

The binaries end up in `build/`:

| Offset | File |
|---|---|
| `0x1000` | `build/bootloader/bootloader.bin` |
| `0x8000` | `build/partition_table/partition-table.bin` |
| `0x10000` | `build/wiiu-wake-esp32.bin` |

## Flashing

You only need to flash once. Install [esptool](https://github.com/espressif/esptool) with `pip install esptool` and replace `COM14` with your serial port (for example `/dev/ttyUSB0` on Linux):

```bash
python -m esptool --chip esp32 -p COM14 -b 460800 write-flash --flash-mode dio --flash-freq 40m --flash-size 4MB 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/wiiu-wake-esp32.bin
```

Flashing again keeps the settings and the pairing.

## Setup

Configure Wi-Fi once over the serial port (115200 baud). You can use any serial monitor or the included `serialcmd.py`, which needs `pyserial` and does not reset the board. `--wifi` asks for the name and password.

```bash
python serialcmd.py --port COM14 --seconds 15 --wifi
```

After `wifi` the board restarts and prints its IP address. In a serial monitor, use `wifi "My Network" "My Password"`. Put each value in double quotes if it contains spaces, and escape `"` and `\` inside quotes with a backslash.

### Setting a token

Without a token, anyone on your network can wake the console, start pairing and delete the pairing. Any website opened in a browser on your network could also send these requests. The HTTP interface is plain HTTP and is meant for a trusted local network only. Do not expose it to the internet.

To require a token for HTTP requests, use the `token` command:

```bash
python serialcmd.py --port COM14 "token MySecretToken"
```

The value of `token` is hidden in the output, but may end up in your shell history.

## Usage

Open `http://<esp32-ip>/` in a browser. The page shows the current status and has **Wake** and **Sync** buttons. If a token is set, it asks for it once and remembers it in that browser.

To pair, press **Sync** and then the red SYNC button on the console within 120 seconds. After that, **Wake** turns the console on.

The same works from scripts:

```bash
curl -X POST -H "Authorization: Bearer MySecretToken" http://<esp32-ip>/sync
curl -X POST -H "Authorization: Bearer MySecretToken" http://<esp32-ip>/wake
```

Both also work over the serial port:

```bash
python serialcmd.py --port COM14 --seconds 130 "sync"
python serialcmd.py --port COM14 "wake"
```

## HTTP API

If a token is set, all endpoints need the header `Authorization: Bearer <token>` and answer `401` otherwise.

| Endpoint | Description | Responses |
|---|---|---|
| `GET /` | Web page. Needs no token, the data behind it does | `200` |
| `POST /wake` | Wake the paired console | `202`, or `409` if nothing is paired or the device is busy |
| `POST /sync` | Open the sync window for 120 seconds. It closes early once pairing succeeds | `202`, or `409` during a wake |
| `POST /forget` | Delete the stored pairing | `200`, or `409` during a wake |
| `GET /info` | Status as JSON | `200` |

Example response of `GET /info`:

```json
{
  "bdaddr": "XX:XX:XX:XX:XX:XX",
  "console": "XX:XX:XX:XX:XX:XX",
  "paired": true,
  "connected": false,
  "syncing": false,
  "sync_left_s": 0,
  "last_sync": "paired",
  "ip": "192.168.1.50",
  "rssi": -54,
  "uptime_ms": 200000,
  "wake_state": "on",
  "last_wake": { "time_ms": 123456, "result": "done" }
}
```

`wake_state` is one of:

| Value | Meaning |
|---|---|
| `idle` | No wake since boot, or the last one failed to connect |
| `waking` | Sending the wake signal |
| `verifying` | Signal sent, waiting for the console to come up |
| `on` | The console answered after the last wake |
| `unconfirmed` | The console did not answer within 60 seconds. It may still be starting, or it may be off |

`last_wake.result` is one of `none`, `paging`, `done` or `failed_0xNN` (HCI status code). `done` only means that the wake signal was sent. `time_ms` is the time since boot. `last_sync` is one of `none`, `open`, `paired` or `timeout`. `rssi` is the Wi-Fi signal strength in dBm, or 0 when not connected.

## Serial commands

| Command | Description |
|---|---|
| `sync`, `wake`, `info`, `forget` | Same as the HTTP API |
| `wifi "<ssid>" "<password>"` | Save Wi-Fi credentials and restart. Quotes are needed for values with spaces |
| `token <value>`, `token clear` | Set or remove the HTTP token |

## How it works

**Waking.** The firmware pages the console and, as soon as the connection is up, disconnects with reason `0x15` (remote device terminated connection due to power off). It does not start authentication or L2CAP itself.

**Verifying.** To check whether the console started, the firmware connects like a controller about 25 seconds after the wake signal, authenticates and requests the HID control channel. The console's Bluetooth module accepts the connection even in standby, but answers on L2CAP only once the console is running. On an answer the firmware disconnects with reason `0x13`. Otherwise it retries until 60 seconds after the wake. The console briefly sees a connecting controller during this.

**Pairing.** The firmware uses the name `Nintendo RVL-CNT-01`, class of device `0x002504` and the host address reversed as PIN. It answers the SDP queries with the responses of a real Wii Remote and handles only the HID reports that the console needs to finish the setup.

## Limitations

Tested on one Wii U, with and without Bloopair.

- Only one pairing is stored.
- While the sync window is open, any Bluetooth device nearby could pair and replace the stored console.
- Console status `unconfirmed` does not mean the wake failed: a slow start can take longer than the check.
- HTTP is unencrypted, so the token is sent in plain text.

## Credits

- [WiimoteEmulator](https://github.com/rnconrad/WiimoteEmulator) by rnconrad: the SDP responses
- [bluubomb](https://github.com/GaryOderNichts/bluubomb) by GaryOderNichts, based on WiimoteEmulator: the write-up on pairing with a Wii U
- [Bloopair](https://github.com/GaryOderNichts/Bloopair) by GaryOderNichts: the insight into how the console handles controllers
- [WiiBrew](https://wiibrew.org/wiki/Wiimote): Wii Remote protocol documentation, including the power button behaviour and disconnect reason `0x15`

## License

This project's code is licensed under the [GNU General Public License v2.0](LICENSE) (GPL-2.0-only).

The SDP record tables in `main/main.c` are byte-identical to those in WiimoteEmulator and bluubomb. Neither project carries a license.

## AI disclaimer

The code and documentation of this project were written with AI assistance under human supervision. Review the code before you rely on it.
