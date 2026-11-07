idf_component_register(SRCS "main.c" INCLUDE_DIRS "." EMBED_TXTFILES "index.html" REQUIRES bt nvs_flash esp_driver_uart esp_timer esp_wifi esp_netif esp_event esp_http_server)
