set(SOURCES main.c lvglDisplay.c ui_page01.c http_listener.c http_requester.c)
set(COMPONENT_USED nvs_flash esp_netif esp-tls esp_event esp_wifi esp_http_server esp_http_client spiffs esp_timer)
idf_component_register(
    SRCS ${SOURCES}
	PRIV_REQUIRES ${COMPONENT_USED}
    INCLUDE_DIRS ".")
