set(SOURCES main.c services.c rgb_panel.c touch_panel.c page_intro.c listener.c requester.c network.c)
set(COMPONENT_USED nvs_flash esp_netif esp-tls esp_event esp_wifi esp_http_server esp_http_client spiffs esp_timer esp_psram) 
idf_component_register(
    SRCS ${SOURCES}
	PRIV_REQUIRES ${COMPONENT_USED}
    INCLUDE_DIRS ".")
