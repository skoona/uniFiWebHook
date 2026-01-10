set(SOURCES main.c services.c rgb_panel.c page_intro.c listener.c requester.c)
set(COMPONENT_USED esp-tls esp_http_server esp_http_client spiffs esp_timer esp_psram) 
idf_component_register(
    SRCS ${SOURCES}
    REQUIRES wifi_network
	PRIV_REQUIRES ${COMPONENT_USED}
    INCLUDE_DIRS ".")
