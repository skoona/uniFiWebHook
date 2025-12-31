/*
 * services.c
 */

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_lv_decoder.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "lvgl.h"
#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define BEEP_DURATION_MS 500

extern QueueHandle_t imageServiceQueue;
extern QueueHandle_t urlServiceQueue;
extern SemaphoreHandle_t spiffsMutex;

extern void unifi_api_request_gt2k(esp_http_client_method_t method, char *path);
extern void unifi_async_api_request(esp_http_client_method_t method, char *path);

extern void skn_beep();
extern esp_err_t fileList();
extern char *TAG; // = "Services";

void standBy(char *message) {
	lv_obj_t *standby;
	static lv_style_t style_red;

	if (message == NULL)
		return;

	skn_beep(BEEP_DURATION_MS);
	ESP_LOGI("Services", "standBy(): Enter...");

	lv_style_init(&style_red);
	lv_style_set_text_font(&style_red, &lv_font_montserrat_32);
	lv_style_set_bg_color(&style_red, lv_color_make(128, 128, 128));
	lv_style_set_bg_opa(&style_red, LV_OPA_COVER);
	lv_style_set_text_color(&style_red,lv_color_make(0xff, 0x00, 0x00));

	standby = lv_label_create(lv_screen_active());
	lv_label_set_text(standby, message);
	lv_obj_add_style(standby, &style_red, 0);
	lv_obj_center(standby);

	ESP_LOGI("Services", "standBy(): Exit...");
}

esp_err_t writeBinaryImageFile(char *path, void *buffer, int bufLen) {
	uint written = 0;
	int event_file = 0;
	ESP_LOGI("Services", "writeBinaryImageFile(): Proceeding with: %s", path);

	if (xSemaphoreTake(spiffsMutex, portMAX_DELAY) == pdTRUE) {
		event_file = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
		if (event_file == -1) {
			ESP_LOGE("Services", "Failed to open %s file for writing", path);
			xSemaphoreGive(spiffsMutex);
			return ESP_FAIL;
		} else {
			written = write(event_file, buffer, bufLen);
			close(event_file);
			ESP_LOGI("Services", "File written, name: %s, bytes: %d", path, written);
		}
		xSemaphoreGive(spiffsMutex);
		// Create image
		vTaskDelay(pdMS_TO_TICKS(100));
		xQueueSend(imageServiceQueue, path, 0);
	}
	xSemaphoreGive(spiffsMutex);
	return ESP_OK;
}
void vUrlServiceTask(void *pvParameters) {
	char url[288];		// Used to receive data
	BaseType_t xReturn; // Used to receive return value
	QueueHandle_t urlQueue = pvParameters;
	ESP_LOGI("URLTask", "URLTask(): Initializing...");
	vTaskDelay(pdMS_TO_TICKS(10000));

	while (1) {
		xReturn = xQueueReceive(urlQueue, url, portMAX_DELAY);
		if (xReturn == pdTRUE) {
			ESP_LOGI("URLTask", "Received URL: %s", url);
			unifi_api_request_gt2k(HTTP_METHOD_GET, url);
		} else {
			ESP_LOGE("URLTask", "Cannot process URL: %s", url);
		}
		ESP_LOGI("URLTask", "URLTask(): Processing complete...");
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
