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


extern QueueHandle_t imageServiceQueue;
extern SemaphoreHandle_t spiffsMutex;
extern void unifi_api_request_gt2k(esp_http_client_method_t method, char *path);

extern char *TAG; // = "Services";

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
		xQueueSend(imageServiceQueue, path, portMAX_DELAY);
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
