/* ESP HTTP Listener

*/

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "mbedtls/base64.h"
#include <cJSON.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "LISTENER";
extern QueueHandle_t urlQueue;
static QueueHandle_t serverQueue;
extern esp_err_t writeBinaryImageFile(char *path, void *buffer, int bufLen);
extern void standBy(char *message);
typedef struct _alarmRequest {
	char uri[64];
	char path[64];
	char content_type[32];
	size_t received_len; 
	char *content;
} AlarmRequest;

esp_err_t writeBase64Buffer(char *path, const unsigned char *input_buffer) {
	size_t input_len = strlen((const char *)input_buffer);
	unsigned char *output_buffer;
	size_t output_len;
	
	// Allocate memory for the output buffer
	output_len = ((input_len + 3) / 3) * 4 + 1;
	output_buffer = (unsigned char *)calloc(output_len, sizeof(unsigned char));
	if (output_buffer == NULL) {
		ESP_LOGE(TAG,"Failed to allocate [%d:%d] bytes for base64 output buffer for: %s",input_len, output_len, path);
		return ESP_FAIL;		
	}
	// 'data:image/jpeg;base64,' = 23 bytes
	int ret = mbedtls_base64_decode(output_buffer, output_len, &output_len, input_buffer, input_len);
	if (ret == 0) {
		// Decoding successful.
		ret = writeBinaryImageFile(path, output_buffer, output_len);
		
	} else {
		ESP_LOGE(TAG, "Failed to decode base64 contents for: %s", path);
		ret = ESP_FAIL;		
	}
	free(output_buffer); // Clean up memory
	return ret;	
}

/* Gather values from menuconfig
   - lookup device and generate URL
   - submit URL to cause image display
   - get constants from config.json in future
*/
esp_err_t handleAlarms(char *device) {
	char *alarm_id = NULL;

	if (strcmp(CONFIG_CAMERA_FRONT_DOOR_MAC, device) == 0) { // front door
		alarm_id = CONFIG_CAMERA_FRONT_DOOR_ID;
	} else if (strcmp(CONFIG_CAMERA_GARAGE_VIEW_MAC, device) == 0) { // Garage
		alarm_id = CONFIG_CAMERA_GARAGE_VIEW_ID;
	} else if (strcmp(CONFIG_CAMERA_KITCHEN_VIEW_MAC, device) ==
			   0) { // Kitchen and Dining
		alarm_id = CONFIG_CAMERA_KITCHEN_VIEW_ID;
	} else if (strcmp(CONFIG_CAMERA_SOUTH_VIEW_MAC, device) ==
			   0) { // South View
		alarm_id = CONFIG_CAMERA_SOUTH_VIEW_ID;
	} else if (strcmp(CONFIG_CAMERA_EAST_VIEW_MAC, device) == 0) { // East View
		alarm_id = CONFIG_CAMERA_EAST_VIEW_ID;
	} else {
		ESP_LOGE(TAG, "Unknown Device: %s", device);
		return ESP_FAIL;
	}

	char url[254];

	sprintf(url, "%s/%s/snapshot", CONFIG_PROTECT_API_ENDPOINT, alarm_id);
	xQueueSend(urlQueue, url, 10);
	return ESP_OK;
}

esp_err_t processAlarmResponse(char *path, cJSON * root, char *target) {
	cJSON *alarm = NULL;
	cJSON *triggers = NULL;
	cJSON *elem = NULL;
	cJSON *device = NULL;
	cJSON *thumbnail = NULL;
	char customPath[128];
	esp_err_t ret = ESP_OK;
	
	memset(target, 0, sizeof(target));

	alarm = cJSON_GetObjectItemCaseSensitive(root, "alarm");
	if (alarm == NULL) {
		ESP_LOGE(TAG, "Alarm element not found");
		return ESP_FAIL;
	}
	triggers = cJSON_GetObjectItemCaseSensitive(alarm, "triggers");
	if (triggers == NULL) {
		ESP_LOGE(TAG, "Triggers element not found");
		return ESP_FAIL;
	}	
	elem = cJSON_GetArrayItem(triggers, 0);
	if (elem == NULL) {
		ESP_LOGE(TAG, "First Element in triggers not found");
		return ESP_FAIL;
	}
	device = cJSON_GetObjectItemCaseSensitive(elem, "device");
	if (device == NULL || !cJSON_IsString(device) || (device->valuestring == NULL)) {
		ESP_LOGE(TAG, "Device element not found");
		return ESP_FAIL;
	}
	thumbnail = cJSON_GetObjectItemCaseSensitive(alarm, "thumbnail");
	if (thumbnail == NULL || !cJSON_IsString(thumbnail) || (thumbnail->valuestring == NULL)) {
		ESP_LOGE(TAG, "Thumbnail element not found");		
		ret = ESP_FAIL;
		strcpy(target, device->valuestring);
	} else {				
		char *pValue = thumbnail->valuestring;
		sprintf(customPath, "/spiffs/%s-Event.jpg", device->valuestring);
		return writeBase64Buffer( customPath, (const unsigned char *)&pValue[23]);
	}
	
	return ret;
}

esp_err_t handleWebhookResult(char *path, char *content, char *content_type, size_t bytes_received) {
	char device[32] = {'\0'};
	memset(device, 0,sizeof(device));

	cJSON *json = cJSON_Parse(content);
	if (json == NULL) {
		ESP_LOGI(TAG, "cJSON_Parse Failed: [L=%d]%s\n", bytes_received, content);
		return ESP_FAIL;
	}
	
	processAlarmResponse(path, json, device);
	if (strlen(device) > 0) { // Image was included so no seperate request vi HandleAlarms()
		handleAlarms(device);		
	}

	cJSON_Delete(json);

	return ESP_OK;
}

static void vServerRequestsTask(void *pvParameters) {
	AlarmRequest alarm = {0};		// Used to receive data
	BaseType_t xReturn; // Used to receive return value
	QueueHandle_t serverQueue = pvParameters;
	vTaskDelay(pdMS_TO_TICKS(1000));
	while (1) {
		xReturn = xQueueReceive(serverQueue, &alarm, pdMS_TO_TICKS(3000));
		if (xReturn == pdTRUE) {
			ESP_LOGI(TAG, "Processing HTTP_POST webhook from: %s", alarm.uri);
			handleWebhookResult(alarm.path, alarm.content, alarm.content_type,
								alarm.received_len);
			free(alarm.content);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

// URI handler for the root path "/"
esp_err_t root_get_handler(httpd_req_t *req) {
	httpd_resp_set_status(req, "200 OK");
    const char* resp_str = "Hello from ESP32-S3!";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* URI handler for UniFi Webhook
 - LISTENER: Notification Received:
	{
	“alarm”:	{
		“name”:	“Person”,
		“sources”:	[],
		“conditions”:	[{
				“condition”:	{
					“type”:	“is”,
					“source”:	“person”
				}
			}],
		“triggers”:	[{
				“key”:	“person”,
				“device”:	“70A7413F0FD7”,
				“zones”:	{
					“loiter”:	[],
					“zone”:	[],
					“line”:	[1]
				},
				“eventId”:	“690e50b200ba0503e40e9015”,
				“timestamp”:	1762545842310
			}, {
				“key”:	“person”,
				“device”:	“70A7413F0FD7”,  ---- MAC ADDRESS
				“zones”:	{
					“loiter”:	[],
					“zone”:	[1],
					“line”:	[]
				},
				“eventId”:	“690e50b2030c0503e40e9029”,
				“timestamp”:	1762545842808
			}],
		“eventPath”:	“/protect/events/event/690e50b200ba0503e40e9015”,
		“eventLocalLink”:“https://10.100.1.1/protect/events/event/690e50b200ba0503e40e9015”,
		"thumbnail":"data:image/jpeg;base64,...."
	},
	“timestamp”:	1762545843235
}
*/
esp_err_t unifi_cb(httpd_req_t *req) {
	char *content; 
	char *path = "/spiffs/listener_event.jpg";

	// Get the content length from the request headers
	size_t content_len = (req->content_len +4);
	size_t bytes_received = 0;
	size_t received_len = 0;
	size_t total_len = req->content_len;
	size_t remaining_sz = total_len;
	standBy("Please StandBy...");

	content = calloc(content_len, sizeof(char)); // allocate 512KB
	if (content == NULL) { // Error or connection closed
        httpd_resp_send_500(req); // alloc error
		ESP_LOGE(TAG, "Receive Buffer Allocation Failed, req->content_len: %d, content_len:%d", req->content_len, content_len);
        return ESP_FAIL;
    }

    // Read the request body into the buffer
	// Content-Type:application/json BytesReceived:5501 req->content_len:45337 Content:[{"alarm":
	while (received_len < total_len) {
		bytes_received = httpd_req_recv(req, &content[received_len], remaining_sz);
		if (bytes_received <= 0) { // Error or connection closed
			if (bytes_received == HTTPD_SOCK_ERR_TIMEOUT) {
				ESP_LOGE(TAG, "SOCKET Timeout Received. %s", req->uri);
				httpd_resp_send_408(req); // Request timeout
			}
			return ESP_FAIL;
		}
		received_len += bytes_received;
		remaining_sz = total_len - received_len;
		ESP_LOGI(TAG, "content_len:%d\treq->content_len:%d\ttotal_len:%d\tbytes_received:%d\treceived_len:%d\tremaining_sz:%d",
			content_len, req->content_len, total_len, bytes_received, received_len,remaining_sz);
	}
	// Might be binary image: check headers 
	// -- key=Content-Type, value=image/jpeg
	char content_type[32];
	httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type));

	// Null-terminate the received data for string manipulation
	content[received_len] = '\0';
	ESP_LOGI(TAG, "Content-Type:%s total_len:%d req->content_len:%d", content_type, total_len, req->content_len);

	AlarmRequest qAlarm = {0};
	strncpy(qAlarm.path, path, sizeof(qAlarm.path));
	strncpy(qAlarm.content_type, content_type, sizeof(qAlarm.content_type));
	strncpy(qAlarm.uri, req->uri, sizeof(qAlarm.uri));	
	qAlarm.received_len = received_len;
	qAlarm.content = content;

	// Send it to be processed
	xQueueSend(serverQueue, &qAlarm, pdMS_TO_TICKS(10));

	// Send a response back to the client
	httpd_resp_set_status(req, "204 OK");
	httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}


// Function to start the HTTP server
httpd_handle_t start_http_listener(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	// Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {

        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_ANY,
            .handler  = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

		// Webhook for all notifications
        httpd_uri_t unifi_cb_uri = {
            .uri      = "/notify",
            .method   = HTTP_POST,
            .handler  = unifi_cb,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &unifi_cb_uri);

		serverQueue = xQueueCreate(8, sizeof(AlarmRequest));
		if (serverQueue != NULL) {
			xTaskCreatePinnedToCore(vServerRequestsTask, "vServerRequestsTask",
									6144, serverQueue, 8, NULL,
									tskNO_AFFINITY);
		}
	}
    return server;
}

