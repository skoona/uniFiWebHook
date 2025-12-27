/* ESP HTTP Client

*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include <cJSON.h>

static const char *TAG = "REQUESTER";
extern esp_err_t writeBinaryImageFile(char *path, void *buffer, int bufLen);
extern void standBy(char *message);

static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
	static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    static bool image = false;   // Content type flag
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
			if(strcmp(evt->header_value, "image/jpeg") == 0) {
                image = true;
            }
			ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s",
					evt->header_key, evt->header_value);
			break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                int copy_len = 0;
                int content_len = esp_http_client_get_content_length(evt->client);
                if (output_buffer == NULL) {
                    // We initialize output_buffer with 0 because it is used by strlen() and similar functions therefore should be null terminated.
                    output_buffer = (char *) calloc(content_len + 4, sizeof(char)); 
                    output_len = 0;
                    if (output_buffer == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                copy_len = MIN(evt->data_len, (content_len - output_len));
                if (copy_len) {
                    memcpy(output_buffer + output_len, evt->data, copy_len);
                }               
                output_len += copy_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                if (image){
					ESP_LOGI(TAG, "Image Received: [L=%d] Content Type:%s\n",
							 output_len, image ? "True":"False");
					image = false;
					char *path = "/spiffs/requester_event.jpg";
					esp_err_t rc = writeBinaryImageFile(path, output_buffer, output_len); 
					free(output_buffer);
					output_buffer = NULL;
					output_len = 0;
                    image = false;
					return rc;
				}

				cJSON *json = cJSON_Parse(output_buffer);
				if (json == NULL) {
					ESP_LOGE(TAG, "cJSON_Parse Failed: [L=%d]%s\n",
							 output_len, output_buffer);
					free(output_buffer);
					output_buffer = NULL;
					output_len = 0;
					return ESP_FAIL;
				}
				char *json_string = cJSON_Print(json);
				if (json_string==NULL) {
					ESP_LOGE(
						TAG,
						"cJSON_Print Failed: [L=%d]\n",
						output_len);
					cJSON_Delete(json);
					free(output_buffer);
					output_buffer = NULL;
					output_len = 0;
					return ESP_FAIL;
				}
				int json_len = strlen(json_string);

				ESP_LOGI(TAG, "Notification Received: [L=%d][R=%d]%s\n",
						 output_len, json_len, json_string);

				cJSON_free(json_string);
				cJSON_Delete(json);
				free(output_buffer);
                output_buffer = NULL;
			}
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGE(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGE(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

void unifi_async_api_request(esp_http_client_method_t method, char *path) {
	esp_http_client_config_t config = {
        .url = path,
		.method = method,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
		.skip_cert_common_name_check = true,
        .event_handler = _http_event_handler,
        .is_async = true,
        .timeout_ms = 5000,
    };

	ESP_LOGI(TAG, "HTTPS async requests =>");
    esp_http_client_handle_t client = esp_http_client_init(&config);
	esp_http_client_set_header(client, "X-API-KEY", CONFIG_PROTECT_API_TOKEN);

    esp_err_t err;
    while (1) {
        err = esp_http_client_perform(client);
        if (err != ESP_ERR_HTTP_EAGAIN) {
            break;
        }
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void unifi_api_request_gt2k(esp_http_client_method_t method, char *path) {
	esp_http_client_config_t config = {
        .url = path,
		.method = method,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
		.skip_cert_common_name_check = true,
        .event_handler = _http_event_handler,
    };
	standBy("Please StandBy...");

	ESP_LOGI(TAG, "HTTPS request with hostname and path => %s\n", path); // CONFIG_EXAMPLE_HTTP_ENDPOINT);
    esp_http_client_handle_t client = esp_http_client_init(&config);
	esp_http_client_set_header(client, "X-API-KEY", CONFIG_PROTECT_API_TOKEN);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int64_t len = esp_http_client_get_content_length(client);
		ESP_LOGI(TAG, "HTTPS Status(d) = %d, Length = %d\n",
                esp_http_client_get_status_code(client), len);
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

