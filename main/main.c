/*
 * LVGL 9+, ILI9488, and Touch FT6436
 * skoona@gmail.com
 * 12/29/2025
 * LVGL Template
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SKN_LVGL_PRIORITY 8
#define SKN_LVGL_STACK_SZ 8192

void skn_display_task(void *pvParameters);

void app_main(void) {

	xTaskCreatePinnedToCore(
		skn_display_task, 
		"SKN Display", 
		SKN_LVGL_STACK_SZ, NULL, 
		SKN_LVGL_PRIORITY, NULL, 
		tskNO_AFFINITY);
}
