/*
 * rgb_panel.c
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_lcd_ili9488.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_ft6x36.h"
#include "esp_log.h"
#include "esp_lv_decoder.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "lvgl.h"
#include <stdio.h>

extern char *TAG; //  = "Display";

#define SKN_LCD_H_RES 320
#define SKN_LCD_V_RES 480
#define SKN_LCD_PIXEL_CLOCK_HZ (20 * 1024 * 1024)
#define SKN_LCD_I80_BUS_WIDTH 16
#define SKN_LCD_FACTOR 16

// -- LV_MEM = 56K
#define SKN_COLOR_BYTE_SZ sizeof(uint16_t)
#define SKN_I80_PIXELS_CNT (SKN_LCD_H_RES * SKN_LCD_V_RES)
#define SKN_DRAW_BUFF_SZ    ((SKN_I80_PIXELS_CNT / SKN_LCD_FACTOR) * SKN_COLOR_BYTE_SZ ) // (SKN_LCD_H_RES * SKN_LCD_FACTOR * SKN_COLOR_BYTE_SZ)
#define SKN_I80_COLOR_BUFF_SZ (SKN_DRAW_BUFF_SZ / 4 )
#define SKN_TRANSFER_BUFF_SZ  (SKN_DRAW_BUFF_SZ  + 32)

#define SKN_LCD_BK_LIGHT_ON_LEVEL 1
#define SKN_LCD_BK_LIGHT_OFF_LEVEL !SKN_LCD_BK_LIGHT_ON_LEVEL
#define SKN_PIN_NUM_DATA0 47
#define SKN_PIN_NUM_DATA1 21
#define SKN_PIN_NUM_DATA2 14
#define SKN_PIN_NUM_DATA3 13
#define SKN_PIN_NUM_DATA4 12
#define SKN_PIN_NUM_DATA5 11
#define SKN_PIN_NUM_DATA6 10
#define SKN_PIN_NUM_DATA7 9
#define SKN_PIN_NUM_DATA8 3
#define SKN_PIN_NUM_DATA9 8
#define SKN_PIN_NUM_DATA10 16
#define SKN_PIN_NUM_DATA11 15
#define SKN_PIN_NUM_DATA12 7
#define SKN_PIN_NUM_DATA13 6
#define SKN_PIN_NUM_DATA14 5
#define SKN_PIN_NUM_DATA15 4

#define SKN_PIN_NUM_PCLK 18
#define SKN_PIN_NUM_CS -1
#define SKN_PIN_NUM_DC 45
#define SKN_PIN_NUM_RST 48
#define SKN_PIN_NUM_BK_LIGHT 46

// Bit number used to represent command and parameter
#define SKN_LCD_CMD_BITS 8
#define SKN_LCD_PARAM_BITS 8

#define SKN_LVGL_TICK_PERIOD_MS 5

const uint32_t panel_Hres = SKN_LCD_H_RES;
const uint32_t panel_Vres = SKN_LCD_V_RES;
extern QueueHandle_t imageServiceQueue;
extern SemaphoreHandle_t spiffsMutex;

extern void skn_touch_event_handler(lv_event_t *e);
extern void ui_skoona_page(lv_obj_t *scr);
extern void skn_touch_init();
extern void logMemoryStats(char *message);
void standBy(char *message);

// A custom function to be called by an LVGL timer
void skn_image_handler_cb(lv_timer_t *timer) {
	
	char path[256] = {0}; // Used to receive data
	char image[288] = {"S:"};
	BaseType_t xReturn; // Used to receive return value
	QueueHandle_t ImageQueue = (QueueHandle_t)timer->user_data;
	static lv_obj_t *currentImage = NULL;
	static lv_style_t image_style;
	uint32_t startTime = esp_timer_get_time();
	
	lv_style_init(&image_style);
	lv_style_set_y(&image_style, 2);
	lv_style_set_max_height(&image_style, 318);
	lv_style_set_x(&image_style, 2);
	lv_style_set_max_width(&image_style, 478);
	
	xReturn = xQueueReceive(ImageQueue, path, 0);
	if (xReturn == pdTRUE) {
		ESP_LOGI("ImageService", "skn_image_handler_cb() Entered...");
		if (xSemaphoreTake(spiffsMutex, portMAX_DELAY) == pdTRUE) {

			standBy("Please StandBy...");

			ESP_LOGI("ImageService", "Received image file: %s", path);

			currentImage = lv_img_create(lv_screen_active());
			if (currentImage != NULL) {
				lv_obj_set_style_bg_color(lv_screen_active(), lv_color_white(), 0);

				sprintf(image, "S:%s", path);
				lv_img_set_src(currentImage, image);
				lv_obj_add_style(currentImage, &image_style, 0);

				lv_image_set_inner_align(currentImage, LV_IMAGE_ALIGN_STRETCH);

				ESP_LOGI("ImageService", "Completed processing for image file: %s", path);
			} else {
				ESP_LOGE("ImageService", "Failed to create image for %s", path);
			}
			xSemaphoreGive(spiffsMutex);
		}
		ESP_LOGI("ImageService", "skn_image_handler_cb() Exiting... DurationMS: %ld",(esp_timer_get_time() - startTime) );
	}
}
static bool skn_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
										esp_lcd_panel_io_event_data_t *edata,
										void *user_ctx) {
	lv_display_t *disp_driver = (lv_display_t *)user_ctx;
	lv_disp_flush_ready(disp_driver);
	return false;
}
static void skn_lvgl_flush_cb(lv_display_t *display, const lv_area_t *area,
							  uint8_t *color_map) {
	esp_lcd_panel_handle_t panel_handle =
		(esp_lcd_panel_handle_t)display->user_data;

	int offsetx1 = area->x1;
	int offsetx2 = area->x2;
	int offsety1 = area->y1;
	int offsety2 = area->y2;
	esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1,
							  offsety2 + 1, color_map);
	lv_display_flush_ready(display);
}
static uint32_t skn_tick_cb(void) {
	return (uint32_t)esp_timer_get_time() / 1000ULL;
}
void skn_lvgl_touch_cb(lv_indev_t *drv, lv_indev_data_t *data) {
	uint8_t touchpad_cnt = 0;
	esp_lcd_touch_point_data_t touch_data;
	esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;

	/* Read touch controller data */
	esp_lcd_touch_read_data(tp);

	esp_err_t err = esp_lcd_touch_get_data(tp, &touch_data, &touchpad_cnt, 1);
	if (err == ESP_OK && touchpad_cnt > 0) {
		data->point.x = touch_data.x;
		data->point.y = touch_data.y;
		data->state = LV_INDEV_STATE_PRESSED;
		printf("-->Touch: X=%3.0ld,\tY=%3.0ld\n", data->point.x, data->point.y);
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
}
void vDisplayServiceTask(void *pvParameters) {

	static lv_display_t *display; // contains callback functions

	ESP_LOGI(TAG, "Turn off LCD backlight");
	gpio_config_t bk_gpio_config = {
		.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << SKN_PIN_NUM_BK_LIGHT};
	ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
	gpio_set_level(SKN_PIN_NUM_BK_LIGHT, SKN_LCD_BK_LIGHT_OFF_LEVEL);

	ESP_LOGI(TAG, "Initialize Intel 8080 bus");
	esp_lcd_i80_bus_handle_t i80_bus = NULL;
	esp_lcd_i80_bus_config_t bus_config = {
		.clk_src = LCD_CLK_SRC_DEFAULT,
		.dc_gpio_num = SKN_PIN_NUM_DC,
		.wr_gpio_num = SKN_PIN_NUM_PCLK,
		.data_gpio_nums =
			{
				SKN_PIN_NUM_DATA0,
				SKN_PIN_NUM_DATA1,
				SKN_PIN_NUM_DATA2,
				SKN_PIN_NUM_DATA3,
				SKN_PIN_NUM_DATA4,
				SKN_PIN_NUM_DATA5,
				SKN_PIN_NUM_DATA6,
				SKN_PIN_NUM_DATA7,
				SKN_PIN_NUM_DATA8,
				SKN_PIN_NUM_DATA9,
				SKN_PIN_NUM_DATA10,
				SKN_PIN_NUM_DATA11,
				SKN_PIN_NUM_DATA12,
				SKN_PIN_NUM_DATA13,
				SKN_PIN_NUM_DATA14,
				SKN_PIN_NUM_DATA15,

			},
		.bus_width = SKN_LCD_I80_BUS_WIDTH,
		.max_transfer_bytes = SKN_TRANSFER_BUFF_SZ,
		.psram_trans_align = 0,
		.sram_trans_align = 0,
	};
	ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

	esp_lcd_panel_io_handle_t io_handle = NULL;
	esp_lcd_panel_io_i80_config_t io_config = {
		.cs_gpio_num = SKN_PIN_NUM_CS,
		.pclk_hz = SKN_LCD_PIXEL_CLOCK_HZ,
		.trans_queue_depth = 10,
		.dc_levels =
			{
				.dc_idle_level = 0,
				.dc_cmd_level = 0,
				.dc_dummy_level = 0,
				.dc_data_level = 1,
			},
		.on_color_trans_done = skn_notify_lvgl_flush_ready,
		.user_ctx = &display,
		.lcd_cmd_bits = SKN_LCD_CMD_BITS,
		.lcd_param_bits = SKN_LCD_PARAM_BITS,
		.flags =
			{
				.swap_color_bytes = false, // !CONFIG_LV_COLOR_16_SWAP,
				.cs_active_high = false,
				.reverse_color_bits = false,
			},
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

	ESP_LOGI(TAG, "Install LCD driver of ili9488");
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = SKN_PIN_NUM_RST,
		.color_space = ESP_LCD_COLOR_SPACE_BGR,
		.bits_per_pixel = 16,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_config, SKN_I80_COLOR_BUFF_SZ, &panel_handle));

	esp_lcd_panel_reset(panel_handle);
	esp_lcd_panel_init(panel_handle);
	esp_lcd_panel_invert_color(panel_handle, false);
	esp_lcd_panel_set_gap(panel_handle, 0, 0);

	/*
	 * Rotate LCD display
	 * - following lv_display_create has x/y/ reversed
	 */
	esp_lcd_panel_swap_xy(panel_handle, true);
	esp_lcd_panel_mirror(panel_handle, true, false);

	ESP_LOGI(TAG, "Turn on LCD backlight");
	gpio_set_level(SKN_PIN_NUM_BK_LIGHT, SKN_LCD_BK_LIGHT_ON_LEVEL);

	ESP_LOGI(TAG, "Initialize LVGL library");
	lv_init();
	lv_tick_set_cb(skn_tick_cb);

	ESP_LOGI(TAG, "Register display driver to LVGL");
	display = lv_display_create(SKN_LCD_V_RES, SKN_LCD_H_RES);

	// initialize LVGL draw buffers
	printf("Color Sz: %d\tlv_color_t: %d\tDraw buffer: %'.0u\tTransfer "
		   "buffer: %'.0u\tPixel Cnt: %'.0u\n",
		   lv_color_format_get_size(lv_display_get_color_format(display)),
		   sizeof(lv_color_t), SKN_DRAW_BUFF_SZ, SKN_TRANSFER_BUFF_SZ,
		   SKN_I80_PIXELS_CNT);

	uint8_t *buf1 = heap_caps_malloc(SKN_DRAW_BUFF_SZ, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
	assert(buf1);
	uint8_t *buf2 = heap_caps_malloc(SKN_DRAW_BUFF_SZ, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
	assert(buf2);
	lv_display_set_buffers(display, buf1, buf2, SKN_DRAW_BUFF_SZ, LV_DISPLAY_RENDER_MODE_PARTIAL);

	lv_display_set_flush_cb(display, skn_lvgl_flush_cb);
	lv_display_set_user_data(display, panel_handle);
	
	skn_touch_init();

	esp_lv_decoder_handle_t decoder_handle = NULL;

	lv_lock();
		esp_lv_decoder_init(&decoder_handle); // Initialize this after lvgl starts
		lv_obj_t *scr = lv_obj_create(NULL);
		lv_screen_load(scr);

		ui_skoona_page(scr);
	lv_unlock();

	logMemoryStats("END Startup");

	lv_timer_create(skn_image_handler_cb, 3000, imageServiceQueue);

	while (1) {
		// lv_timer_handler();
		// vTaskDelay(pdMS_TO_TICKS(5));
		lv_timer_periodic_handler();
		// lv_lock();
		// lv_timer_handler_run_in_period(5);
		// lv_unlock();
	}
}
