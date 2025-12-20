/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_ili9488.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_ft6x36.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include <stdio.h>
#include "lv_demos.h"

static const char *TAG = "SKN";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The pixel number in horizontal and vertical
#define SKN_LCD_H_RES              320
#define SKN_LCD_V_RES              480
#define SKN_LCD_PIXEL_CLOCK_HZ     (20 * 1024 * 1024)
#define SKN_LCD_I80_BUS_WIDTH      16
#define SKN_LCD_FACTOR             24
#define SKN_LVGL_PRIORITY           4
#define SKN_LVGL_STACK_SZ        8192

// -- LV_MEM = 56K
#define SKN_COLOR_BYTE_SZ    sizeof(lv_color_t)
#define SKN_TRANSFER_BUFF_SZ (SKN_LCD_H_RES * (SKN_LCD_FACTOR+2) * SKN_COLOR_BYTE_SZ)
#define SKN_COLOR_I80_SZ     (SKN_LCD_H_RES * SKN_LCD_FACTOR)
#define SKN_DRAW_PIXEL_CNT   (SKN_COLOR_I80_SZ * SKN_COLOR_BYTE_SZ)
#define SKN_DRAW_BUFF_SZ     SKN_DRAW_PIXEL_CNT

#define SKN_LCD_BK_LIGHT_ON_LEVEL  1
#define SKN_LCD_BK_LIGHT_OFF_LEVEL !SKN_LCD_BK_LIGHT_ON_LEVEL
#define SKN_PIN_NUM_DATA0          47
#define SKN_PIN_NUM_DATA1          21
#define SKN_PIN_NUM_DATA2          14
#define SKN_PIN_NUM_DATA3          13
#define SKN_PIN_NUM_DATA4          12
#define SKN_PIN_NUM_DATA5          11
#define SKN_PIN_NUM_DATA6          10
#define SKN_PIN_NUM_DATA7          9
#define SKN_PIN_NUM_DATA8          3
#define SKN_PIN_NUM_DATA9          8
#define SKN_PIN_NUM_DATA10         16
#define SKN_PIN_NUM_DATA11         15
#define SKN_PIN_NUM_DATA12         7
#define SKN_PIN_NUM_DATA13         6
#define SKN_PIN_NUM_DATA14         5
#define SKN_PIN_NUM_DATA15         4

#define SKN_PIN_NUM_PCLK           18
#define SKN_PIN_NUM_CS             -1
#define SKN_PIN_NUM_DC             45
#define SKN_PIN_NUM_RST            48
#define SKN_PIN_NUM_BK_LIGHT       46

// Bit number used to represent command and parameter
#define SKN_LCD_CMD_BITS           8
#define SKN_LCD_PARAM_BITS         8

#define SKN_LVGL_TICK_PERIOD_MS    2

void skn_lvgl_touch_cb(lv_indev_t *drv, lv_indev_data_t *data) {
	uint8_t touchpad_cnt = 0;
	esp_lcd_touch_point_data_t touch_data;

	/* Read touch controller data */
	esp_lcd_touch_read_data(drv->user_data);

	esp_err_t err = esp_lcd_touch_get_data(drv->user_data, &touch_data, &touchpad_cnt, 1);
	if (err == ESP_OK && touchpad_cnt > 0) {
		data->point.x = touch_data.x;
		data->point.y = touch_data.y;
		data->state = LV_INDEV_STATE_PRESSED;
        printf("-->Touch: X=%3.0ld,\tY=%3.0ld\n", data->point.x, data->point.y);
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
}
bool skn_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
	lv_display_t *disp_driver = (lv_display_t *)user_ctx;
	lv_disp_flush_ready(disp_driver);
    return false;
}
void skn_lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *color_map)
{
	esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) display->user_data;
	int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
	lv_display_flush_ready(display);
}
uint32_t skn_tick_cb(void) { return (uint32_t)esp_timer_get_time() / 1000ULL; }

void skn_touch_init() {
	esp_lcd_panel_io_handle_t tp_io_handle = NULL;
	esp_lcd_touch_handle_t tp;

	ESP_LOGI(TAG, "Initialize I2C");

	const i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 38,
		.scl_io_num = 39,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};
	/* Initialize I2C */
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));

	esp_lcd_panel_io_i2c_config_t tp_io_config =
		ESP_LCD_TOUCH_IO_I2C_FT6x36_CONFIG();

	ESP_LOGI(TAG, "Initialize touch IO (I2C)");

	/* Touch IO handle */
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(
		(esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle));

	esp_lcd_touch_config_t tp_cfg = {
		.x_max = SKN_LCD_V_RES,
		.y_max = SKN_LCD_H_RES,
		.rst_gpio_num = -1,
		.int_gpio_num = -1,
		.levels =
			{
				.reset = 0,
				.interrupt = 0,
			},
		.flags =
			{
				.swap_xy = 0,
				.mirror_x = 0,
				.mirror_y = 0, // 0 | 1
			},
	};

	/* Initialize touch */
	ESP_LOGI(TAG, "Initialize touch controller FT6x36");
	ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft6x36(tp_io_handle, &tp_cfg, &tp));

	/*Create an input device for touch handling*/
	lv_indev_t *indev = lv_indev_create();
	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(indev, skn_lvgl_touch_cb);
	lv_indev_set_user_data(indev, tp);
}
void skn_display_init(void *pvParameters) {

	static lv_display_t *display; // contains callback functions

	ESP_LOGI(TAG, "Turn off LCD backlight");
	gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT,
									.pin_bit_mask =
										1ULL << SKN_PIN_NUM_BK_LIGHT};
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
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

	ESP_LOGI(TAG, "Install LCD driver of ili9488");
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = SKN_PIN_NUM_RST,
		.color_space = ESP_LCD_COLOR_SPACE_RGB,
		.bits_per_pixel = 16,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(
		io_handle, &panel_config, SKN_DRAW_BUFF_SZ, &panel_handle));

	esp_lcd_panel_reset(panel_handle);
	esp_lcd_panel_init(panel_handle);
	esp_lcd_panel_swap_xy(panel_handle, false);
	esp_lcd_panel_invert_color(panel_handle, false);
	esp_lcd_panel_set_gap(panel_handle, 0, 0);

	ESP_LOGI(TAG, "Turn on LCD backlight");
	gpio_set_level(SKN_PIN_NUM_BK_LIGHT, SKN_LCD_BK_LIGHT_ON_LEVEL);

	ESP_LOGI(TAG, "Initialize LVGL library");
	lv_init();

	lv_tick_set_cb(skn_tick_cb);

	ESP_LOGI(TAG, "Register display driver to LVGL");
	display = lv_display_create(SKN_LCD_H_RES, SKN_LCD_V_RES);

	// initialize LVGL draw buffers
	printf("Color Sz: %d\tlv_color_t: %d\tDraw buffer: %'.0u\tTransfer "
			"buffer: %'.0u\tAlternate SZ: %'.0u\n",
			lv_color_format_get_size(lv_display_get_color_format(display)),
			sizeof(lv_color_t), 
			SKN_DRAW_BUFF_SZ, 
			SKN_TRANSFER_BUFF_SZ,
			SKN_COLOR_I80_SZ
		);

	uint8_t *buf1 = heap_caps_malloc(SKN_DRAW_BUFF_SZ, MALLOC_CAP_DMA | MALLOC_CAP_32BIT); 
	assert(buf1); 
	uint8_t *buf2 = heap_caps_malloc(SKN_DRAW_BUFF_SZ, MALLOC_CAP_DMA | MALLOC_CAP_32BIT); 
	assert(buf2); 
	lv_display_set_buffers(display, buf1, buf2, SKN_COLOR_I80_SZ, LV_DISPLAY_RENDER_MODE_PARTIAL);		

	lv_display_set_flush_cb(display, skn_lvgl_flush_cb);
	lv_display_set_user_data(display, panel_handle);

	// lv_disp_set_rotation(display, LV_DISPLAY_ROTATION_270);

	// Initialize Touch Controller
	skn_touch_init();

	lv_demo_widgets();

	while (1) {
		lv_timer_handler();
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
void app_main(void) {

	xTaskCreatePinnedToCore(
		skn_display_init, 
		"SKN Display", 
		SKN_LVGL_STACK_SZ, NULL, 
		SKN_LVGL_PRIORITY, NULL, 
		1);
}
