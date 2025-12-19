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
#define SKN_LCD_PIXEL_CLOCK_HZ     (20 * 1024 * 1024)
#define SKN_LCD_I80_BUS_WIDTH 16
#define SKN_BUFFER_FACTOR 100

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

// The pixel number in horizontal and vertical
#define SKN_LCD_H_RES              320
#define SKN_LCD_V_RES              480
// Bit number used to represent command and parameter
#define SKN_LCD_CMD_BITS           8
#define SKN_LCD_PARAM_BITS         8

#define SKN_LVGL_TICK_PERIOD_MS    2

esp_lcd_touch_handle_t tp;
esp_lcd_panel_io_handle_t tp_io_handle = NULL;

static void skn_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
	uint16_t touchpad_x[1] = {0};
	uint16_t touchpad_y[1] = {0};
	uint8_t touchpad_cnt = 0;

	/* Read touch controller data */
	esp_lcd_touch_read_data(drv->user_data);

	/* Get coordinates */
	bool touchpad_pressed = esp_lcd_touch_get_coordinates(
		drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

	if (touchpad_pressed && touchpad_cnt > 0) {
		data->point.x = touchpad_x[0];
		data->point.y = touchpad_y[0];
		data->state = LV_INDEV_STATE_PRESSED;
        printf("-->Touch: X=%3.0d,\tY=%3.0d\n", data->point.x, data->point.y);
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
}
/**
 * @brief i2c FT6236 initialization
 */
static lv_indev_t * touch_init(lv_disp_t *disp) {

	ESP_LOGI(TAG, "Initialize I2C");

	const i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 38,
		.scl_io_num = 39,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 400000,
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

	// Input device driver (Touch)
	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.disp = disp;
	indev_drv.read_cb = skn_lvgl_touch_cb;
	indev_drv.user_data = tp;

	return lv_indev_drv_register(&indev_drv);
}

static bool skn_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void skn_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void skn_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(SKN_LVGL_TICK_PERIOD_MS);
}

lv_disp_t *display_init() {
	static lv_disp_draw_buf_t
		disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
	static lv_disp_drv_t disp_drv; // contains callback functions

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
		.max_transfer_bytes =
			SKN_LCD_H_RES * SKN_BUFFER_FACTOR * sizeof(lv_color_t),
		.psram_trans_align = 64,
		.sram_trans_align = 4};
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
		.user_ctx = &disp_drv,
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
		io_handle, &panel_config, SKN_LCD_H_RES * 100 * sizeof(lv_color_t),
		&panel_handle));

	esp_lcd_panel_reset(panel_handle);
	esp_lcd_panel_init(panel_handle);
	esp_lcd_panel_swap_xy(panel_handle, false);
	esp_lcd_panel_invert_color(panel_handle, false);
	// the gap is LCD panel specific, even panels with the same driver IC, can
	// have different gap value
	esp_lcd_panel_set_gap(panel_handle, 0, 0);

	ESP_LOGI(TAG, "Turn on LCD backlight");
	gpio_set_level(SKN_PIN_NUM_BK_LIGHT, SKN_LCD_BK_LIGHT_ON_LEVEL);

	ESP_LOGI(TAG, "Initialize LVGL library");
	lv_init();
	// alloc draw buffers used by LVGL
	// it's recommended to choose the size of the draw buffer(s) to be at least
	// 1/10 screen sized
	lv_color_t *buf1 = heap_caps_malloc(
		SKN_LCD_H_RES * SKN_BUFFER_FACTOR * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf1);
	lv_color_t *buf2 = heap_caps_malloc(
		SKN_LCD_H_RES * SKN_BUFFER_FACTOR * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf2);
	// initialize LVGL draw buffers
	lv_disp_draw_buf_init(&disp_buf, buf1, buf2,
						  SKN_LCD_H_RES * SKN_BUFFER_FACTOR);

	ESP_LOGI(TAG, "Register display driver to LVGL");
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = SKN_LCD_H_RES;
	disp_drv.ver_res = SKN_LCD_V_RES;
	disp_drv.flush_cb = skn_lvgl_flush_cb;
	disp_drv.draw_buf = &disp_buf;
	disp_drv.user_data = panel_handle;
	lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
	// lv_disp_set_rotation(disp, LV_DISP_ROT_90);

	ESP_LOGI(TAG, "Install LVGL tick timer");
	// Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
	const esp_timer_create_args_t lvgl_tick_timer_args = {
		.callback = &skn_increase_lvgl_tick, .name = "lvgl_tick"};
	esp_timer_handle_t lvgl_tick_timer = NULL;
	ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer,
											 SKN_LVGL_TICK_PERIOD_MS * 1000));
	return disp;											 
}

void app_main(void)
{
	// Initialize the Display
	lv_disp_t *disp = display_init();

	// Initialize Touch Controller
	// lv_indev_t *touch_device = touch_init(disp);
	touch_init(disp);

	lv_demo_widgets();

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10));
		lv_timer_handler();
	}
}
