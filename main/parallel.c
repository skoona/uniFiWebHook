/* parallel.c
*/
#include "esp_check.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_lcd_touch_ft6x36.h"
#include "esp_lvgl_port.h"
#include "lv_api_map_v8.h"
#include <esp_lcd_io_i80.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>

#include <lvgl.h>

/* LCD settings */
#define I80_LCD_LVGL_FULL_REFRESH           (0)
#define I80_LCD_LVGL_DIRECT_MODE            (0)
#define I80_LCD_LVGL_AVOID_TEAR             (1)
#define I80_LCD_RGB_BOUNCE_BUFFER_MODE      (1)
#define I80_LCD_DRAW_BUFF_DOUBLE            (0)
#define I80_LCD_DRAW_BUFF_HEIGHT            (100)
#define I80_LCD_RGB_BUFFER_NUMS             (2)
#define I80_LCD_RGB_BOUNCE_BUFFER_HEIGHT    (10)

#define I80_TOUCH_I2C_NUM       (0)
#define I80_TOUCH_I2C_CLK_HZ    (400000)

#define I80_LCD_RST_GPIO             -1
#define I80_LCD_CS_GPIO              -1
#define I80_LCD_RS_GPIO              45
#define I80_LCD_WR_GPIO              18
#define I80_LCD_RD_GPIO              48

#define I80_LCD_DATA_0               47
#define I80_LCD_DATA_1               21
#define I80_LCD_DATA_2               14
#define I80_LCD_DATA_3               13
#define I80_LCD_DATA_4               12
#define I80_LCD_DATA_5               11
#define I80_LCD_DATA_6               10
#define I80_LCD_DATA_7               9
#define I80_LCD_DATA_8               3
#define I80_LCD_DATA_9               8
#define I80_LCD_DATA_10              16
#define I80_LCD_DATA_11              15
#define I80_LCD_DATA_12              7
#define I80_LCD_DATA_13              6
#define I80_LCD_DATA_14              5
#define I80_LCD_DATA_15              4

#define I80_LCD_DATA_GPIO {          \
    I80_LCD_DATA_0,                  \
    I80_LCD_DATA_1,                  \
    I80_LCD_DATA_2,                  \
    I80_LCD_DATA_3,                  \
    I80_LCD_DATA_4,                  \
    I80_LCD_DATA_5,                  \
    I80_LCD_DATA_6,                  \
    I80_LCD_DATA_7,                  \
    I80_LCD_DATA_8,                  \
    I80_LCD_DATA_9,                  \
    I80_LCD_DATA_10,                 \
    I80_LCD_DATA_11,                 \
    I80_LCD_DATA_12,                 \
    I80_LCD_DATA_13,                 \
    I80_LCD_DATA_14,                 \
    I80_LCD_DATA_15,                 \
}

#define I80_LCD_PIXEL_CLK_HZ         80000000 // 10000000
#define I80_LCD_H_RES                320
#define I80_LCD_V_RES                480
#define I80_LCD_CMD_BITS             8
#define I80_LCD_PARAM_BITS           8
#define I80_LCD_BUS_WIDTH            16
#define I80_LCD_DMA_BURST_SIZE       64

#define I80_LCD_LVGL_DRAW_BUF_LINES  100

static const char *TAG = "parallel";
/* LCD IO and panel */
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;
/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

extern void ui_skoona_page(lv_obj_t *scr);

static bool i80_tft_lcd_notify_flush_ready(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_io_event_data_t* event_data, void* user_data) {
    lv_display_flush_ready((lv_display_t*)(user_data));
    return false;
}

static void i80_tft_lcd_flush_cb(lv_display_t* display, const lv_area_t* area, uint8_t* color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)(lv_display_get_user_data(display));
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // because LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(color_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

void i80_tft_lcd_init() {
    esp_lcd_i80_bus_handle_t bus_handle;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;

    // Bus init.
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num         = I80_LCD_RS_GPIO,
        .wr_gpio_num         = I80_LCD_WR_GPIO,
        .clk_src             = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums      = I80_LCD_DATA_GPIO,
        .bus_width           = I80_LCD_BUS_WIDTH,
        .max_transfer_bytes  = 32004, // I80_LCD_H_RES * 100 * sizeof(uint16_t),
        .dma_burst_size      = I80_LCD_DMA_BURST_SIZE,
    };

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &bus_handle));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num         = I80_LCD_CS_GPIO,
        .pclk_hz             = I80_LCD_PIXEL_CLK_HZ,
        .trans_queue_depth   = 10,
        .lcd_cmd_bits        = I80_LCD_CMD_BITS,
        .lcd_param_bits      = I80_LCD_PARAM_BITS,
        .dc_levels           = {
            .dc_idle_level   = 0,
            .dc_cmd_level    = 0,
            .dc_dummy_level  = 0,
            .dc_data_level   = 1,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus_handle, &io_config, &io_handle));

    // Panel init.
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num  = I80_LCD_RST_GPIO,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_set_gap(panel_handle, 0, 20);

    lv_init();
    lv_display_t *display = lv_display_create(I80_LCD_H_RES, I80_LCD_V_RES);

    size_t draw_buffer_size = I80_LCD_H_RES * I80_LCD_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buffer1 = esp_lcd_i80_alloc_draw_buffer(io_handle, draw_buffer_size, 0);
    void *buffer2 = esp_lcd_i80_alloc_draw_buffer(io_handle, draw_buffer_size, 0);
    assert(buffer1);
    assert(buffer2);

    lv_display_set_buffers(display, buffer1, buffer2, draw_buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(display, panel_handle);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(display, i80_tft_lcd_flush_cb);

    lv_disp_set_default(display);
}

static esp_err_t app_touch_init(void)
{
    /* Initilize I2C */
    i2c_master_bus_handle_t i2c_handle = NULL;
    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = 0,
        .sda_io_num = 38,
        .scl_io_num = 39,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_config, &i2c_handle), TAG, "");


    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = I80_LCD_H_RES,
        .y_max = I80_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

	esp_lcd_panel_io_handle_t io_handle = NULL;
	
	esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_FT6x36_CONFIG();
    
	io_config.scl_speed_hz = I80_TOUCH_I2C_CLK_HZ;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &io_config, &io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_ft6x36(io_handle, &tp_cfg, &touch_handle); 

 return ESP_FAIL;
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 6144,         /* LVGL task stack size */
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    uint32_t buff_size = I80_LCD_H_RES * I80_LCD_DRAW_BUFF_HEIGHT;
#if I80_LCD_LVGL_FULL_REFRESH || I80_LCD_LVGL_DIRECT_MODE
    buff_size = I80_LCD_H_RES * I80_LCD_V_RES;
#endif

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .panel_handle = lcd_panel,
        .buffer_size = buff_size,
        .double_buffer = I80_LCD_DRAW_BUFF_DOUBLE,
        .hres = I80_LCD_H_RES,
        .vres = I80_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = true,
#if I80_LCD_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif I80_LCD_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
            .swap_bytes = false,
        }
    };
    const lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = {
#if I80_LCD_RGB_BOUNCE_BUFFER_MODE
            .bb_mode = true,
#else
            .bb_mode = false,
#endif
#if I80_LCD_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };
    lvgl_disp = lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}

void app_main(void)
{
    /* LCD HW initialization */
    i80_tft_lcd_init();

    /* Touch initialization */
    ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    lvgl_port_lock(0);

	ui_skoona_page(lv_scr_act());

    lvgl_port_unlock();

}