#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "regex.h"

#include "lcd.h"
#include "gui.h"
#include "pic.h"
#include "xpt2046.h"

#define CHANNEL_COUNT 13

// usage: (channel * CHANNEL2FREQ)
#define CHANNEL2FREQ 5 + 2407

// usage: (channel * CHANNEL2X)
#define CHANNEL2X (SCREEN_W / (CHANNEL_COUNT + 1)) + 23

// usage: (rssi * RSSI2Y)
#define RSSI2Y -2

// touchscreen stuff
#define CMD_X_READ 0b10010000 //0x90
#define CMD_Y_READ 0b11010000 //0xD0
#define SCREEN_W 320
#define SCREEN_H 240
#define BASELINE 200

#define PAUSE_X1 5
#define PAUSE_Y1 200
#define PAUSE_X2 30
#define PAUSE_Y2 225

// speed 3,     speed 2,     speed 1
// 150ms = ~2s, 230ms = ~3s, 300ms = ~4s
#define SPEED2INTERVAL -77 + 384

#define MIN_SPEED 1
#define MAX_SPEED 3

#define SPEED_X1 250
#define SPEED_Y1 3
#define SPEED_X2 296
#define SPEED_Y2 15

#define MAX_APS 50

typedef struct {
    char ssid[32];
    int8_t rssi;
    uint8_t rate;
    uint8_t sig_mode;
    uint8_t channel;
	uint8_t bandwidth;
    uint32_t timestamp;
} ap_info_t;

QueueHandle_t ap_queue;

const static char *TAG = "wifi_mapper";

volatile bool playing = true;
volatile uint8_t speed_setting = MIN_SPEED;


void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}


void draw_pause() {
    uint16_t x_unit = (PAUSE_X2 - PAUSE_X1) / 5;
    uint16_t y_unit = (PAUSE_Y2 - PAUSE_Y1) / 5;
    
    LCD_DrawFillRectangle(PAUSE_X1, PAUSE_Y1, PAUSE_X2, PAUSE_Y2, GRAY); // button bg

    LCD_DrawFillRectangle(PAUSE_X1 + x_unit, PAUSE_Y1 + y_unit, PAUSE_X1 + 2 * x_unit, PAUSE_Y2 - y_unit, WHITE); // left bar
    LCD_DrawFillRectangle(PAUSE_X2 - 2 * x_unit, PAUSE_Y1 + y_unit, PAUSE_X2 - x_unit, PAUSE_Y2 - y_unit, WHITE); // right bar
}

void draw_play() {
    uint16_t x_unit = (PAUSE_X2 - PAUSE_X1) / 5;
    uint16_t y_unit = (PAUSE_Y2 - PAUSE_Y1) / 5;

    LCD_DrawFillRectangle(PAUSE_X1, PAUSE_Y1, PAUSE_X2, PAUSE_Y2, GRAY); // button bg

    LCD_DrawFillTriangel( // play icon
        PAUSE_X1 + x_unit, PAUSE_Y1 + y_unit,
        PAUSE_X1 + x_unit, PAUSE_Y2 - y_unit,
        PAUSE_X2 - x_unit, (PAUSE_Y2 + PAUSE_Y1) / 2,
        WHITE
    );
}


void draw_channel_selection(uint8_t channel, uint16_t color) {
    uint16_t x_coord = channel * CHANNEL2X;
    LCD_DrawLine(x_coord, -10 * RSSI2Y, x_coord, BASELINE - 1, color);
    if (color == GREEN) {
        LCD_DrawLine(x_coord - 1, -10 * RSSI2Y, x_coord - 1, BASELINE - 1, BLACK);
        LCD_DrawLine(x_coord + 1, -10 * RSSI2Y, x_coord + 1, BASELINE - 1, BLACK);
    }
}

bool isBetween(uint16_t value, uint16_t min, uint16_t max) {
    if (value > max) {
        return false;
    }
    if (value < min) {
        return false;
    }
    return true;
}


static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!playing || type != WIFI_PKT_MGMT) {
		return; // filter
	}

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*) buf;
    ap_info_t info;

    // RSSI & channel
    info.rssi = pkt->rx_ctrl.rssi;
    info.rate = pkt->rx_ctrl.rate;
    info.sig_mode = pkt->rx_ctrl.sig_mode;
    info.channel = pkt->rx_ctrl.channel;
    info.timestamp = pkt->rx_ctrl.timestamp;
	
	// Determine bandwidth
	if (pkt->rx_ctrl.sig_mode == 1) { // 802.11n
		info.bandwidth = (pkt->rx_ctrl.cwb ? 40 : 20);
	} else {
		info.bandwidth = 20;
	}

    xQueueSendFromISR(ap_queue, &info, NULL);
}


void sniffer_task(void *pvParameters) {
    uint8_t channel = 1;

    ap_info_t sync;
    sync.rssi = 0;
    sync.bandwidth = 0;

    while (1) {
        if (playing) {
            channel++;
        }
        if (channel > CHANNEL_COUNT) {
            channel = 1;
		};
        sync.channel = channel;
        
        xQueueSend(ap_queue, &sync, NULL);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(speed_setting * SPEED2INTERVAL));
    }
}


void render_task(void *pvParameters) {
    ap_info_t info;

    LCD_DrawFillRectangle(0, 0, SCREEN_W, SCREEN_H, BLACK); // Clear screen

    // vertical grid lines (channels)
    uint16_t line_x = 0;
    char text[32];
    for (uint16_t i = 1; i <= CHANNEL_COUNT; i++) {
        line_x = i * CHANNEL2X;
        LCD_DrawLine(line_x, -10 * RSSI2Y, line_x, BASELINE, GRAY);
        LCD_DrawLine(line_x, BASELINE, line_x, BASELINE + 4, WHITE);

        sprintf(text, "Ch.%d", i);

	    LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_PORTRAIT_INVERTED);
        LCD_ShowString(BASELINE + 7, SCREEN_W - line_x - 7, BLACK, WHITE, 12, text, 0);
	    LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_LANDSCAPE);
    }

    // horizontal grid lines (rssi)
    uint16_t line_y = 0;
    for (uint16_t nrssi = 10; nrssi <= 90; nrssi += 10) {
        line_y = -nrssi * RSSI2Y;
        LCD_DrawLine(CHANNEL2X, line_y, SCREEN_W, line_y, GRAY);

        sprintf(text, "-%ddBm", nrssi);
        LCD_ShowString(5, line_y - 6, BLACK, WHITE, 12, text, 1);
    }

    LCD_DrawLine(CHANNEL2X, BASELINE, SCREEN_W, BASELINE, WHITE);

    uint16_t delayInTicks = pdMS_TO_TICKS(speed_setting * SPEED2INTERVAL - 1);
    bool wasPlaying = false;
    uint8_t prevSpeed = 0;

    while (1) {
        // check if speed_setting was changed
        if (speed_setting != prevSpeed) {
            sprintf(text, "SPEED: %d", speed_setting);
            LCD_DrawFillRectangle(SPEED_X1, SPEED_Y1, SPEED_X2, SPEED_Y2, GRAY);
            LCD_ShowString(SPEED_X1, SPEED_Y1, GRAY, WHITE, 12, text, 1);
            delayInTicks = pdMS_TO_TICKS(speed_setting * SPEED2INTERVAL - 1);
            prevSpeed = speed_setting;
            vTaskDelay(delayInTicks);
        }
        // check if pause/play was pressed
        if (playing ^ wasPlaying) {
            if (playing) {
                draw_pause();
            } else {
                draw_play();
            }
            wasPlaying = playing;
            vTaskDelay(delayInTicks);
        }
        if (playing && xQueueReceive(ap_queue, &info, delayInTicks)) {
            if (info.rssi == 0 && info.bandwidth == 0) { // sync frame
                draw_channel_selection(info.channel, GRAY);
                if (info.channel == CHANNEL_COUNT) {
                    draw_channel_selection(1, GREEN);
                } else {
                    draw_channel_selection(info.channel + 1, GREEN);
                }
            } else {
                ESP_LOGI(TAG, "Freq: %d\tBandwidth: %d\tRSSI: %ddB", info.channel * CHANNEL2FREQ, info.bandwidth, info.rssi);

                LCD_Draw_FillCircle(info.channel * CHANNEL2X, info.rssi * RSSI2Y, 1, GREEN);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void app_main(void) {
	ESP_LOGI(TAG, "Starting init......");

	ESP_LOGI(TAG, "Initializing event loop...");
    esp_event_loop_create_default();

	ESP_LOGI(TAG, "Initializing nvs...");
	init_nvs();

	ESP_LOGI(TAG, "Initializing wifi config...");
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    ESP_LOGI(TAG, "Setting promiscuous mode...");
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_cb);

    ESP_LOGI(TAG, "Creating AP queue...");
    ap_queue = xQueueCreate(MAX_APS, sizeof(ap_info_t));

	ESP_LOGI(TAG, "Clearing screen...");
	Init_LCD(WHITE);

	ESP_LOGI(TAG, "Display init...");
	xpt2046_init();

	LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_LANDSCAPE);

	ESP_LOGI(TAG, "TP adjust...");
	TP_Adjust();

    ESP_LOGI(TAG, "Starting sniffer task...");
    xTaskCreate(sniffer_task, "sniffer_task", 4096, NULL, 5, NULL);

	ESP_LOGI(TAG, "Starting render task...");
    xTaskCreate(render_task, "render_task", 4096, NULL, 2, NULL);

    bool touch_read = false;
    bool posedge = false;

    while (1) { // touch event loop
        touch_read = xpt2046_read();
		if (touch_read && !posedge) {
            // pause/play button
            if (isBetween(TouchX, PAUSE_X1, PAUSE_X2) && isBetween(TouchY, PAUSE_Y1, PAUSE_Y2)) {
                playing = !playing;
                ESP_LOGI(TAG, "Playing: %d", playing);
            }
            // speed
            if (TouchX > SPEED_X1 && TouchY < SPEED_Y2) {
                speed_setting++;
                if (speed_setting > MAX_SPEED) {
                    speed_setting = MIN_SPEED;
                }
                ESP_LOGI(TAG, "Speed: %d", speed_setting);
            }
		}
        posedge = touch_read;
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}
