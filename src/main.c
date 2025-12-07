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
#define CHANNEL2X (SCREEN_W / (CHANNEL_COUNT + 1)) + 20

// usage: (rssi * RSSI2Y)
#define RSSI2Y -2 - 10

// 150ms = ~2s, 230ms = ~3s, 300ms = ~4s
#define CHANNEL_HOP_INTERVAL_MS 150

#define SCREEN_W 320
#define SCREEN_H 240
#define BASELINE 180

#define MAX_APS 50

typedef struct {
    char ssid[32];
    int8_t rssi;
    uint8_t channel;
	uint8_t bandwidth;
} ap_info_t;

QueueHandle_t ap_queue;

const static char *TAG = "wifi_mapper";


void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}


static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) {
		return; // filter
	}

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*) buf;
    ap_info_t info;

    // RSSI & channel
    info.rssi = pkt->rx_ctrl.rssi;
    info.channel = pkt->rx_ctrl.channel;
	
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
		channel++;
        if (channel > CHANNEL_COUNT) {
            channel = 1;
		};
        sync.channel = channel;
        
        xQueueSend(ap_queue, &sync, NULL);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(CHANNEL_HOP_INTERVAL_MS));
    }
}


void draw_channel_selection(uint8_t channel, uint16_t color) {
    uint16_t x_coord = channel * CHANNEL2X;
    LCD_DrawLine(x_coord, 0, x_coord, BASELINE - 1, color);
    if (color == GREEN) {
        LCD_DrawLine(x_coord - 1, 0, x_coord - 1, BASELINE - 1, BLACK);
        LCD_DrawLine(x_coord + 1, 0, x_coord + 1, BASELINE - 1, BLACK);
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
        LCD_DrawLine(line_x, 0, line_x, BASELINE, GRAY);
        LCD_DrawLine(line_x, BASELINE, line_x, BASELINE + 4, WHITE);

        sprintf(text, "%d", i * CHANNEL2FREQ);

	    LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_PORTRAIT_INVERTED);
        LCD_ShowString(BASELINE + 7, SCREEN_W - line_x - 6, BLACK, WHITE, 12, text, 0);
	    LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_LANDSCAPE);
    }

    // horizontal grid lines (rssi)
    uint16_t line_y = 0;
    for (uint16_t nrssi = 10; nrssi <= 90; nrssi += 10) {
        line_y = -nrssi * RSSI2Y;
        LCD_DrawLine(40, line_y, SCREEN_W, line_y, GRAY);;

        sprintf(text, "-%ddBm", nrssi);
        LCD_ShowString(5, line_y - 6, BLACK, WHITE, 12, text, 1);
    }

    LCD_DrawLine(40, BASELINE, SCREEN_W, BASELINE, WHITE);

    uint16_t delayInTicks = pdMS_TO_TICKS(CHANNEL_HOP_INTERVAL_MS - 1);

    while (1) {
        if (xQueueReceive(ap_queue, &info, delayInTicks)) {
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
}
