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
#define SPEED_Y2 14

#define MAX_APS 50
#define TAP_RADIUS 10

#define TEXT_SPACING 24

typedef struct {
    char ssid[32];
    int8_t rssi;
    uint8_t rate;
    uint8_t sig_mode;
    uint8_t channel;
    uint8_t bandwidth;
    uint32_t timestamp;
    bool visible;
    uint16_t x;
    uint16_t y;
} ap_info_t;

QueueHandle_t ap_queue;

typedef struct {
    uint16_t x;
    uint16_t y;
} touch_event_t;

QueueHandle_t touch_queue;

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


void draw_x_values(bool useFrequencies) {
    char text[32];

    LCD_DrawFillRectangle(PAUSE_X2 + 5, BASELINE + 5, SCREEN_W, SCREEN_H, BLACK);

    LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_PORTRAIT_INVERTED); // to print text sideways
    for (uint16_t i = 1; i <= CHANNEL_COUNT; i++) {
        if (useFrequencies) {
            sprintf(text, "%d", i * CHANNEL2FREQ);
        } else {
            sprintf(text, "Ch.%d", i);
        }

        LCD_ShowString(BASELINE + 7, SCREEN_W - (i * CHANNEL2X) - 7, BLACK, WHITE, 12, text, 0);
    }
    LCD_Set_Orientation(LCD_DISPLAY_ORIENTATION_LANDSCAPE);
}


void draw_info(ap_info_t info) {
    LCD_DrawFillRectangle(PAUSE_X2 + 5, BASELINE + 7, SCREEN_W - 5, SCREEN_H - 5, GRAY);
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


// not actual distance
uint16_t crude_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint16_t min_x = (x1 < x2) ? x1 : x2;
    uint16_t max_x = (x1 > x2) ? x1 : x2;
    uint16_t d_x = max_x - min_x;
    uint16_t min_y = (y1 < y2) ? y1 : y2;
    uint16_t max_y = (y1 > y2) ? y1 : y2;
    uint16_t d_y = max_y - min_y;
    return (d_x > d_y) ? d_x : d_y;
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
    sync.visible = false;

    while (1) {
        if (playing) {
            channel++;
        }
        if (channel > CHANNEL_COUNT) {
            channel = 1;
		};
        sync.channel = channel;
        
        if (playing) {
            xQueueSend(ap_queue, &sync, NULL);
            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        }

        vTaskDelay(pdMS_TO_TICKS(speed_setting * SPEED2INTERVAL));
    }
}


void render_task(void *pvParameters) {
    ap_info_t info;
    touch_event_t event;
    ap_info_t packetCache[MAX_APS];
    uint8_t cacheIndex = 0;
    int8_t selectedIndex = -1;

    bool usingFreqLabel = false;

    LCD_DrawFillRectangle(0, 0, SCREEN_W, SCREEN_H, BLACK); // Clear screen

    // vertical grid lines (channels)
    uint16_t line_x = 0;
    char text[32];
    for (uint16_t i = 1; i <= CHANNEL_COUNT; i++) {
        line_x = i * CHANNEL2X;
        LCD_DrawLine(line_x, -10 * RSSI2Y, line_x, BASELINE, GRAY);
        LCD_DrawLine(line_x, BASELINE, line_x, BASELINE + 4, WHITE);
    }

    draw_x_values(usingFreqLabel);

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

    // draw initial values for buttons
    draw_pause();
    sprintf(text, "SPEED: %d", MIN_SPEED);
    LCD_DrawFillRectangle(SPEED_X1 - 2, SPEED_Y1, SPEED_X2 + 2, SPEED_Y2, GRAY);
    LCD_ShowString(SPEED_X1, SPEED_Y1, GRAY, WHITE, 12, text, 1);

    while (1) {
        if (xQueueReceive(touch_queue, &event, 0)) {
            bool eventHandled = false;

            // clear circle around selected packet, if any
            if (selectedIndex > 0) {
                LCD_Draw_Circle(packetCache[selectedIndex].x, packetCache[selectedIndex].y, 3, BLACK);
                draw_x_values(usingFreqLabel);
                selectedIndex = -1;
            }

            // pause/play button
            if (!eventHandled && isBetween(event.x, PAUSE_X1, PAUSE_X2) && isBetween(event.y, PAUSE_Y1, PAUSE_Y2)) {
                playing = !playing;

                if (playing) {
                    draw_pause();
                } else {
                    draw_play();
                }
                
                ESP_LOGI(TAG, "Playing: %d", playing);
                eventHandled = true;
            }
            
            // speed
            if (!eventHandled && event.x > SPEED_X1 && event.y < SPEED_Y2) {
                speed_setting++;
                if (speed_setting > MAX_SPEED) {
                    speed_setting = MIN_SPEED;
                }

                sprintf(text, "SPEED: %d", speed_setting);
                LCD_DrawFillRectangle(SPEED_X1, SPEED_Y1, SPEED_X2, SPEED_Y2, GRAY);
                LCD_ShowString(SPEED_X1, SPEED_Y1, GRAY, WHITE, 12, text, 1);
                delayInTicks = pdMS_TO_TICKS(speed_setting * SPEED2INTERVAL - 1);
                
                ESP_LOGI(TAG, "Speed: %d", speed_setting);
                eventHandled = true;
            }

            // check if x-axis was tapped on
            if (!eventHandled && event.y > BASELINE) {
                usingFreqLabel = !usingFreqLabel;
                draw_x_values(usingFreqLabel);
                eventHandled = true;
            }

            // check if tap was on a graphed packet
            if (!eventHandled) {
                for (uint16_t i = 0; i < MAX_APS; i++) {
                    if (packetCache[i].visible && crude_distance(packetCache[i].x, packetCache[i].y, event.x, event.y) < TAP_RADIUS) {
                        // pause scanning
                        playing = false;
                        draw_play();

                        selectedIndex = i;
                        draw_info(packetCache[i]);

                        // circle the packet
                        LCD_Draw_Circle(packetCache[i].x, packetCache[i].y, 3, WHITE);

                        break;
                    }
                }
            }
        }
        if (playing && xQueueReceive(ap_queue, &info, delayInTicks)) {
            if (info.rssi == 0 && info.bandwidth == 0) { // sync frame
                draw_channel_selection(info.channel, GRAY);
                if (info.channel == CHANNEL_COUNT) {
                    draw_channel_selection(1, GREEN);
                } else {
                    draw_channel_selection(info.channel + 1, GREEN);
                }

                // loop through all cached packets and mark old ones as "invisible"
                for (uint16_t i = 0; i < MAX_APS; i++) {
                    if (packetCache[i].channel == info.channel) {
                        packetCache[i].visible = false;
                    }
                }

            } else {
                ESP_LOGI(TAG, "Freq: %d\tBandwidth: %d\tRSSI: %ddBm", info.channel * CHANNEL2FREQ, info.bandwidth, info.rssi);
                
                ap_info_t copy = info;
                copy.x = info.channel * CHANNEL2X;
                copy.y = info.rssi * RSSI2Y;
                copy.visible = true;

                packetCache[cacheIndex] = copy;
                cacheIndex++;
                if (cacheIndex >= MAX_APS) {
                    cacheIndex = 0;
                }

                LCD_Draw_FillCircle(copy.x, copy.y, 1, GREEN);
            }
        } else if (!playing) {
            vTaskDelay(delayInTicks);
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

    ESP_LOGI(TAG, "Creating touch event queue...");
    touch_queue = xQueueCreate(10, sizeof(touch_event_t));

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
    xTaskCreate(render_task, "render_task", 8192, NULL, 2, NULL);

    bool touch_read = false;
    bool posedge = false;

    touch_event_t event = {
        .x = TouchX,
        .y = TouchY
    };
    while (1) { // touch event loop
        touch_read = xpt2046_read();
		if (touch_read && !posedge) {
            event.x = TouchX;
            event.y = TouchY;
            xQueueSend(touch_queue, &event, 0);
		}
        posedge = touch_read;
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}
