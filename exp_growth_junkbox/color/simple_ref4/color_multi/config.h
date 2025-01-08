#pragma once

// Screen definitions
#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
#define OLED_SCREEN_ADDRESS 0x3C

constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;

#define TFT_SCREEN_WIDTH 320
#define TFT_SCREEN_HEIGHT 240

// Analysis parameters
#define MAX_DATASET_WINDOW 100 
#define GROWTH_EST_BACKWARD_TIME_WINDOW 5.0 // seconds
#define GROWTH_EST_FORWARD_TIME_WINDOW  2.0 // seconds
