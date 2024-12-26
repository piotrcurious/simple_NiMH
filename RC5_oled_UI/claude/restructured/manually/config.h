// config.h
#pragma once

namespace Config {
    // Hardware configuration
    constexpr uint8_t SCREEN_WIDTH = 128;
    constexpr uint8_t SCREEN_HEIGHT = 64;
    constexpr int8_t OLED_RESET = -1;
    constexpr uint8_t IR_RECEIVE_PIN = 15;
    constexpr uint8_t NUM_CHANNELS = 4;
    
    // Charge parameters
    constexpr float DELTA_V_THRESHOLD_MV = 5.0f;
    constexpr float DELTA_TEMP_THRESHOLD_C = 2.0f;
    constexpr uint32_t MAX_CHARGE_TIME_MS = 14400000; // 4 hours
    constexpr uint16_t MAX_CHARGE_CURRENT_MA = 2000;
    constexpr uint16_t MIN_CHARGE_CURRENT_MA = 50;
    
    // UI parameters
    constexpr uint16_t GRAPH_HISTORY_SIZE = 100;
    constexpr uint16_t SCREEN_UPDATE_MS = 50;
    constexpr uint16_t SENSOR_UPDATE_MS = 1000;
}
