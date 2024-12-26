// data_types.h
#pragma once

struct RollingGraph {
    CircularBuffer<float, Config::GRAPH_HISTORY_SIZE> values;
    float minValue = 0;
    float maxValue = 0;
    float scale = 1;
    
    void add(float value) {
        values.push(value);
        updateBounds();
    }
    
    void updateBounds() {
        if (values.isEmpty()) return;
        
        minValue = maxValue = values[0];
        for (const auto& value : values) {
            minValue = std::min(minValue, value);
            maxValue = std::max(maxValue, value);
        }
        scale = (maxValue - minValue) > 0 ? 40.0f / (maxValue - minValue) : 1.0f;
    }
};

struct ChannelData {
    float currentMa = 0;
    float voltageMv = 0;
    float batteryTemp = 0;
    float deltaTemp = 0;
    ChargeState state = ChargeState::IDLE;
    uint32_t startTime = 0;
    uint32_t lastUpdateTime = 0;
    uint16_t capacityMah = 0;
    RollingGraph voltageHistory;
    RollingGraph currentHistory;
    uint16_t peakVoltage = 0;
    uint8_t negDeltaV = 0;
    bool tempCheckEnabled = true;
    uint16_t targetCurrentMa = 0;
};
