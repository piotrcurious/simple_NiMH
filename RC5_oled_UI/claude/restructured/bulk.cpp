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

// types.h
#pragma once
#include <CircularBuffer.h>

enum class ScreenID {
    MAIN_STATUS,
    CHANNEL_DETAIL,
    CHARGE_GRAPH,
    MAINTENANCE,
    SETTINGS
};

enum class ChargeState {
    IDLE,
    CHARGING,
    DONE,
    ERROR,
    MAINTENANCE
};

enum class IRCommand {
    UP = 0x01,
    DOWN = 0x02,
    LEFT = 0x03,
    RIGHT = 0x04,
    SELECT = 0x05,
    BACK = 0x06,
    NUM_1 = 0x11,
    NUM_2 = 0x12,
    NUM_3 = 0x13,
    NUM_4 = 0x14,
    PLUS = 0x1A,
    MINUS = 0x1B
};

// hardware.h
#pragma once

class HardwareManager {
public:
    virtual ~HardwareManager() = default;
    virtual void begin() = 0;
    virtual float readVoltage(uint8_t channel) = 0;
    virtual float readCurrent(uint8_t channel) = 0;
    virtual float readTemperature(uint8_t channel) = 0;
    virtual void setChargeCurrent(uint8_t channel, uint16_t currentMa) = 0;
};

class ESP32Hardware : public HardwareManager {
private:
    const uint8_t voltagePins[Config::NUM_CHANNELS] = {32, 33, 34, 35};
    const uint8_t currentPins[Config::NUM_CHANNELS] = {36, 37, 38, 39};
    const uint8_t tempPins[Config::NUM_CHANNELS + 1] = {25, 26, 27, 28, 29}; // +1 for ambient
    const uint8_t pwmPins[Config::NUM_CHANNELS] = {16, 17, 18, 19};
    
public:
    void begin() override {
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            pinMode(pwmPins[i], OUTPUT);
            analogWrite(pwmPins[i], 0);
        }
    }
    
    float readVoltage(uint8_t channel) override {
        float raw = analogRead(voltagePins[channel]) * 3.3f / 4095.0f;
        return raw * 1000.0f; // Convert to mV
    }
    
    // ... similar implementations for other hardware methods
};

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

// channel_manager.h
#pragma once

class ChannelManager {
private:
    HardwareManager& hardware;
    ChannelData channels[Config::NUM_CHANNELS];
    float ambientTemp = 0;
    
    void updateChannel(uint8_t channel) {
        auto& ch = channels[channel];
        if (ch.state == ChargeState::IDLE) return;
        
        // Update measurements
        ch.voltageMv = hardware.readVoltage(channel);
        ch.currentMa = hardware.readCurrent(channel);
        ch.batteryTemp = hardware.readTemperature(channel);
        ch.deltaTemp = ch.batteryTemp - ambientTemp;
        
        // Update histories
        ch.voltageHistory.add(ch.voltageMv);
        ch.currentHistory.add(ch.currentMa);
        
        // Update capacity
        uint32_t deltaTime = millis() - ch.lastUpdateTime;
        ch.capacityMah += (ch.currentMa * deltaTime) / 3600000.0f;
        
        // Peak voltage detection
        if (ch.voltageMv > ch.peakVoltage) {
            ch.peakVoltage = ch.voltageMv;
        } else if (ch.peakVoltage - ch.voltageMv > Config::DELTA_V_THRESHOLD_MV) {
            ch.negDeltaV = ch.peakVoltage - ch.voltageMv;
        }
        
        ch.lastUpdateTime = millis();
    }
    
    void checkTermination(uint8_t channel) {
        auto& ch = channels[channel];
        if (ch.state != ChargeState::CHARGING) return;
        
        bool terminate = false;
        
        // Check termination conditions
        if (ch.negDeltaV >= Config::DELTA_V_THRESHOLD_MV ||
            (ch.tempCheckEnabled && ch.deltaTemp > Config::DELTA_TEMP_THRESHOLD_C) ||
            (millis() - ch.startTime > Config::MAX_CHARGE_TIME_MS)) {
            terminate = true;
        }
        
        if (terminate) {
            stopCharging(channel);
            ch.state = ChargeState::DONE;
        }
    }
    
public:
    ChannelManager(HardwareManager& hw) : hardware(hw) {}
    
    void update() {
        ambientTemp = hardware.readTemperature(Config::NUM_CHANNELS); // Last sensor is ambient
        
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            updateChannel(i);
            checkTermination(i);
            hardware.setChargeCurrent(i, channels[i].targetCurrentMa);
        }
    }
    
    void startCharging(uint8_t channel, uint16_t current) {
        if (channel >= Config::NUM_CHANNELS) return;
        
        auto& ch = channels[channel];
        ch.state = ChargeState::CHARGING;
        ch.startTime = ch.lastUpdateTime = millis();
        ch.capacityMah = 0;
        ch.peakVoltage = 0;
        ch.negDeltaV = 0;
        ch.targetCurrentMa = current;
    }
    
    void stopCharging(uint8_t channel) {
        if (channel >= Config::NUM_CHANNELS) return;
        
        auto& ch = channels[channel];
        ch.state = ChargeState::IDLE;
        ch.targetCurrentMa = 0;
    }
    
    ChannelData& getChannel(uint8_t channel) {
        return channels[channel];
    }
};

// ui_base.h
#pragma once

class Screen {
protected:
    Adafruit_SSD1306& display;
    ChannelManager& channels;
    
    // Common drawing utilities
    void drawBattery(uint8_t x, uint8_t y, uint8_t level, bool charging) {
        // Implementation...
    }
    
    void drawGraph(const RollingGraph& graph, uint8_t x, uint8_t y, 
                  uint8_t width, uint8_t height) {
        // Implementation...
    }
    
public:
    Screen(Adafruit_SSD1306& disp, ChannelManager& ch) 
        : display(disp), channels(ch) {}
    virtual ~Screen() = default;
    
    virtual void draw() = 0;
    virtual void handleInput(IRCommand cmd) = 0;
    virtual void update() {} // Optional update method
};

// screens/main_status_screen.h
#pragma once

class MainStatusScreen : public Screen {
    // Implementation moved from previous version...
};

// screens/maintenance_screen.h
#pragma once

class MaintenanceScreen : public Screen {
    // Implementation moved from previous version...
};

// ui_manager.h
#pragma once

class UIManager {
private:
    Adafruit_SSD1306 display;
    IRrecv& irReceiver;
    ChannelManager& channelManager;
    std::unique_ptr<Screen> screens[5];
    Screen* currentScreen;
    uint32_t lastUpdate;
    
public:
    UIManager(IRrecv& ir, ChannelManager& cm) 
        : irReceiver(ir), channelManager(cm), lastUpdate(0) {
        display = Adafruit_SSD1306(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, 
                                 &Wire, Config::OLED_RESET);
    }
    
    void begin() {
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        display.clearDisplay();
        display.setTextColor(WHITE);
        
        // Initialize screens
        screens[static_cast<int>(ScreenID::MAIN_STATUS)] = 
            std::make_unique<MainStatusScreen>(display, channelManager);
        // ... initialize other screens
        
        currentScreen = screens[0].get();
    }
    
    void update() {
        uint32_t now = millis();
        
        // Handle IR input
        if (irReceiver.decode()) {
            handleInput(static_cast<IRCommand>(irReceiver.decodedIRData.command));
            irReceiver.resume();
        }
        
        // Update current screen
        currentScreen->update();
        currentScreen->draw();
        
        lastUpdate = now;
    }
    
    void handleInput(IRCommand cmd) {
        // Implementation...
    }
};

// main.cpp
#include "config.h"
#include "hardware.h"
#include "channel_manager.h"
#include "ui_manager.h"

ESP32Hardware hardware;
ChannelManager channelManager(hardware);
IRrecv irReceiver(Config::IR_RECEIVE_PIN);
UIManager uiManager(irReceiver, channelManager);

void setup() {
    Wire.begin();
    hardware.begin();
    irReceiver.enableIRIn();
    uiManager.begin();
}

void loop() {
    static uint32_t lastSensorUpdate = 0;
    uint32_t now = millis();
    
    // Update sensors at regular interval
    if (now - lastSensorUpdate >= Config::SENSOR_UPDATE_MS) {
        channelManager.update();
        lastSensorUpdate = now;
    }
    
    // Update UI
    uiManager.update();
    delay(Config::SCREEN_UPDATE_MS);
}
