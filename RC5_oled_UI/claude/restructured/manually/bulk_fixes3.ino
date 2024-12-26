// First, add missing includes to the main header
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <esp_task_wdt.h>
#include <CircularBuffer.h>
#include <algorithm>
#include <memory>

// Add version control and EEPROM structure versioning
namespace Config {
    constexpr uint16_t FIRMWARE_VERSION = 0x0100;  // v1.0
    constexpr uint16_t EEPROM_VERSION = 0x0100;    // v1.0
    constexpr uint16_t EEPROM_SETTINGS_ADDR = 0;
    constexpr uint16_t EEPROM_CALIBRATION_ADDR = 256;
    constexpr uint16_t EEPROM_VERSION_ADDR = 510;  // Last 2 bytes
}

// Add proper initialization checks
bool initSystem() {
    // Initialize I2C
    if (!Wire.begin()) {
        return false;
    }
    
    // Initialize EEPROM and check version
    EEPROM.begin(512);
    uint16_t storedVersion;
    EEPROM.get(Config::EEPROM_VERSION_ADDR, storedVersion);
    
    if (storedVersion != Config::EEPROM_VERSION) {
        // First time or version mismatch - initialize EEPROM
        EEPROM.put(Config::EEPROM_VERSION_ADDR, Config::EEPROM_VERSION);
        
        // Initialize settings with defaults
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            ChannelSettings defaultSettings;
            EEPROM.put(Config::EEPROM_SETTINGS_ADDR + i * sizeof(ChannelSettings), 
                      defaultSettings);
        }
        
        // Initialize calibration with defaults
        CalibrationData defaultCalibration;
        EEPROM.put(Config::EEPROM_CALIBRATION_ADDR, defaultCalibration);
        
        EEPROM.commit();
    }
    EEPROM.end();
    
    return true;
}

// Add proper error handling to ESP32Hardware
class ESP32Hardware : public HardwareManager {
private:
    CalibrationData calibration;
    bool initialized = false;
    
    bool initADC() {
        // Configure ADC for voltage readings
        analogReadResolution(12);
        analogSetAttenuation(ADC_11db);
        analogSetPinAttenuation(36, ADC_11db);
        
        // Test ADC functionality
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            if (analogRead(voltagePins[i]) == 0 || analogRead(voltagePins[i]) == 4095) {
                return false;
            }
        }
        return true;
    }
    
public:
    bool begin() override {
        if (initialized) return true;
        
        // Load calibration data
        EEPROM.begin(512);
        EEPROM.get(Config::EEPROM_CALIBRATION_ADDR, calibration);
        EEPROM.end();
        
        // Initialize pins
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            pinMode(pwmPins[i], OUTPUT);
            analogWrite(pwmPins[i], 0);
        }
        
        // Initialize ADC
        if (!initADC()) {
            return false;
        }
        
        initialized = true;
        return true;
    }
    
    // Add proper error checking to sensor readings
    float readVoltage(uint8_t channel) override {
        if (!initialized || channel >= Config::NUM_CHANNELS) return 0.0f;
        
        // Take multiple readings for stability
        float sum = 0.0f;
        const int samples = 5;
        
        for (int i = 0; i < samples; i++) {
            float raw = analogRead(voltagePins[channel]) * 3.3f / 4095.0f;
            sum += (raw + calibration.voltageOffset[channel]) * 
                   calibration.voltageScale[channel];
            delay(1);
        }
        
        return (sum / samples) * 1000.0f; // Convert to mV
    }
};

// Add proper initialization checking to ChannelManager
class ChannelManager {
private:
    bool initialized = false;
    uint32_t lastUpdateTime = 0;
    static constexpr uint32_t MIN_UPDATE_INTERVAL = 100; // Minimum 100ms between updates
    
public:
    bool begin() {
        if (initialized) return true;
        
        // Initialize hardware
        if (!hardware.begin()) {
            return false;
        }
        
        // Initialize channels
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            channels[i] = ChannelData();
        }
        
        initialized = true;
        return true;
    }
    
    void update() override {
        if (!initialized) return;
        
        uint32_t now = millis();
        if (now - lastUpdateTime < MIN_UPDATE_INTERVAL) return;
        
        // Update ambient temperature first
        ambientTemp = hardware.readTemperature(Config::NUM_CHANNELS);
        
        // Check ambient temperature limits
        if (ambientTemp < Config::MIN_AMBIENT_TEMP_C || 
            ambientTemp > Config::MAX_AMBIENT_TEMP_C) {
            // Stop all charging if ambient temperature is out of range
            for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
                if (channels[i].state == ChargeState::CHARGING) {
                    stopCharging(i);
                }
            }
            return;
        }
        
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            updateChannel(i);
            checkSafety(i);
            handleMaintenance(i);
            updateHardware(i);
        }
        
        lastUpdateTime = now;
    }
};

// Add proper screen buffer management to UIManager
class UIManager {
private:
    bool initialized = false;
    static constexpr size_t SCREEN_BUFFER_SIZE = 1024;
    std::unique_ptr<uint8_t[]> screenBuffer;
    
public:
    UIManager(IRrecv& ir, ChannelManager& cm) 
        : irReceiver(ir), 
          channelManager(cm),
          lastUpdate(0),
          screenBuffer(new uint8_t[SCREEN_BUFFER_SIZE]) {
        
        display = Adafruit_SSD1306(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, 
                                 &Wire, Config::OLED_RESET);
    }
    
    bool begin() {
        if (initialized) return true;
        
        // Initialize display
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            return false;
        }
        
        display.setBuffer(screenBuffer.get(), SCREEN_BUFFER_SIZE);
        display.clearDisplay();
        display.setTextColor(WHITE);
        
        // Initialize screens
        try {
            screens[static_cast<int>(ScreenID::MAIN_STATUS)] = 
                std::make_unique<MainStatusScreen>(display, channelManager);
            screens[static_cast<int>(ScreenID::CHANNEL_DETAIL)] = 
                std::make_unique<ChannelDetailScreen>(display, channelManager);
            screens[static_cast<int>(ScreenID::CHARGE_GRAPH)] = 
                std::make_unique<ChargeGraphScreen>(display, channelManager);
            screens[static_cast<int>(ScreenID::MAINTENANCE)] = 
                std::make_unique<MaintenanceScreen>(display, channelManager);
            screens[static_cast<int>(ScreenID::SETTINGS)] = 
                std::make_unique<SettingsScreen>(display, channelManager);
        } catch (const std::bad_alloc& e) {
            return false;
        }
        
        currentScreen = screens[0].get();
        initialized = true;
        return true;
    }
    
    void update() {
        if (!initialized) return;
        
        uint32_t now = millis();
        
        // Handle IR input
        if (irReceiver.decode()) {
            handleInput(static_cast<IRCommand>(irReceiver.decodedIRData.command));
            irReceiver.resume();
        }
        
        // Update and draw current screen
        if (currentScreen) {
            currentScreen->update();
            currentScreen->draw();
            display.display();
        }
        
        lastUpdate = now;
    }
};

// Update main loop with proper initialization and error handling
void setup() {
    Serial.begin(115200);
    
    // Initialize watchdog
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);
    
    // Initialize system
    if (!initSystem()) {
        Serial.println("System initialization failed!");
        while (1) {
            esp_task_wdt_reset();
            delay(1000);
        }
    }
    
    // Initialize hardware manager
    if (!hardware.begin()) {
        Serial.println("Hardware initialization failed!");
        while (1) {
            esp_task_wdt_reset();
            delay(1000);
        }
    }
    
    // Initialize channel manager
    if (!channelManager.begin()) {
        Serial.println("Channel manager initialization failed!");
        while (1) {
            esp_task_wdt_reset();
            delay(1000);
        }
    }
    
    // Initialize IR receiver
    irReceiver.enableIRIn();
    
    // Initialize UI manager
    if (!uiManager.begin()) {
        Serial.println("UI initialization failed!");
        while (1) {
            esp_task_wdt_reset();
            delay(1000);
        }
    }
    
    Serial.println("System initialized successfully!");
}

void loop() {
    static uint32_t lastSensorUpdate = 0;
    static uint32_t lastWatchdogReset = 0;
    uint32_t now = millis();
    
    // Reset watchdog timer
    if (now - lastWatchdogReset >= 1000) {
        esp_task_wdt_reset();
        lastWatchdogReset = now;
    }
    
    // Update sensors
    if (now - lastSensorUpdate >= Config::SENSOR_UPDATE_MS) {
        channelManager.update();
        lastSensorUpdate = now;
    }
    
    // Update UI
    uiManager.update();
    
    // Maintain minimum loop time
    uint32_t elapsed = millis() - now;
    if (elapsed < Config::SCREEN_UPDATE_MS) {
        delay(Config::SCREEN_UPDATE_MS - elapsed);
    }
}
