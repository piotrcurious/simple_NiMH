// Add EEPROM support for settings persistence
#include <EEPROM.h>

// Add error handling enum
enum class ErrorCode {
    NONE,
    OVER_TEMP,
    OVER_VOLTAGE,
    UNDER_VOLTAGE,
    CONNECTION_ERROR,
    TIMEOUT
};

// Add error messages
const char* getErrorMessage(ErrorCode error) {
    switch (error) {
        case ErrorCode::OVER_TEMP: return "OVER TEMP";
        case ErrorCode::OVER_VOLTAGE: return "OVER VOLT";
        case ErrorCode::UNDER_VOLTAGE: return "UNDER VOLT";
        case ErrorCode::CONNECTION_ERROR: return "NO BATT";
        case ErrorCode::TIMEOUT: return "TIMEOUT";
        default: return "OK";
    }
}

// Add battery detection and safety thresholds to config.h
namespace Config {
    constexpr float MIN_BATTERY_VOLTAGE_MV = 800.0f;
    constexpr float MAX_BATTERY_VOLTAGE_MV = 1600.0f;
    constexpr float MAX_BATTERY_TEMP_C = 45.0f;
    constexpr float MIN_AMBIENT_TEMP_C = 5.0f;
    constexpr float MAX_AMBIENT_TEMP_C = 40.0f;
    constexpr uint16_t BATTERY_DETECT_THRESHOLD_MV = 500;
    constexpr uint32_t ERROR_RETRY_TIME_MS = 300000; // 5 minutes
    constexpr uint16_t MAINTENANCE_INTERVAL_MS = 3600000; // 1 hour
    constexpr uint16_t MAINTENANCE_DURATION_MS = 300000; // 5 minutes
}

// Add error handling to ChannelData struct
struct ChannelData {
    // ... existing members ...
    ErrorCode error = ErrorCode::NONE;
    uint32_t errorTime = 0;
    uint32_t maintenanceTime = 0;
    bool maintenanceActive = false;
};

// Add calibration data structure
struct CalibrationData {
    float voltageOffset[Config::NUM_CHANNELS] = {0};
    float voltageScale[Config::NUM_CHANNELS] = {1.0f};
    float currentOffset[Config::NUM_CHANNELS] = {0};
    float currentScale[Config::NUM_CHANNELS] = {1.0f};
    float tempOffset[Config::NUM_CHANNELS + 1] = {0};
};

// Add calibration to ESP32Hardware
class ESP32Hardware : public HardwareManager {
private:
    CalibrationData calibration;
    
    void loadCalibration() {
        EEPROM.begin(512);
        EEPROM.get(0, calibration);
        EEPROM.end();
    }
    
public:
    void begin() override {
        loadCalibration();
        
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            pinMode(pwmPins[i], OUTPUT);
            analogWrite(pwmPins[i], 0);
        }
        
        // Configure ADC
        analogReadResolution(12);
        analogSetAttenuation(ADC_11db);
    }
    
    float readVoltage(uint8_t channel) override {
        if (channel >= Config::NUM_CHANNELS) return 0.0f;
        
        float raw = analogRead(voltagePins[channel]) * 3.3f / 4095.0f;
        float voltage = (raw + calibration.voltageOffset[channel]) * 
                       calibration.voltageScale[channel];
        return voltage * 1000.0f; // Convert to mV
    }
    
    float readCurrent(uint8_t channel) override {
        if (channel >= Config::NUM_CHANNELS) return 0.0f;
        
        float raw = analogRead(currentPins[channel]) * 3.3f / 4095.0f;
        float current = (raw + calibration.currentOffset[channel]) * 
                       calibration.currentScale[channel];
        return current * 1000.0f; // Convert to mA
    }
};

// Add error handling and safety checks to ChannelManager
class ChannelManager {
private:
    void checkSafety(uint8_t channel) {
        auto& ch = channels[channel];
        if (ch.state != ChargeState::CHARGING && 
            ch.state != ChargeState::MAINTENANCE) return;
            
        ErrorCode newError = ErrorCode::NONE;
        
        // Check temperature limits
        if (ch.batteryTemp > Config::MAX_BATTERY_TEMP_C) {
            newError = ErrorCode::OVER_TEMP;
        }
        
        // Check voltage limits
        if (ch.voltageMv > Config::MAX_BATTERY_VOLTAGE_MV) {
            newError = ErrorCode::OVER_VOLTAGE;
        } else if (ch.voltageMv < Config::MIN_BATTERY_VOLTAGE_MV && 
                  ch.currentMa > Config::MIN_CHARGE_CURRENT_MA) {
            newError = ErrorCode::UNDER_VOLTAGE;
        }
        
        // Check for battery presence
        if (ch.voltageMv < Config::BATTERY_DETECT_THRESHOLD_MV) {
            newError = ErrorCode::CONNECTION_ERROR;
        }
        
        // Check timeout
        if (millis() - ch.startTime > Config::MAX_CHARGE_TIME_MS) {
            newError = ErrorCode::TIMEOUT;
        }
        
        // Handle error state changes
        if (newError != ErrorCode::NONE) {
            if (ch.error == ErrorCode::NONE) {
                ch.error = newError;
                ch.errorTime = millis();
                stopCharging(channel);
            }
        } else {
            ch.error = ErrorCode::NONE;
        }
    }
    
    void handleMaintenance(uint8_t channel) {
        auto& ch = channels[channel];
        if (ch.state != ChargeState::MAINTENANCE) return;
        
        uint32_t now = millis();
        
        if (ch.maintenanceActive) {
            if (now - ch.maintenanceTime >= Config::MAINTENANCE_DURATION_MS) {
                ch.maintenanceActive = false;
                ch.targetCurrentMa = 0;
                ch.maintenanceTime = now;
            }
        } else {
            if (now - ch.maintenanceTime >= Config::MAINTENANCE_INTERVAL_MS) {
                ch.maintenanceActive = true;
                ch.targetCurrentMa = Config::MIN_CHARGE_CURRENT_MA;
                ch.maintenanceTime = now;
            }
        }
    }
    
public:
    void update() override {
        ambientTemp = hardware.readTemperature(Config::NUM_CHANNELS);
        
        // Check ambient temperature
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
            
            // Try to recover from errors after timeout
            auto& ch = channels[i];
            if (ch.error != ErrorCode::NONE) {
                if (millis() - ch.errorTime >= Config::ERROR_RETRY_TIME_MS) {
                    ch.error = ErrorCode::NONE;
                }
            }
            
            // Update hardware
            hardware.setChargeCurrent(i, ch.maintenanceActive ? 
                Config::MIN_CHARGE_CURRENT_MA : ch.targetCurrentMa);
        }
    }
};

// Add settings persistence to SettingsScreen
class SettingsScreen : public Screen {
private:
    static constexpr uint16_t SETTINGS_EEPROM_ADDR = 256;
    
    void saveSettings() {
        EEPROM.begin(512);
        EEPROM.put(SETTINGS_EEPROM_ADDR, settings);
        EEPROM.commit();
        EEPROM.end();
    }
    
    void loadSettings() {
        EEPROM.begin(512);
        EEPROM.get(SETTINGS_EEPROM_ADDR, settings);
        EEPROM.end();
        
        // Validate loaded settings
        for (auto& setting : settings) {
            if (setting.chargeCurrentMa > Config::MAX_CHARGE_CURRENT_MA) {
                setting.chargeCurrentMa = Config::MAX_CHARGE_CURRENT_MA;
            }
            if (setting.chargeCurrentMa < Config::MIN_CHARGE_CURRENT_MA) {
                setting.chargeCurrentMa = Config::MIN_CHARGE_CURRENT_MA;
            }
        }
    }
    
public:
    SettingsScreen(Adafruit_SSD1306& disp, ChannelManager& ch) 
        : Screen(disp, ch) {
        loadSettings();
    }
    
    void handleInput(IRCommand cmd) override {
        // ... existing implementation ...
        
        if (cmd == IRCommand::SELECT && editMode) {
            saveSettings();
        }
    }
};

// Add memory management improvements to UIManager
class UIManager {
private:
    static constexpr size_t SCREEN_BUFFER_SIZE = 1024;
    uint8_t screenBuffer[SCREEN_BUFFER_SIZE];
    
public:
    UIManager(IRrecv& ir, ChannelManager& cm) 
        : irReceiver(ir), channelManager(cm), lastUpdate(0) {
        display = Adafruit_SSD1306(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, 
                                 &Wire, Config::OLED_RESET);
        display.setBuffer(screenBuffer, SCREEN_BUFFER_SIZE);
    }
};

// Add watchdog timer support
#include <esp_task_wdt.h>

void setup() {
    // Enable watchdog with 5 second timeout
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);
    
    Wire.begin();
    hardware.begin();
    irReceiver.enableIRIn();
    uiManager.begin();
    
    // Load settings from EEPROM
    EEPROM.begin(512);
    // ... load settings ...
    EEPROM.end();
}

void loop() {
    static uint32_t lastSensorUpdate = 0;
    static uint32_t lastWatchdogReset = 0;
    uint32_t now = millis();
    
    // Reset watchdog timer periodically
    if (now - lastWatchdogReset >= 1000) {
        esp_task_wdt_reset();
        lastWatchdogReset = now;
    }
    
    // Update sensors at regular interval
    if (now - lastSensorUpdate >= Config::SENSOR_UPDATE_MS) {
        channelManager.update();
        lastSensorUpdate = now;
    }
    
    // Update UI
    uiManager.update();
    delay(Config::SCREEN_UPDATE_MS);
}
