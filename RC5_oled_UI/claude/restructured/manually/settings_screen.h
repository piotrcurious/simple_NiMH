
// screens/settings_screen.h
#pragma once

class SettingsScreen : public Screen {
private:
    enum class Setting {
        CHARGE_CURRENT,
        DELTA_V_THRESHOLD,
        TEMP_CHECK,
        TIMEOUT,
        NUM_SETTINGS
    };
    
    Setting currentSetting = Setting::CHARGE_CURRENT;
    uint8_t channelIndex = 0;
    bool editMode = false;
    
    struct ChannelSettings {
        uint16_t chargeCurrentMa = 1000;
        uint8_t deltaVThresholdMv = 5;
        bool tempCheckEnabled = true;
        uint16_t timeoutMinutes = 240;
    };
    
    ChannelSettings settings[Config::NUM_CHANNELS];
    
    void drawSettingValue(Setting setting, uint8_t y) {
        display.setCursor(70, y);
        
        switch (setting) {
            case Setting::CHARGE_CURRENT:
                display.print(settings[channelIndex].chargeCurrentMa);
                display.print(F("mA"));
                break;
            case Setting::DELTA_V_THRESHOLD:
                display.print(settings[channelIndex].deltaVThresholdMv);
                display.print(F("mV"));
                break;
            case Setting::TEMP_CHECK:
                display.print(settings[channelIndex].tempCheckEnabled ? F("ON") : F("OFF"));
                break;
            case Setting::TIMEOUT:
                display.print(settings[channelIndex].timeoutMinutes);
                display.print(F("min"));
                break;
            default:
                break;
        }
    }
void adjustSetting(bool increase) {
        if (!editMode) return;
        
        switch (currentSetting) {
            case Setting::CHARGE_CURRENT:
                if (increase) {
                    settings[channelIndex].chargeCurrentMa += 100;
                    if (settings[channelIndex].chargeCurrentMa > Config::MAX_CHARGE_CURRENT_MA)
                        settings[channelIndex].chargeCurrentMa = Config::MAX_CHARGE_CURRENT_MA;
                } else {
                    settings[channelIndex].chargeCurrentMa = 
                        std::max(Config::MIN_CHARGE_CURRENT_MA,
                                settings[channelIndex].chargeCurrentMa - 100);
                }
                break;
            case Setting::DELTA_V_THRESHOLD:
                if (increase && settings[channelIndex].deltaVThresholdMv < 10)
                    settings[channelIndex].deltaVThresholdMv++;
                else if (!increase && settings[channelIndex].deltaVThresholdMv > 2)
                    settings[channelIndex].deltaVThresholdMv--;
                break;
            case Setting::TEMP_CHECK:
                settings[channelIndex].tempCheckEnabled = 
                    !settings[channelIndex].tempCheckEnabled;
                break;
            case Setting::TIMEOUT:
                if (increase)
                    settings[channelIndex].timeoutMinutes += 30;
                else
                    settings[channelIndex].timeoutMinutes = 
                        std::max(30, settings[channelIndex].timeoutMinutes - 30);
                break;
            default:
                break;
        }
    }

public:
    SettingsScreen(Adafruit_SSD1306& disp, ChannelManager& ch)
        : Screen(disp, ch) {}
        
    void setChannel(uint8_t channel) {
        channelIndex = channel;
    }
    
    void draw() override {
        display.clearDisplay();
        
        // Draw title
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print(F("CH"));
        display.print(channelIndex + 1);
        display.print(F(" Settings"));
        
        // Draw settings list
        const char* settingNames[] = {
            "Charge Current",
            "Delta-V",
            "Temp Check",
            "Timeout"
        };
        
        for (int i = 0; i < static_cast<int>(Setting::NUM_SETTINGS); i++) {
            uint8_t y = 15 + i * 10;
            
            // Selection indicator
            if (i == static_cast<int>(currentSetting)) {
                display.fillRect(0, y - 1, editMode ? 120 : 60, 9, INVERSE);
            }
            
            display.setCursor(2, y);
            display.print(settingNames[i]);
            drawSettingValue(static_cast<Setting>(i), y);
        }
        
        // Draw bottom menu
        display.drawFastHLine(0, 56, Config::SCREEN_WIDTH, WHITE);
        display.setCursor(0, 57);
        display.print(editMode ? F("±:Adj SEL:Save") : F("SEL:Edit BACK:Menu"));
        
        display.display();
    }
    void handleInput(IRCommand cmd) override {
        switch (cmd) {
            case IRCommand::UP:
                if (!editMode && currentSetting > Setting::CHARGE_CURRENT)
                    currentSetting = static_cast<Setting>(
                        static_cast<int>(currentSetting) - 1);
                break;
            case IRCommand::DOWN:
                if (!editMode && currentSetting < Setting::NUM_SETTINGS - 1)
                    currentSetting = static_cast<Setting>(
                        static_cast<int>(currentSetting) + 1);
                break;
            case IRCommand::SELECT:
                editMode = !editMode;
                break;
            case IRCommand::PLUS:
                adjustSetting(true);
                break;
            case IRCommand::MINUS:
                adjustSetting(false);
                break;
            default:
                break;
        }
    }
    
    // Method to apply settings to a channel
    void applySettings() {
        auto& ch = channels.getChannel(channelIndex);
        ch.targetCurrentMa = settings[channelIndex].chargeCurrentMa;
        ch.tempCheckEnabled = settings[channelIndex].tempCheckEnabled;
        // Other settings would be applied through the channel manager
    }
};
