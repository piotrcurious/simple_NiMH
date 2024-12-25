// screens/channel_detail_screen.h
#pragma once

class ChannelDetailScreen : public Screen {
private:
    uint8_t channelIndex;
    bool showVoltageGraph = true;

    void drawStats() {
        const auto& ch = channels.getChannel(channelIndex);
        
        display.setTextSize(1);
        display.setCursor(0, 10);
        
        // Status line with channel number and state
        display.print(F("CH"));
        display.print(channelIndex + 1);
        display.print(F(" - "));
        switch (ch.state) {
            case ChargeState::CHARGING: display.print(F("CHARGING")); break;
            case ChargeState::DONE: display.print(F("DONE")); break;
            case ChargeState::ERROR: display.print(F("ERROR")); break;
            case ChargeState::MAINTENANCE: display.print(F("MAINT")); break;
            default: display.print(F("IDLE")); break;
        }
        
        // Current measurements
        display.setCursor(0, 20);
        display.print(F("I:"));
        display.print(ch.currentMa, 0);
        display.print(F("mA "));
        display.print(F("V:"));
        display.print(ch.voltageMv, 0);
        display.print(F("mV"));
        
        // Temperature and capacity
        display.setCursor(0, 30);
        display.print(F("ΔT:"));
        display.print(ch.deltaTemp, 1);
        display.print(F("°C "));
        display.print(F("C:"));
        display.print(ch.capacityMah);
        display.print(F("mAh"));
        
        // Peak voltage and -ΔV
        display.setCursor(0, 40);
        display.print(F("Peak:"));
        display.print(ch.peakVoltage);
        display.print(F("mV "));
        display.print(F("-ΔV:"));
        display.print(ch.negDeltaV);
        display.print(F("mV"));
        
        // Time elapsed
        uint32_t elapsed = (millis() - ch.startTime) / 1000;
        display.setCursor(0, 50);
        display.print(F("Time: "));
        display.print(elapsed / 3600);
        display.print(F(":"));
        display.print((elapsed % 3600) / 60);
        display.print(F(":"));
        display.print(elapsed % 60);
    }
    
    void drawGraph() {
        const auto& ch = channels.getChannel(channelIndex);
        display.setTextSize(1);
        
        // Draw graph title
        display.setCursor(70, 10);
        display.print(showVoltageGraph ? F("Voltage") : F("Current"));
        
        // Draw the appropriate graph
        const auto& graph = showVoltageGraph ? ch.voltageHistory : ch.currentHistory;
        drawGraph(graph, 65, 20, 60, 40);
        
        // Draw scale values
        display.setCursor(65, 20);
        display.print(graph.maxValue, 0);
        display.setCursor(65, 52);
        display.print(graph.minValue, 0);
    }

public:
    ChannelDetailScreen(Adafruit_SSD1306& disp, ChannelManager& ch)
        : Screen(disp, ch), channelIndex(0) {}
        
    void setChannel(uint8_t channel) {
        channelIndex = channel;
    }
    
    void draw() override {
        display.clearDisplay();
        
        // Draw title bar
        display.fillRect(0, 0, Config::SCREEN_WIDTH, 8, WHITE);
        display.setTextColor(BLACK);
        display.setTextSize(1);
        display.setCursor(2, 0);
        display.print(F("Channel Details"));
        display.setTextColor(WHITE);
        
        drawStats();
        drawGraph();
        
        // Draw bottom menu
        display.drawFastHLine(0, 56, Config::SCREEN_WIDTH, WHITE);
        display.setTextSize(1);
        display.setCursor(0, 57);
        display.print(F("SEL:"));
        display.print(showVoltageGraph ? F("I") : F("V"));
        display.print(F(" BACK:Menu"));
        
        display.display();
    }
    
    void handleInput(IRCommand cmd) override {
        switch (cmd) {
            case IRCommand::SELECT:
                showVoltageGraph = !showVoltageGraph;
                break;
            default:
                break;
        }
    }
};

// screens/charge_graph_screen.h
#pragma once

class ChargeGraphScreen : public Screen {
private:
    uint8_t channelIndex;
    uint8_t graphTimeScale = 0;  // 0=5min, 1=30min, 2=1hr, 3=all
    static constexpr uint8_t GRAPH_HEIGHT = 45;
    
    void drawTimeScale() {
        display.setTextSize(1);
        display.setCursor(0, 57);
        
        switch (graphTimeScale) {
            case 0: display.print(F("5min")); break;
            case 1: display.print(F("30min")); break;
            case 2: display.print(F("1hr")); break;
            case 3: display.print(F("All")); break;
        }
    }
    
    void drawDualGraph() {
        const auto& ch = channels.getChannel(channelIndex);
        
        // Calculate visible data points based on time scale
        uint16_t points = Config::GRAPH_HISTORY_SIZE;
        switch (graphTimeScale) {
            case 0: points = 30; break;    // 5 minutes (10s samples)
            case 1: points = 180; break;   // 30 minutes
            case 2: points = 360; break;   // 1 hour
            default: break;                // All points
        }
        
        // Draw voltage graph in upper half
        drawGraph(ch.voltageHistory, 20, 10, 100, GRAPH_HEIGHT/2);
        
        // Draw current graph in lower half
        drawGraph(ch.currentHistory, 20, 10 + GRAPH_HEIGHT/2, 100, GRAPH_HEIGHT/2);
        
        // Draw axes labels
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.print(F("V"));
        display.setCursor(0, 10 + GRAPH_HEIGHT/2);
        display.print(F("I"));
    }

public:
    ChargeGraphScreen(Adafruit_SSD1306& disp, ChannelManager& ch)
        : Screen(disp, ch), channelIndex(0) {}
        
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
        display.print(F(" Charge Graph"));
        
        drawDualGraph();
        
        // Draw current values
        const auto& ch = channels.getChannel(channelIndex);
        display.setCursor(70, 0);
        display.print(ch.voltageMv, 0);
        display.print(F("mV "));
        display.print(ch.currentMa, 0);
        display.print(F("mA"));
        
        // Draw bottom menu
        display.drawFastHLine(0, 56, Config::SCREEN_WIDTH, WHITE);
        drawTimeScale();
        display.setCursor(35, 57);
        display.print(F("SEL:Scale BACK:Menu"));
        
        display.display();
    }
    
    void handleInput(IRCommand cmd) override {
        switch (cmd) {
            case IRCommand::SELECT:
                graphTimeScale = (graphTimeScale + 1) % 4;
                break;
            default:
                break;
        }
    }
};

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
