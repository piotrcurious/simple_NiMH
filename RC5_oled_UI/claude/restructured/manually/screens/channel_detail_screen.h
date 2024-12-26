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
