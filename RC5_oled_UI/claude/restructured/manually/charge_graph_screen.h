
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
