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
