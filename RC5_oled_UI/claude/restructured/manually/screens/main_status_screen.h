// screens/main_status_screen.h
#pragma once

// Enhanced main status screen
class MainStatusScreen : public Screen {
private:
    ChannelStatus* channels;
    uint8_t numChannels;
    uint8_t selectedChannel;
    
    const char* getStateSymbol(ChargeState state) {
        switch(state) {
            case IDLE: return "-";
            case CHARGING: return "↑";
            case DONE: return "✓";
            case ERROR: return "!";
            case MAINTENANCE: return "~";
            default: return "?";
        }
    }
    
    void drawChannel(uint8_t channel, uint8_t x, uint8_t y) {
        // Channel number and state
        display->setCursor(x, y);
        display->print(F("CH"));
        display->print(channel + 1);
        display->print(getStateSymbol(channels[channel].state));
        
        // Battery symbol with charge level
        uint8_t level = map(channels[channel].voltageMv, 1000, 1500, 0, 100);
        drawBattery(x + 25, y, level, channels[channel].state == CHARGING);
        
        // Current and voltage
        display->setCursor(x + 45, y);
        display->print(channels[channel].currentMa, 0);
        display->print(F("mA"));
        
        // Temperature
        if (channels[channel].tempCheckEnabled) {
            display->setCursor(x + 85, y);
            display->print(channels[channel].deltaTemp, 1);
            display->print(F("°"));
        }
    }

public:
    MainStatusScreen(Adafruit_SSD1306* disp, ChannelStatus* ch, uint8_t num) 
        : Screen(disp), channels(ch), numChannels(num), selectedChannel(0) {}
        
    void draw() override {
        display->clearDisplay();
        display->setTextSize(1);
        
        // Title bar
        display->fillRect(0, 0, SCREEN_WIDTH, 8, WHITE);
        display->setTextColor(BLACK);
        display->setCursor(2, 0);
        display->print(F("NiMH Charger v1.0"));
        display->setTextColor(WHITE);
        
        // Draw each channel status
        for (uint8_t i = 0; i < numChannels; i++) {
            drawChannel(i, 0, 12 + i * 12);
        }
        
        // Selection indicator
        display->fillTriangle(
            120, 12 + selectedChannel * 12 + 2,
            124, 12 + selectedChannel * 12 + 4,
            120, 12 + selectedChannel * 12 + 6,
            WHITE
        );
        
        // Bottom status bar
        display->drawFastHLine(0, 56, SCREEN_WIDTH, WHITE);
        display->setCursor(2, 57);
        display->print(F("SEL:Detail  +:Start  -:Stop"));
        
        display->display();
    }
    
    void handleInput(uint16_t command) override {
        switch (command) {
            case IR_UP:
                if (selectedChannel > 0) selectedChannel--;
                break;
            case IR_DOWN:
                if (selectedChannel < numChannels - 1) selectedChannel++;
                break;
            case IR_SELECT:
                // Switch to detail view
                break;
        }
    }
};
