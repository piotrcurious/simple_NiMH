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
