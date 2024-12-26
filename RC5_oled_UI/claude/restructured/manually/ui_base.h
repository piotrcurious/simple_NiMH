// ui_base.h
#pragma once
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "types.h"
#include "channel_manager.h"

class Screen {
protected:
    Adafruit_SSD1306& display;
    ChannelManager& channels;
    
    // Common drawing utilities
    void drawBattery(uint8_t x, uint8_t y, uint8_t level, bool charging) {
        const uint8_t width = 12;
        const uint8_t height = 6;
        const uint8_t tipWidth = 2;
        const uint8_t tipHeight = 2;
        
        // Draw battery outline
        display.drawRect(x, y, width, height, WHITE);
        display.drawRect(x + width, y + (height-tipHeight)/2, tipWidth, tipHeight, WHITE);
        
        // Draw fill level
        uint8_t fillWidth = (width - 2) * level / 100;
        if (fillWidth > 0) {
            display.fillRect(x + 1, y + 1, fillWidth, height - 2, WHITE);
        }
        
        // Draw charging indicator
        if (charging) {
            display.drawLine(x + width/2 - 2, y + height/2, x + width/2, y + 1, WHITE);
            display.drawLine(x + width/2, y + 1, x + width/2 + 2, y + height/2, WHITE);
        }
    }
    
    void drawGraph(const RollingGraph& graph, uint8_t x, uint8_t y, 
                  uint8_t width, uint8_t height) {
        if (graph.values.isEmpty()) return;
        
        // Draw axes
        display.drawFastVLine(x, y, height, WHITE);
        display.drawFastHLine(x, y + height - 1, width, WHITE);
        
        // Calculate points
        uint8_t numPoints = std::min(width - 2, (uint8_t)graph.values.size());
        uint8_t startIdx = graph.values.size() - numPoints;
        
        // Plot points
        for (uint8_t i = 0; i < numPoints; i++) {
            float value = graph.values[startIdx + i];
            float normalized = (value - graph.minValue) * graph.scale;
            uint8_t plotY = y + height - 2 - normalized;
            
            // Draw point
            display.drawPixel(x + 1 + i, plotY, WHITE);
            
            // Connect to previous point if not first point
            if (i > 0) {
                float prevValue = graph.values[startIdx + i - 1];
                float prevNormalized = (prevValue - graph.minValue) * graph.scale;
                uint8_t prevY = y + height - 2 - prevNormalized;
                
                display.drawLine(x + i, prevY, x + 1 + i, plotY, WHITE);
            }
        }
    }
    
    void drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                        uint8_t percentage) {
        // Draw outline
        display.drawRect(x, y, width, height, WHITE);
        
        // Draw fill
        uint8_t fillWidth = (width - 2) * percentage / 100;
        if (fillWidth > 0) {
            display.fillRect(x + 1, y + 1, fillWidth, height - 2, WHITE);
        }
    }
    
    void drawCenteredText(const char* text, uint8_t y) {
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((Config::SCREEN_WIDTH - w) / 2, y);
        display.print(text);
    }
    
    void drawHeader(const char* title) {
        display.fillRect(0, 0, Config::SCREEN_WIDTH, 8, WHITE);
        display.setTextColor(BLACK);
        display.setTextSize(1);
        display.setCursor(2, 0);
        display.print(title);
        display.setTextColor(WHITE);
    }
    
    void drawFooter(const char* text) {
        display.drawFastHLine(0, 56, Config::SCREEN_WIDTH, WHITE);
        display.setTextSize(1);
        display.setCursor(0, 57);
        display.print(text);
    }

public:
    Screen(Adafruit_SSD1306& disp, ChannelManager& ch) 
        : display(disp), channels(ch) {}
    
    virtual ~Screen() = default;
    
    virtual void draw() = 0;
    virtual void handleInput(IRCommand cmd) = 0;
    virtual void update() {} // Optional update method
};
