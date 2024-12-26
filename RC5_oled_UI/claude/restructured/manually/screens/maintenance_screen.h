// screens/maintenance_screen.h
#pragma once

// Maintenance screen with real-time voltage graph and pulse control
class MaintenanceScreen : public Screen {
private:
    ChannelStatus* channel;
    uint16_t pulseCurrentMa;
    uint32_t lastPulseTime;
    bool pulsing;
    
    void drawControls() {
        display->drawFastHLine(0, 56, SCREEN_WIDTH, WHITE);
        display->setCursor(2, 57);
        display->print(F("±:"));
        display->print(pulseCurrentMa);
        display->print(F("mA  SEL:Pulse"));
    }

public:
    MaintenanceScreen(Adafruit_SSD1306* disp, ChannelStatus* ch) 
        : Screen(disp), channel(ch), pulseCurrentMa(100), lastPulseTime(0), pulsing(false) {}
        
    void draw() override {
        display->clearDisplay();
        display->setTextSize(1);
        
        // Title
        display->setCursor(0, 0);
        display->print(F("Maintenance - CH"));
        display->print((channel - &channel[0]) + 1);
        
        // Current voltage
        display->setCursor(80, 0);
        display->print(channel->voltageMv);
        display->print(F("mV"));
        
        // Voltage graph
        drawGraph(channel->voltageHistory, 20, 10, 88, 40);
        
        // Pulse status
        display->setCursor(2, 45);
        display->print(F("Pulse: "));
        display->print(pulsing ? F("ON") : F("OFF"));
        if (pulsing) {
            display->print(F(" "));
            display->print((millis() - lastPulseTime) / 1000);
            display->print(F("s"));
        }
        
        drawControls();
        
        display->display();
    }
    
    void handleInput(uint16_t command) override {
        switch (command) {
            case IR_PLUS:
                pulseCurrentMa += 50;
                if (pulseCurrentMa > 1000) pulseCurrentMa = 1000;
                break;
            case IR_MINUS:
                pulseCurrentMa -= 50;
                if (pulseCurrentMa < 50) pulseCurrentMa = 50;
                break;
            case IR_SELECT:
                if (!pulsing) {
                    pulsing = true;
                    lastPulseTime = millis();
                    channel->targetCurrentMa = pulseCurrentMa;
                } else {
                    pulsing = false;
                    channel->targetCurrentMa = 0;
                }
                break;
        }
    }
    void update() {
        // Check pulse duration
        if (pulsing && (millis() - lastPulseTime) > 5000) {  // 5 second pulse
            pulsing = false;
            channel->targetCurrentMa = 0;
        }
        
        // Add voltage reading to history
        channel->voltageHistory.add(channel->voltageMv);
    }
};
