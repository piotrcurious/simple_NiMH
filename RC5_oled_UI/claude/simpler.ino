#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <CircularBuffer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define IR_RECEIVE_PIN 15

// IR Remote commands (example for a standard remote)
enum IRCommands {
    IR_UP = 0x01,
    IR_DOWN = 0x02,
    IR_LEFT = 0x03,
    IR_RIGHT = 0x04,
    IR_SELECT = 0x05,
    IR_BACK = 0x06,
    IR_1 = 0x11,
    IR_2 = 0x12,
    IR_3 = 0x13,
    IR_4 = 0x14,
    IR_PLUS = 0x1A,
    IR_MINUS = 0x1B
};

// Screen IDs
enum ScreenID {
    MAIN_STATUS,
    CHANNEL_DETAIL,
    CHARGE_GRAPH,
    MAINTENANCE,
    SETTINGS
};

// Battery states
enum ChargeState {
    IDLE,
    CHARGING,
    DONE,
    ERROR,
    MAINTENANCE
};

// Rolling graph data structure
struct RollingGraph {
    CircularBuffer<float, 100> values;
    float minValue;
    float maxValue;
    float scale;
    
    void add(float value) {
        values.push(value);
        updateBounds();
    }
    
    void updateBounds() {
        minValue = values[0];
        maxValue = values[0];
        for(size_t i = 0; i < values.size(); i++) {
            minValue = min(minValue, values[i]);
            maxValue = max(maxValue, values[i]);
        }
        scale = (maxValue - minValue) > 0 ? 40.0 / (maxValue - minValue) : 1.0;
    }
};

// Enhanced channel status
struct ChannelStatus {
    float currentMa;
    float voltageMv;
    float batteryTemp;
    float deltaTemp;
    ChargeState state;
    uint32_t startTime;
    uint32_t lastUpdateTime;
    uint16_t capacityMah;
    RollingGraph voltageHistory;
    RollingGraph currentHistory;
    uint16_t peakVoltage;
    uint8_t negDeltaV;
    bool tempCheckEnabled;
    uint16_t targetCurrentMa;
};

// Screen base class
class Screen {
protected:
    Adafruit_SSD1306* display;
    
    void drawBattery(uint8_t x, uint8_t y, uint8_t level, bool charging) {
        display->drawRect(x, y, 12, 6, WHITE);
        display->drawPixel(x + 12, y + 2, WHITE);
        display->drawPixel(x + 12, y + 3, WHITE);
        
        if (level > 0) {
            uint8_t width = map(level, 0, 100, 0, 10);
            display->fillRect(x + 1, y + 1, width, 4, WHITE);
        }
        
        if (charging) {
            display->drawPixel(x + 5, y - 1, WHITE);
            display->drawPixel(x + 6, y - 1, WHITE);
        }
    }
    
    void drawGraph(RollingGraph& graph, uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
        // Draw axes
        display->drawLine(x, y + height, x + width, y + height, WHITE);
        display->drawLine(x, y, x, y + height, WHITE);
        
        // Draw graph lines
        for (uint8_t i = 0; i < width - 1 && i < graph.values.size() - 1; i++) {
            uint8_t x1 = x + i;
            uint8_t x2 = x + i + 1;
            uint8_t y1 = y + height - ((graph.values[i] - graph.minValue) * graph.scale);
            uint8_t y2 = y + height - ((graph.values[i + 1] - graph.minValue) * graph.scale);
            display->drawLine(x1, y1, x2, y2, WHITE);
        }
        
        // Draw min/max values
        display->setTextSize(1);
        display->setCursor(x - 20, y);
        display->print(graph.maxValue, 1);
        display->setCursor(x - 20, y + height - 6);
        display->print(graph.minValue, 1);
    }
    
public:
    Screen(Adafruit_SSD1306* disp) : display(disp) {}
    virtual void draw() = 0;
    virtual void handleInput(uint16_t command) = 0;
};

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

// Detailed channel view
class ChannelDetailScreen : public Screen {
private:
    ChannelStatus* channel;
    
    void drawStats() {
        display->setCursor(0, 10);
        display->print(F("Current: "));
        display->print(channel->currentMa);
        display->print(F("mA\nVoltage: "));
        display->print(channel->voltageMv);
        display->print(F("mV\nTemp Δ: "));
        display->print(channel->deltaTemp);
        display->print(F("°C\nCapacity: "));
        display->print(channel->capacityMah);
        display->print(F("mAh\nPeak V: "));
        display->print(channel->peakVoltage);
        display->print(F("mV\n-ΔV: "));
        display->print(channel->negDeltaV);
        display->print(F("mV"));
    }

public:
    ChannelDetailScreen(Adafruit_SSD1306* disp, ChannelStatus* ch) 
        : Screen(disp), channel(ch) {}
        
    void draw() override {
        display->clearDisplay();
        display->setTextSize(1);
        
        // Title
        display->setCursor(0, 0);
        display->print(F("Channel "));
        display->print((channel - &channel[0]) + 1);
        display->print(F(" Details"));
        
        drawStats();
        
        // Draw mini graphs
        display->drawFastHLine(64, 10, 64, WHITE);
        drawGraph(channel->voltageHistory, 64, 12, 64, 20);
        drawGraph(channel->currentHistory, 64, 34, 64, 20);
        
        display->display();
    }
    
    void handleInput(uint16_t command) override {
        switch (command) {
            case IR_BACK:
                // Return to main screen
                break;
        }
    }
};

// Main UI controller with enhanced features
class BatteryChargerUI {
private:
    Adafruit_SSD1306 display;
    IRrecv irReceiver;
    Screen* currentScreen;
    Screen* screens[5];  // Array of available screens
    ChannelStatus channels[4];  // Support for 4 charging channels
    float ambientTemp;
    uint32_t lastUpdate;
    
    void setupChannels() {
        for (uint8_t i = 0; i < 4; i++) {
            channels[i].currentMa = 0;
            channels[i].voltageMv = 0;
            channels[i].batteryTemp = 0;
            channels[i].deltaTemp = 0;
            channels[i].state = IDLE;
            channels[i].startTime = 0;
            channels[i].lastUpdateTime = 0;
            channels[i].capacityMah = 0;
            channels[i].peakVoltage = 0;
            channels[i].negDeltaV = 0;
            channels[i].tempCheckEnabled = true;
            channels[i].targetCurrentMa = 0;
        }
    }
    
    void readSensors() {
        // Example sensor reading code - replace with actual implementation
        for (uint8_t i = 0; i < 4; i++) {
            if (channels[i].state == CHARGING || channels[i].state == MAINTENANCE) {
                // Read voltage (example using ADC)
                float rawVoltage = analogRead(32 + i) * 3.3 / 4095.0;
                channels[i].voltageMv = rawVoltage * 1000;  // Convert to mV
                
                // Read current
                float rawCurrent = analogRead(36 + i) * 3.3 / 4095.0;
                channels[i].currentMa = rawCurrent * 1000;  // Convert to mA
                
                // Read temperature (continued from previous part)
                channels[i].batteryTemp = readTemperature(i);
                channels[i].deltaTemp = channels[i].batteryTemp - ambientTemp;
                
                // Update histories
                channels[i].voltageHistory.add(channels[i].voltageMv);
                channels[i].currentHistory.add(channels[i].currentMa);
                
                // Update capacity
                uint32_t deltaTime = millis() - channels[i].lastUpdateTime;
                channels[i].capacityMah += 
                    (channels[i].currentMa * deltaTime) / 3600000.0;
                
                // Check for peak voltage and -ΔV
                if (channels[i].voltageMv > channels[i].peakVoltage) {
                    channels[i].peakVoltage = channels[i].voltageMv;
                } else if (channels[i].peakVoltage - channels[i].voltageMv > 5) {
                    channels[i].negDeltaV = channels[i].peakVoltage - channels[i].voltageMv;
                }
                
                channels[i].lastUpdateTime = millis();
            }
        }
        
        // Read ambient temperature
        ambientTemp = readTemperature(4);  // Separate sensor for ambient
    }
    
    float readTemperature(uint8_t sensor) {
        // Example temperature reading - replace with actual sensor code
        float rawTemp = analogRead(25 + sensor) * 3.3 / 4095.0;
        return (rawTemp - 0.5) * 100.0;  // Example conversion for LM35
    }
    
    void checkChargeTermination(uint8_t channel) {
        ChannelStatus& ch = channels[channel];
        
        if (ch.state != CHARGING) return;
        
        bool terminate = false;
        
        // Check -ΔV termination
        if (ch.negDeltaV >= 5) {  // 5mV drop
            terminate = true;
        }
        
        // Check temperature rise
        if (ch.tempCheckEnabled && ch.deltaTemp > 2.0) {  // 2°C rise
            terminate = true;
        }
        
        // Check timeout
        if (millis() - ch.startTime > 14400000) {  // 4 hour timeout
            terminate = true;
        }
        
        if (terminate) {
            stopCharging(channel);
            ch.state = DONE;
        }
    }
    
    void controlCharging(uint8_t channel) {
        ChannelStatus& ch = channels[channel];
        
        if (ch.state == CHARGING || ch.state == MAINTENANCE) {
            // Example PWM control - replace with actual hardware control
            uint32_t pwmValue = map(ch.targetCurrentMa, 0, 2000, 0, 255);
            analogWrite(16 + channel, pwmValue);  // PWM pins for each channel
        } else {
            analogWrite(16 + channel, 0);  // Turn off charging
        }
    }

public:
    BatteryChargerUI() : 
        display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
        irReceiver(IR_RECEIVE_PIN),
        lastUpdate(0) {
        setupChannels();
    }
    
    void begin() {
        // Initialize display
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        display.clearDisplay();
        display.setTextColor(WHITE);
        
        // Initialize IR receiver
        irReceiver.enableIRIn();
        
        // Setup PWM outputs
        for (uint8_t i = 0; i < 4; i++) {
            pinMode(16 + i, OUTPUT);
            analogWrite(16 + i, 0);
        }
        
        // Setup screens
        screens[MAIN_STATUS] = new MainStatusScreen(&display, channels, 4);
        screens[CHANNEL_DETAIL] = new ChannelDetailScreen(&display, &channels[0]);
        screens[CHARGE_GRAPH] = new ChargeGraphScreen(&display, &channels[0]);
        screens[MAINTENANCE] = new MaintenanceScreen(&display, &channels[0]);
        screens[SETTINGS] = nullptr;  // To be implemented if needed
        
        currentScreen = screens[MAIN_STATUS];
    }
    
    void update() {
        uint32_t now = millis();
        
        // Handle IR input
        if (irReceiver.decode()) {
            handleIRCommand(irReceiver.decodedIRData.command);
            irReceiver.resume();
        }
        
        // Update sensors and charge control at regular intervals
        if (now - lastUpdate >= 1000) {  // 1 second update interval
            readSensors();
            
            for (uint8_t i = 0; i < 4; i++) {
                checkChargeTermination(i);
                controlCharging(i);
            }
            
            lastUpdate = now;
        }
        
        // Update maintenance screen if active
        if (currentScreen == screens[MAINTENANCE]) {
            ((MaintenanceScreen*)currentScreen)->update();
        }
        
        // Draw current screen
        currentScreen->draw();
    }
    
    void handleIRCommand(uint16_t command) {
        // Global commands
        switch (command) {
            case IR_BACK:
                setScreen(MAIN_STATUS);
                return;
            case IR_1:
            case IR_2:
            case IR_3:
            case IR_4:
                selectChannel(command - IR_1);
                return;
        }
        
        // Pass other commands to current screen
        currentScreen->handleInput(command);
    }
    
    void selectChannel(uint8_t channel) {
        if (channel >= 4) return;
        
        // Update channel-specific screens
        ((ChannelDetailScreen*)screens[CHANNEL_DETAIL])->setChannel(&channels[channel]);
        ((MaintenanceScreen*)screens[MAINTENANCE])->setChannel(&channels[channel]);
    }
    
    void startCharging(uint8_t channel, uint16_t current) {
        if (channel >= 4) return;
        
        channels[channel].state = CHARGING;
        channels[channel].startTime = millis();
        channels[channel].lastUpdateTime = millis();
        channels[channel].capacityMah = 0;
        channels[channel].peakVoltage = 0;
        channels[channel].negDeltaV = 0;
        channels[channel].targetCurrentMa = current;
    }
    
    void stopCharging(uint8_t channel) {
        if (channel >= 4) return;
        
        channels[channel].state = IDLE;
        channels[channel].targetCurrentMa = 0;
    }
    
    void setScreen(ScreenID id) {
        if (screens[id] != nullptr) {
            currentScreen = screens[id];
        }
    }
};

// Main program
BatteryChargerUI chargerUI;

void setup() {
    Wire.begin();
    chargerUI.begin();
}

void loop() {
    chargerUI.update();
    delay(50);  // Small delay to prevent display flickering
}
