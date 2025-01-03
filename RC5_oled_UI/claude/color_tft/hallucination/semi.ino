#include <TFT_eSPI.h>
#include <IRremote.h>
#include <vector>
#include <memory>

// Display setup
TFT_eSPI tft = TFT_eSPI();
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// IR remote codes (example for typical RC5 remote)
enum IRCommand {
    CMD_UP = 0x10,
    CMD_DOWN = 0x11,
    CMD_LEFT = 0x15,
    CMD_RIGHT = 0x16,
    CMD_SELECT = 0x17,
    CMD_MENU = 0x12,
    CMD_BACK = 0x13,
    CMD_0 = 0x00,
    CMD_1 = 0x01,
    CMD_2 = 0x02,
    CMD_3 = 0x03,
};

// UI Colors
namespace UIColors {
    const uint16_t BACKGROUND = TFT_BLACK;
    const uint16_t TEXT_PRIMARY = TFT_WHITE;
    const uint16_t TEXT_SECONDARY = TFT_LIGHTGREY;
    const uint16_t ACCENT = TFT_CYAN;
    const uint16_t WARNING = TFT_YELLOW;
    const uint16_t ERROR = TFT_RED;
    const uint16_t SUCCESS = TFT_GREEN;
    const uint16_t GRAPH_LINE = TFT_CYAN;
    const uint16_t BATTERY_GOOD = TFT_GREEN;
    const uint16_t BATTERY_LOW = TFT_RED;
}

// Battery Parameters
struct BatteryParams {
    float nominalVoltage = 1.2f;  // NiMH cell voltage
    float maxVoltage = 1.5f;      // Max charging voltage
    float minVoltage = 0.9f;      // Min safe voltage
    float maxCurrent = 2.0f;      // Max charging current
    float trickleCurrent = 0.05f; // Trickle charge current
    float deltaV = -0.005f;       // -dV detection threshold
    float deltaTemp = 1.5f;       // Temperature rise threshold
};

// Channel state and data
class ChargingChannel {
public:
    enum State {
        IDLE,
        CHARGING,
        DISCHARGING,
        TRICKLE,
        ERROR,
        COMPLETE
    };

    struct Measurements {
        float voltage = 0;
        float current = 0;
        float temperature = 0;
        float capacity = 0;     // mAh
        uint32_t timestamp = 0; // millis() timestamp
    };

    State state = IDLE;
    BatteryParams params;
    float targetCurrent = 0;
    float rampRate = 0.1;      // A/s
    uint32_t stateTime = 0;    // Time in current state
    bool selfTestEnabled = false;
    String errorMessage;
    
    static const size_t HISTORY_SIZE = 300; // 5 minutes at 1s intervals
    std::vector<Measurements> history;
    
    // Latest measurements
    Measurements current;
    
    ChargingChannel() {
        history.reserve(HISTORY_SIZE);
    }
    
    void update(float v, float i, float t) {
        current.voltage = v;
        current.current = i;
        current.temperature = t;
        current.timestamp = millis();
        
        if(history.size() >= HISTORY_SIZE) {
            history.erase(history.begin());
        }
        history.push_back(current);
        
        updateState();
    }
    
private:
    void updateState() {
        if(history.size() < 2) return;
        
        switch(state) {
            case CHARGING:
                checkChargingTermination();
                break;
            case DISCHARGING:
                checkDischargingTermination();
                break;
            case ERROR:
                checkErrorRecovery();
                break;
            default:
                break;
        }
    }
    
    void checkChargingTermination() {
        // -dV detection
        if(history.size() >= 5) {
            float avgDelta = 0;
            for(size_t i = 1; i < 5; i++) {
                avgDelta += history[history.size()-i].voltage - history[history.size()-i-1].voltage;
            }
            avgDelta /= 4;
            
            if(avgDelta < params.deltaV) {
                state = COMPLETE;
                targetCurrent = params.trickleCharge;
            }
        }
        
        // Temperature rise check
        if(current.temperature - history[0].temperature > params.deltaTemp) {
            state = COMPLETE;
            targetCurrent = params.trickleCharge;
        }
        
        // Error conditions
        if(current.voltage > params.maxVoltage || 
           current.current > params.maxCurrent ||
           current.temperature > 45.0f) {
            state = ERROR;
            errorMessage = "Safety limit exceeded";
            targetCurrent = 0;
        }
    }
    
    void checkDischargingTermination() {
        if(current.voltage < params.minVoltage) {
            state = COMPLETE;
            targetCurrent = 0;
        }
    }
    
    void checkErrorRecovery() {
        if(current.voltage < params.maxVoltage && 
           current.temperature < 40.0f) {
            state = IDLE;
            errorMessage = "";
        }
    }
};

// UI Elements
class UIElement {
protected:
    TFT_eSPI& display;
    int16_t x, y, width, height;
    bool visible = true;
    uint16_t bgColor = UIColors::BACKGROUND;
    
public:
    UIElement(TFT_eSPI& d, int16_t x_, int16_t y_, int16_t w, int16_t h) 
        : display(d), x(x_), y(y_), width(w), height(h) {}
    
    virtual void draw() = 0;
    virtual void setPosition(int16_t x_, int16_t y_) {
        x = x_;
        y = y_;
    }
    virtual void setSize(int16_t w, int16_t h) {
        width = w;
        height = h;
    }
    void setVisible(bool v) { visible = v; }
    virtual ~UIElement() {}
};

class StatusBar : public UIElement {
    String title;
    String status;
    
public:
    StatusBar(TFT_eSPI& d) : UIElement(d, 0, 0, SCREEN_WIDTH, 20) {}
    
    void draw() override {
        display.fillRect(x, y, width, height, UIColors::ACCENT);
        display.setTextColor(UIColors::BACKGROUND);
        display.drawString(title, x + 5, y + 2);
        display.drawString(status, x + width - 50, y + 2);
    }
    
    void setTitle(const String& t) { title = t; }
    void setStatus(const String& s) { status = s; }
};

class BatteryDisplay : public UIElement {
    ChargingChannel& channel;
    bool selected = false;
    
public:
    BatteryDisplay(TFT_eSPI& d, ChargingChannel& ch, int16_t x_, int16_t y_) 
        : UIElement(d, x_, y_, 120, 80), channel(ch) {}
    
    void draw() override {
        // Draw border
        uint16_t borderColor = selected ? UIColors::ACCENT : UIColors::TEXT_SECONDARY;
        display.drawRect(x, y, width, height, borderColor);
        
        // Draw battery symbol
        drawBatterySymbol(x + 5, y + 5, 30, 15);
        
        // Draw measurements
        display.setTextColor(UIColors::TEXT_PRIMARY);
        display.drawString(String(channel.current.voltage, 3) + "V", x + 5, y + 25);
        display.drawString(String(channel.current.current, 3) + "A", x + 5, y + 40);
        display.drawString(String(channel.current.temperature, 1) + "°C", x + 5, y + 55);
        
        // Draw state
        String stateStr;
        uint16_t stateColor;
        switch(channel.state) {
            case ChargingChannel::CHARGING:
                stateStr = "CHRG";
                stateColor = UIColors::SUCCESS;
                break;
            case ChargingChannel::DISCHARGING:
                stateStr = "DSCH";
                stateColor = UIColors::WARNING;
                break;
            case ChargingChannel::ERROR:
                stateStr = "ERR";
                stateColor = UIColors::ERROR;
                break;
            case ChargingChannel::COMPLETE:
                stateStr = "DONE";
                stateColor = UIColors::SUCCESS;
                break;
            default:
                stateStr = "IDLE";
                stateColor = UIColors::TEXT_SECONDARY;
        }
        display.setTextColor(stateColor);
        display.drawString(stateStr, x + width - 40, y + 5);
    }
    
    void setSelected(bool sel) { selected = sel; }
    
private:
    void drawBatterySymbol(int16_t x, int16_t y, int16_t w, int16_t h) {
        display.drawRect(x, y, w, h, UIColors::TEXT_PRIMARY);
        display.fillRect(x + w, y + h/4, 3, h/2, UIColors::TEXT_PRIMARY);
        
        float fillPercent = (channel.current.voltage - channel.params.minVoltage) / 
                           (channel.params.maxVoltage - channel.params.minVoltage);
        fillPercent = constrain(fillPercent, 0.0f, 1.0f);
        
        uint16_t fillColor = (fillPercent > 0.3f) ? UIColors::BATTERY_GOOD : UIColors::BATTERY_LOW;
        display.fillRect(x + 1, y + 1, (w-2) * fillPercent, h-2, fillColor);
    }
};

class Graph : public UIElement {
    std::vector<ChargingChannel::Measurements>& history;
    bool showVoltage = true;
    bool showCurrent = true;
    bool showTemp = false;
    float timeScale = 300.0f; // seconds
    
public:
    Graph(TFT_eSPI& d, std::vector<ChargingChannel::Measurements>& h) 
        : UIElement(d, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT - 40), history(h) {}
    
    void draw() override {
        if(history.empty()) return;
        
        // Draw axes
        display.drawLine(30, y, 30, y + height - 20, UIColors::TEXT_SECONDARY); // Y axis
        display.drawLine(30, y + height - 20, width - 10, y + height - 20, UIColors::TEXT_SECONDARY); // X axis
        
        // Draw grid
        for(int i = 1; i < 5; i++) {
            int16_t yPos = y + (height - 20) * i / 5;
            display.drawLine(28, yPos, 32, yPos, UIColors::TEXT_SECONDARY);
        }
        
        // Plot data
        if(showVoltage) plotData(UIColors::SUCCESS, 0.8f, 1.6f, 
            [](const ChargingChannel::Measurements& m) { return m.voltage; });
        
        if(showCurrent) plotData(UIColors::WARNING, -2.0f, 2.0f,
            [](const ChargingChannel::Measurements& m) { return m.current; });
            
        if(showTemp) plotData(UIColors::ERROR, 20.0f, 50.0f,
            [](const ChargingChannel::Measurements& m) { return m.temperature; });
    }
    
private:
    template<typename F>
    void plotData(uint16_t color, float minVal, float maxVal, F getValue) {
        float xScale = (width - 40) / timeScale;
        float yScale = (height - 40) / (maxVal - minVal);
        
        uint32_t now = millis();
        
        for(size_t i = 1; i < history.size(); i++) {
            float x1 = (history[i-1].timestamp - now) / 1000.0f + timeScale;
            float x2 = (history[i].timestamp - now) / 1000.0f + timeScale;
            
            if(x1 < 0 || x2 > timeScale) continue;
            
            float y1 = getValue(history[i-1]);
            float y2 = getValue(history[i]);
            
            display.drawLine(
                30 + x1 * xScale,
                y + height - 20 - (y1 - minVal) * yScale,
                30 + x2 * xScale,
                y + height - 20 - (y2 - minVal) * yScale,
                color
            );
        }
    }
};

// Screen base class
class Screen {
protected:
    TFT_eSPI& display;
    std::vector<ChargingChannel>& channels;
    std::vector<std::unique_ptr<UIElement>> elements;
    StatusBar statusBar;
    
public:
    Screen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) 
        : display(d), channels(ch), statusBar(d) {}
    
    virtual void draw() {
        display.fillScreen(UIColors::BACKGROUND);
        statusBar.draw();
        for(auto& element : elements) {
            if(element != nullptr) {
                element->draw();
            }
        }
    }
    
    virtual void handleButton(uint8_t button) = 0;
    virtual void update() = 0;
    virtual ~Screen() {}
};

// Main overview screen
class MainScreen : public Screen {
    std::vector<std::unique_ptr<BatteryDisplay>> batteries;
    size_t selectedChannel = 0;
    
public:
    MainScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : Screen(d, ch) {
        statusBar.setTitle("Battery Charger");
        
        // Create battery displays in 2x2 grid
        int x = 0, y = 25;
        for(size_t i = 0; i < channels.size(); i++) {
            auto battery = std::make_unique<BatteryDisplay>(display, channels[i], x, y);
            batteries.push_back(std::move(battery));
            
            x += 120;
            if(x >= 240) {
                x = 0;
                y += 85;
            }
        }
        updateSelection();
    }
    
    void handleButton(uint8_t button) override {
        switch(button) {
            case CMD_UP:
                if(selectedChannel >= 2) selectedChannel -= 2;
                break;
            case CMD_DOWN:
                if(selectedChannel + 2 < channels.size()) selectedChannel += 2;
                break;
            case CMD_LEFT:
                if(selectedChannel > 0) selectedChannel--;
                break;
case CMD_RIGHT:
                if(selectedChannel < channels.size() - 1) selectedChannel++;
                break;
            case CMD_SELECT:
                toggleCharging();
                break;
            case CMD_0:
                channels[selectedChannel].targetCurrent = 0; // Stop
                break;
            case CMD_1:
                channels[selectedChannel].targetCurrent = 0.5; // Low current
                break;
            case CMD_2:
                channels[selectedChannel].targetCurrent = 1.0; // Medium current
                break;
            case CMD_3:
                channels[selectedChannel].targetCurrent = 2.0; // High current
                break;
        }
        updateSelection();
    }
    
    void update() override {
        String status = String(selectedChannel + 1) + "/" + String(channels.size());
        statusBar.setStatus(status);
        draw();
    }
    
private:
    void updateSelection() {
        for(size_t i = 0; i < batteries.size(); i++) {
            batteries[i]->setSelected(i == selectedChannel);
        }
    }
    
    void toggleCharging() {
        auto& channel = channels[selectedChannel];
        switch(channel.state) {
            case ChargingChannel::IDLE:
                channel.state = ChargingChannel::CHARGING;
                channel.targetCurrent = 1.0; // Default to 1A
                break;
            case ChargingChannel::CHARGING:
                channel.state = ChargingChannel::IDLE;
                channel.targetCurrent = 0;
                break;
            case ChargingChannel::ERROR:
                if(channel.current.voltage < channel.params.maxVoltage && 
                   channel.current.temperature < 40.0f) {
                    channel.state = ChargingChannel::IDLE;
                    channel.errorMessage = "";
                }
                break;
        }
    }
};

// Detailed channel view screen
class DetailScreen : public Screen {
    size_t channelIndex;
    std::unique_ptr<Graph> graph;
    std::unique_ptr<BatteryDisplay> batteryDisplay;
    bool showMenu = false;
    
    enum MenuItem {
        MENU_CHARGE,
        MENU_DISCHARGE,
        MENU_TEST,
        MENU_SETTINGS,
        MENU_BACK,
        MENU_COUNT
    };
    MenuItem selectedItem = MENU_CHARGE;
    
public:
    DetailScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch, size_t index) 
        : Screen(d, ch), channelIndex(index) {
        
        statusBar.setTitle("Channel " + String(channelIndex + 1));
        
        // Create graph
        graph = std::make_unique<Graph>(display, channels[channelIndex].history);
        graph->setPosition(0, 25);
        
        // Create battery display
        batteryDisplay = std::make_unique<BatteryDisplay>(
            display, channels[channelIndex], 0, SCREEN_HEIGHT - 85);
    }
    
    void draw() override {
        Screen::draw();
        
        if(!showMenu) {
            graph->draw();
            batteryDisplay->draw();
            
            // Draw channel stats
            drawStats();
        } else {
            drawMenu();
        }
    }
    
    void handleButton(uint8_t button) override {
        if(showMenu) {
            handleMenuButton(button);
        } else {
            switch(button) {
                case CMD_MENU:
                    showMenu = true;
                    break;
                case CMD_BACK:
                    showMenu = false;
                    break;
                case CMD_1:
                    channels[channelIndex].targetCurrent = 0.5;
                    break;
                case CMD_2:
                    channels[channelIndex].targetCurrent = 1.0;
                    break;
                case CMD_3:
                    channels[channelIndex].targetCurrent = 2.0;
                    break;
                case CMD_0:
                    channels[channelIndex].targetCurrent = 0;
                    break;
            }
        }
    }
    
    void update() override {
        auto& channel = channels[channelIndex];
        String status = String(channel.current.voltage, 2) + "V " +
                       String(channel.current.current, 2) + "A";
        statusBar.setStatus(status);
        draw();
    }
    
private:
    void drawStats() {
        auto& channel = channels[channelIndex];
        int16_t x = 5, y = SCREEN_HEIGHT - 40;
        
        // Calculate charge/discharge capacity
        float capacity = 0;
        if(!channel.history.empty()) {
            uint32_t duration = (channel.current.timestamp - 
                               channel.history[0].timestamp) / 1000; // seconds
            capacity = (channel.current.current * duration) / 3600.0f; // Ah
        }
        
        display.setTextColor(UIColors::TEXT_PRIMARY);
        display.drawString("Capacity: " + String(capacity, 3) + "Ah", x, y);
        display.drawString("Time: " + formatDuration(channel.stateTime), x + 120, y);
        
        if(channel.errorMessage.length() > 0) {
            display.setTextColor(UIColors::ERROR);
            display.drawString(channel.errorMessage, x, y + 15);
        }
    }
    
    void drawMenu() {
        const char* menuItems[] = {
            "Start Charging",
            "Start Discharge",
            "Battery Test",
            "Settings",
            "Back"
        };
        
        int16_t y = 40;
        for(int i = 0; i < MENU_COUNT; i++) {
            uint16_t color = (i == selectedItem) ? UIColors::ACCENT : UIColors::TEXT_PRIMARY;
            display.setTextColor(color);
            display.drawString(menuItems[i], 20, y);
            y += 30;
        }
    }
    
    void handleMenuButton(uint8_t button) {
        switch(button) {
            case CMD_UP:
                if(selectedItem > 0) selectedItem = (MenuItem)(selectedItem - 1);
                break;
            case CMD_DOWN:
                if(selectedItem < MENU_COUNT - 1) selectedItem = (MenuItem)(selectedItem + 1);
                break;
            case CMD_SELECT:
                executeMenuItem();
                break;
            case CMD_BACK:
            case CMD_MENU:
                showMenu = false;
                break;
        }
    }
    
    void executeMenuItem() {
        auto& channel = channels[channelIndex];
        
        switch(selectedItem) {
            case MENU_CHARGE:
                channel.state = ChargingChannel::CHARGING;
                channel.targetCurrent = 1.0;
                showMenu = false;
                break;
                
            case MENU_DISCHARGE:
                channel.state = ChargingChannel::DISCHARGING;
                channel.targetCurrent = -0.5;
                showMenu = false;
                break;
                
            case MENU_TEST:
                startBatteryTest();
                showMenu = false;
                break;
                
            case MENU_SETTINGS:
                // TODO: Implement settings screen
                break;
                
            case MENU_BACK:
                showMenu = false;
                break;
        }
    }
    
    void startBatteryTest() {
        auto& channel = channels[channelIndex];
        channel.selfTestEnabled = true;
        channel.state = ChargingChannel::CHARGING;
        channel.targetCurrent = 1.0;
        // Test sequence will be handled in channel update
    }
    
    String formatDuration(uint32_t ms) {
        uint32_t seconds = ms / 1000;
        uint32_t minutes = seconds / 60;
        uint32_t hours = minutes / 60;
        
        return String(hours) + ":" + 
               (minutes % 60 < 10 ? "0" : "") + String(minutes % 60) + ":" +
               (seconds % 60 < 10 ? "0" : "") + String(seconds % 60);
    }
};

// Main application class
class BatteryChargerUI {
    TFT_eSPI& display;
    std::vector<ChargingChannel> channels;
    std::unique_ptr<Screen> currentScreen;
    uint32_t lastUpdate = 0;
    static const uint32_t UPDATE_INTERVAL = 100; // 10Hz refresh
    
public:
    BatteryChargerUI(TFT_eSPI& d, size_t numChannels = 4) 
        : display(d), channels(numChannels) {
        display.init();
        display.setRotation(0);
        display.fillScreen(UIColors::BACKGROUND);
        
        showMainScreen();
    }
    
    void handleButton(uint8_t button) {
        if(currentScreen) {
            currentScreen->handleButton(button);
        }
    }
    
    void update() {
        uint32_t now = millis();
        if(now - lastUpdate >= UPDATE_INTERVAL) {
            updateMeasurements();
            if(currentScreen) {
                currentScreen->update();
            }
            lastUpdate = now;
        }
    }
    
private:
    void showMainScreen() {
        currentScreen = std::make_unique<MainScreen>(display, channels);
    }
    
    void showDetailScreen(size_t channel) {
        currentScreen = std::make_unique<DetailScreen>(display, channels, channel);
    }
    
    void updateMeasurements() {
        // Here you would add your actual ADC reading code
        // For now, we'll simulate some values
        for(auto& channel : channels) {
            // Simulate measurements based on state and target current
            float v = channel.current.voltage;
            float i = channel.current.current;
            float t = channel.current.temperature;
            
            if(channel.state == ChargingChannel::CHARGING) {
                // Simulate charging behavior
                if(i < channel.targetCurrent) {
                    i += channel.rampRate * (UPDATE_INTERVAL / 1000.0f);
                    if(i > channel.targetCurrent) i = channel.targetCurrent;
                }
                v += (i * 0.05f * (UPDATE_INTERVAL / 1000.0f));
                t += (i * i * 0.01f * (UPDATE_INTERVAL / 1000.0f));
            } else if(channel.state == ChargingChannel::DISCHARGING) {
                // Simulate discharging
                if(i > channel.targetCurrent) {
                    i -= channel.rampRate * (UPDATE_INTERVAL / 1000.0f);
                    if(i < channel.targetCurrent) i = channel.targetCurrent;
                }
                v += (i * 0.1f * (UPDATE_INTERVAL / 1000.0f));
                t += (i * i * 0.005f * (UPDATE_INTERVAL / 1000.0f));
            }
            
            // Update channel with new measurements
            channel.update(v, i, t);
        }
    }
};

void setup() {
    Serial.begin(115200);
    
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);
    
    // Create UI instance
    BatteryChargerUI ui(tft);
    
    while(true) {
        if(IrReceiver.decode()) {
            uint8_t command = IrReceiver.decodedIRData.command;
            ui.handleButton(command);
            IrReceiver.resume();
        }
        
        ui.update();
        yield(); // Allow ESP32 background tasks
    }
}

void loop() {
    // Empty - using while(true) in setup()
}
