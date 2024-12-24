#include <TFT_eSPI.h>
#include <IRremote.h>
#include <vector>
#include <functional>

// Display and IR setup remains the same as before...

// Layout system
struct Rect {
    int x, y, w, h;
    Rect(int x_ = 0, int y_ = 0, int w_ = 0, int h_ = 0) : x(x_), y(y_), w(w_), h(h_) {}
};

class UIElement {
protected:
    Rect bounds;
    bool visible = true;
    TFT_eSPI& display;
    
public:
    UIElement(TFT_eSPI& d) : display(d) {}
    virtual void draw() = 0;
    virtual void setPosition(const Rect& r) { bounds = r; }
    virtual Rect getBounds() const { return bounds; }
    virtual void setVisible(bool v) { visible = v; }
    bool isVisible() const { return visible; }
    virtual ~UIElement() {}
};

// Layout managers
class LayoutManager {
protected:
    std::vector<UIElement*> elements;
    Rect containerBounds;
    int padding = 4;
    
public:
    void setPadding(int p) { padding = p; }
    void setContainerBounds(const Rect& bounds) { containerBounds = bounds; }
    void addElement(UIElement* element) { elements.push_back(element); }
    virtual void layout() = 0;
    virtual ~LayoutManager() {}
};

class VerticalLayout : public LayoutManager {
public:
    void layout() override {
        if(elements.empty()) return;
        
        int totalHeight = containerBounds.h - (padding * (elements.size() + 1));
        int elementHeight = totalHeight / elements.size();
        
        for(size_t i = 0; i < elements.size(); i++) {
            int y = containerBounds.y + padding + i * (elementHeight + padding);
            elements[i]->setPosition(Rect(
                containerBounds.x + padding,
                y,
                containerBounds.w - 2 * padding,
                elementHeight
            ));
        }
    }
};

class GridLayout : public LayoutManager {
    int rows, cols;
public:
    GridLayout(int r, int c) : rows(r), cols(c) {}
    
    void layout() override {
        if(elements.empty()) return;
        
        int elementWidth = (containerBounds.w - (padding * (cols + 1))) / cols;
        int elementHeight = (containerBounds.h - (padding * (rows + 1))) / rows;
        
        for(size_t i = 0; i < elements.size(); i++) {
            int row = i / cols;
            int col = i % cols;
            
            elements[i]->setPosition(Rect(
                containerBounds.x + padding + col * (elementWidth + padding),
                containerBounds.y + padding + row * (elementHeight + padding),
                elementWidth,
                elementHeight
            ));
        }
    }
};

// UI Elements
class Label : public UIElement {
    String text;
    uint16_t color;
    
public:
    Label(TFT_eSPI& d, const String& t, uint16_t c = TFT_WHITE) 
        : UIElement(d), text(t), color(c) {}
    
    void draw() override {
        if(!visible) return;
        display.setTextColor(color);
        display.drawString(text, bounds.x, bounds.y);
    }
    
    void setText(const String& t) { text = t; }
};

class Graph : public UIElement {
    std::vector<float>& data;
    uint16_t color;
    float minValue, maxValue;
    
public:
    Graph(TFT_eSPI& d, std::vector<float>& data_, uint16_t c = TFT_GREEN)
        : UIElement(d), data(data_), color(c), minValue(0), maxValue(1.5) {}
    
    void draw() override {
        if(!visible || data.empty()) return;
        
        float xScale = bounds.w / float(std::min(size_t(100), data.size()));
        float yScale = bounds.h / (maxValue - minValue);
        
        for(size_t i = 1; i < data.size() && i < 100; i++) {
            display.drawLine(
                bounds.x + (i-1) * xScale,
                bounds.y + bounds.h - (data[data.size()-i-1] - minValue) * yScale,
                bounds.x + i * xScale,
                bounds.y + bounds.h - (data[data.size()-i] - minValue) * yScale,
                color
            );
        }
    }
    
    void setRange(float min, float max) {
        minValue = min;
        maxValue = max;
    }
};

class BatteryIndicator : public UIElement {
    float& voltage;
    float maxVoltage;
    
public:
    BatteryIndicator(TFT_eSPI& d, float& v, float max = 1.5) 
        : UIElement(d), voltage(v), maxVoltage(max) {}
    
    void draw() override {
        if(!visible) return;
        
        // Battery outline
        display.drawRect(bounds.x, bounds.y, bounds.w, bounds.h, TFT_WHITE);
        display.fillRect(bounds.x + bounds.w, bounds.y + bounds.h/4, 
                        bounds.w/10, bounds.h/2, TFT_WHITE);
        
        // Fill level
        int fillWidth = (voltage / maxVoltage) * (bounds.w - 2);
        display.fillRect(bounds.x + 1, bounds.y + 1, 
                        fillWidth, bounds.h - 2, TFT_GREEN);
    }
};

// Enhanced Screen base class
class Screen {
protected:
    TFT_eSPI& display;
    std::vector<ChargingChannel>& channels;
    std::vector<std::unique_ptr<UIElement>> elements;
    std::unique_ptr<LayoutManager> layout;
    
public:
    Screen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) 
        : display(d), channels(ch) {}
    
    virtual void createLayout() = 0;
    virtual void handleButton(uint8_t button) = 0;
    
    void draw() {
        display.fillScreen(TFT_BLACK);
        for(auto& element : elements) {
            if(element->isVisible()) {
                element->draw();
            }
        }
    }
    
    virtual ~Screen() {}
};

// Main channel status screen with new layout
class MainScreen : public Screen {
    std::vector<Label*> voltageLabels;
    std::vector<Label*> currentLabels;
    std::vector<BatteryIndicator*> batteryIndicators;
    
public:
    MainScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : Screen(d, ch) {
        createLayout();
    }
    
    void createLayout() override {
        layout = std::make_unique<VerticalLayout>();
        layout->setContainerBounds(Rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT));
        
        for(size_t i = 0; i < channels.size(); i++) {
            // Create channel container with grid layout
            auto* channelGrid = new GridLayout(2, 3);
            channelGrid->setContainerBounds(Rect(0, i * (SCREEN_HEIGHT/channels.size()), 
                                               SCREEN_WIDTH, SCREEN_HEIGHT/channels.size()));
            
            // Create channel elements
            auto* channelLabel = new Label(display, "CH" + String(i+1));
            auto* voltLabel = new Label(display, "0.00V");
            auto* currLabel = new Label(display, "0.00A");
            auto* battery = new BatteryIndicator(display, channels[i].currentVoltage);
            
            // Store pointers for updating
            voltageLabels.push_back(voltLabel);
            currentLabels.push_back(currLabel);
            batteryIndicators.push_back(battery);
            
            // Add elements to grid
            channelGrid->addElement(channelLabel);
            channelGrid->addElement(voltLabel);
            channelGrid->addElement(currLabel);
            channelGrid->addElement(battery);
            
            // Layout channel grid
            channelGrid->layout();
            
            // Store elements
            elements.push_back(std::unique_ptr<UIElement>(channelLabel));
            elements.push_back(std::unique_ptr<UIElement>(voltLabel));
            elements.push_back(std::unique_ptr<UIElement>(currLabel));
            elements.push_back(std::unique_ptr<UIElement>(battery));
        }
    }
    
    void update() {
        for(size_t i = 0; i < channels.size(); i++) {
            voltageLabels[i]->setText(String(channels[i].currentVoltage, 2) + "V");
            currentLabels[i]->setText(String(channels[i].currentCurrent, 2) + "A");
        }
    }
    
    void handleButton(uint8_t button) override {
        // Handle channel selection
    }
};

// Graph screen with new layout
class GraphScreen : public Screen {
    size_t selectedChannel = 0;
    std::unique_ptr<Graph> voltageGraph;
    std::unique_ptr<Graph> currentGraph;
    
public:
    GraphScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : Screen(d, ch) {
        createLayout();
    }
    
    void createLayout() override {
        layout = std::make_unique<VerticalLayout>();
        layout->setContainerBounds(Rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT));
        
        // Create graphs
        voltageGraph = std::make_unique<Graph>(display, 
            channels[selectedChannel].voltageHistory, TFT_YELLOW);
        currentGraph = std::make_unique<Graph>(display, 
            channels[selectedChannel].currentHistory, TFT_GREEN);
        
        voltageGraph->setRange(0, 1.5);
        currentGraph->setRange(-1, 2);
        
        // Create layout areas
        auto* topArea = new VerticalLayout();
        topArea->setContainerBounds(Rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT/2));
        topArea->addElement(voltageGraph.get());
        
        auto* bottomArea = new VerticalLayout();
        bottomArea->setContainerBounds(Rect(0, SCREEN_HEIGHT/2, 
                                          SCREEN_WIDTH, SCREEN_HEIGHT/2));
        bottomArea->addElement(currentGraph.get());
        
        // Layout graphs
        topArea->layout();
        bottomArea->layout();
    }
    
    void handleButton(uint8_t button) override {
        if(button == IR_BUTTON_CH_UP) {
            selectedChannel = (selectedChannel + 1) % channels.size();
            createLayout();  // Rebuild layout with new channel data
        }
    }
};

// Main application class remains similar but uses new screen implementations...

void setup() {
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);
    
    // Create UI instance
    BatteryChargerUI ui(tft);
    
    // Main loop remains the same...
}

void loop() {
    // Empty - using while(true) in setup()
}
