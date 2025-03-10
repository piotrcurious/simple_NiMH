#include <TFT_eSPI.h>
#include <IRremote.h>
#include <vector>

#define UI_UPDATE_DELAY 500 // 500ms
#define MEASUREMENTS_UPDATE_DELAY 1000 // 1 second

// Display setup
TFT_eSPI tft = TFT_eSPI();
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

#define DESTINATION_SCANLINE 120 // scanline at which screen redraw is initiated

// IR setup
#define IR_RECEIVE_PIN      15
#define IR_BUTTON_CH_UP     0x60
#define IR_BUTTON_CH_DOWN   0x61
#define IR_BUTTON_VOL_UP    0x7
#define IR_BUTTON_VOL_DOWN  0xB
#define IR_BUTTON_OFF       0x2
#define IR_BUTTON_ENTER     0x68
#define IR_BUTTON_BACK      0x58

// Battery charging parameters
struct ChargingChannel {
    float currentVoltage = 0;
    float currentCurrent = 0;
    float targetCurrent = 0;
    float rampRate = 0.1;      // A/s
    bool isCharging = false;
    std::vector<float> voltageHistory;
    std::vector<float> currentHistory;
    uint32_t chargeStartTime = 0;
    bool selfDischargeTest = false;
};

// Screen base class
class Screen {
protected:
    TFT_eSPI& display;
    std::vector<ChargingChannel>& channels;
public:
    Screen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : display(d), channels(ch) {}
    virtual void draw() = 0;
    virtual void handleButton(uint8_t button) = 0;
    virtual ~Screen() {}
};

// Main channel status screen
class MainScreen : public Screen {
public:
    MainScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : Screen(d, ch) {}
    
    void draw() override {
        //display.fillScreen(TFT_BLACK); // to avoid flickering
        
        for(size_t i = 0; i < channels.size(); i++) {
            int yPos = i * 80;
            // Channel header
            if (i==selectedChannel){
            display.fillRect(0, yPos, SCREEN_WIDTH, SCREEN_HEIGHT/channels.size(), TFT_DARKGREEN); // clear background with selection color
            } else {
            display.fillRect(0, yPos, SCREEN_WIDTH, SCREEN_HEIGHT/channels.size(), TFT_BLACK);     // clear background with black
            display.drawRect(0, yPos, SCREEN_WIDTH, SCREEN_HEIGHT/channels.size(), TFT_WHITE);
              
            }

            display.setTextSize(2);
            display.setTextColor(TFT_WHITE);
            display.drawString("CH" + String(i+1), 5, yPos + 5);
            
            // Battery symbol
            drawBatterySymbol(180, yPos + 10, channels[i].currentVoltage);
            
            // Status values
            display.drawString(String(channels[i].currentVoltage, 2) + "V", 10, yPos + 25);
            display.drawString(String(channels[i].currentCurrent, 2) + "A", 10, yPos + 45);
            
            // Status indicator
            if(channels[i].isCharging) {
                display.fillCircle(220, yPos + 40, 5, TFT_GREEN);
            } else {
                display.drawCircle(220, yPos + 40, 5, TFT_RED);
            }
        }
    }
    
    void handleButton(uint8_t button) override {
        // Handle channel selection and basic controls
        if(button == IR_BUTTON_CH_DOWN) selectedChannel = (selectedChannel + 1) % channels.size();
        if(button == IR_BUTTON_CH_UP) selectedChannel = (selectedChannel > 0) ? selectedChannel - 1 : channels.size() - 1;     

//        if(button == IR_BUTTON_ENTER) channels[selectedChannel].isCharging = (channels[selectedChannel].isCharging +1 ) % 1;
        if(button == IR_BUTTON_ENTER) channels[selectedChannel].isCharging = true;
        if(button == IR_BUTTON_OFF)   channels[selectedChannel].isCharging = false;
         
    }
    
private:
    size_t selectedChannel = 0;

    void drawBatterySymbol(int x, int y, float voltage) {
        // Draw battery outline
        display.drawRect(x, y, 30, 15, TFT_WHITE);
        display.fillRect(x + 30, y + 4, 3, 7, TFT_WHITE);
        
        // Fill based on voltage (assuming 1.2V NiMH)
        int fillWidth = (voltage / 1.5) * 28;
        display.fillRect(x + 1, y + 1, fillWidth, 13, TFT_GREEN);
    }
};

// Graph screen for charge process
class GraphScreen : public Screen {
public:
    GraphScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : Screen(d, ch) {}
    
    void draw() override {
        display.fillScreen(TFT_BLACK);

        // draw channel header
        display.setTextSize(2);
        display.setTextColor(TFT_WHITE);
        display.drawString("CH" + String(selectedChannel+1), SCREEN_WIDTH-36,  5);
        
        // Draw axes
        display.drawLine(30, 30, 30, SCREEN_HEIGHT-30, TFT_WHITE);  // Y axis
        display.drawLine(30, SCREEN_HEIGHT-30, SCREEN_WIDTH-30, SCREEN_HEIGHT-30, TFT_WHITE);  // X axis
        
        // Plot voltage history
        if(!channels[selectedChannel].voltageHistory.empty()) {
            int numPoints = channels[selectedChannel].voltageHistory.size();
            float xScale = (SCREEN_WIDTH - 60) / float(numPoints);
            float yScale = (SCREEN_HEIGHT - 60) / 1.5f;  // Assume max 1.5V
            
            for(int i = 1; i < numPoints; i++) {
                display.drawLine(
                    30 + (i-1) * xScale,
                    SCREEN_HEIGHT - 30 - channels[selectedChannel].voltageHistory[i-1] * yScale,
                    30 + i * xScale,
                    SCREEN_HEIGHT - 30 - channels[selectedChannel].voltageHistory[i] * yScale,
                    TFT_YELLOW
                );
            }
        }
    }
    
    void handleButton(uint8_t button) override {
        if(button == IR_BUTTON_CH_UP) selectedChannel = (selectedChannel + 1) % channels.size();
        if(button == IR_BUTTON_CH_DOWN) selectedChannel = (selectedChannel > 0) ? selectedChannel - 1 : channels.size() - 1;
    }
    
private:
    size_t selectedChannel = 0;
};

// Manual maintenance screen
class MaintenanceScreen : public Screen {
public:
    MaintenanceScreen(TFT_eSPI& d, std::vector<ChargingChannel>& ch) : Screen(d, ch) {}
    
    void draw() override {
        display.fillScreen(TFT_BLACK);
        
        // channel info
        display.setTextSize(2);
        display.setTextColor(TFT_WHITE);
        display.drawString("CH" + String(selectedChannel+1), SCREEN_WIDTH-36,  16);

        // Instructions
        display.setTextSize(1);
        display.setTextColor(TFT_WHITE);
        display.drawString("VOL+: Charge pulse", 1, 8*1);
        display.drawString("VOL-: Discharge", 1, 8*2);
        
        // Real-time voltage graph (rolling)
        drawVoltageGraph();
        
        // Current status
        display.drawString("V: " + String(channels[selectedChannel].currentVoltage, 3), 10, SCREEN_HEIGHT - 40);
        display.drawString("I: " + String(channels[selectedChannel].currentCurrent, 3), 120, SCREEN_HEIGHT - 40);
    }
    
    void handleButton(uint8_t button) override {
        switch(button) {
            case IR_BUTTON_VOL_UP:
                // Send charge pulse
                channels[selectedChannel].targetCurrent = 0.5;  // 500mA pulse
                break;
            case IR_BUTTON_VOL_DOWN:
                // Enable discharge
                channels[selectedChannel].targetCurrent = -0.2; // 200mA discharge
                break;
            case IR_BUTTON_CH_UP:
                selectedChannel = (selectedChannel + 1) % channels.size();
                break;
            case IR_BUTTON_CH_DOWN:
                selectedChannel = (selectedChannel > 0) ? selectedChannel - 1 : channels.size() - 1;
                break;
        }
    }
    
private:
    size_t selectedChannel = 0;
    
    void drawVoltageGraph() {
        // Implementation of rolling voltage graph
        const auto& history = channels[selectedChannel].voltageHistory;
        if(history.empty()) return;
        
        float xScale = (SCREEN_WIDTH - 20) / 100.0f;  // Show last 100 samples
        float yScale = 100.0f;  // Scale voltage to pixels
        
        for(size_t i = 1; i < history.size() && i < 100; i++) {
            display.drawLine(
                SCREEN_WIDTH - (i-1) * xScale,
                100 + history[history.size()-i-1] * yScale,
                SCREEN_WIDTH - i * xScale,
                100 + history[history.size()-i] * yScale,
                TFT_GREEN
            );
        }
    }
};

// Main application class
class BatteryChargerUI {
private:
    TFT_eSPI& display;
    std::vector<ChargingChannel> channels;
    std::vector<Screen*> screens;
    size_t currentScreen = 0;
    
public:
    BatteryChargerUI(TFT_eSPI& d, size_t numChannels = 4) : display(d) {
        channels.resize(numChannels);
        
        // Initialize screens
        screens.push_back(new MainScreen(display, channels));
        screens.push_back(new GraphScreen(display, channels));
        screens.push_back(new MaintenanceScreen(display, channels));
        
        // Initialize display
        display.init();
        display.setRotation(0);
        display.fillScreen(TFT_BLACK);

//        tft.writecommand(0xb0);
//        tft.writedata(0x00);
//        tft.writedata(0xc3);
        
    }
    
    void handleButton(uint8_t button) {
        if(button == IR_BUTTON_BACK) {
            currentScreen = (currentScreen + 1) % screens.size();
        } else {
            screens[currentScreen]->handleButton(button);
        }
        update();
    }
    
    void update() {
        screens[currentScreen]->draw();
    }


  
    void update_measurements(){
        updateMeasurements();
      
    }
    ~BatteryChargerUI() {
        for(auto screen : screens) {
            delete screen;
        }
    }
    
private:
    void updateMeasurements() {
        // Update voltage and current measurements for each channel
        for(auto& channel : channels) {
            // Add your ADC reading code here
            
            // Store history
            channel.voltageHistory.push_back(channel.currentVoltage);
            channel.currentHistory.push_back(channel.currentCurrent);
            
            // Limit history size
            if(channel.voltageHistory.size() > 1000) {
                channel.voltageHistory.erase(channel.voltageHistory.begin());
                channel.currentHistory.erase(channel.currentHistory.begin());
            }
        }
    }
};

// Main program
void setup() {
    Serial.begin(115200); // for debug
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);
    
}

// Create UI instance
    BatteryChargerUI ui(tft);  

void handle_IR (){
  if(IrReceiver.decode()) {
          if (IrReceiver.decodedIRData.protocol == SAMSUNG) {

            if (IrReceiver.decodedIRData.address == 0x7) {
            uint8_t command = IrReceiver.decodedIRData.command;
            Serial.print(F("Command 0x"));
            Serial.println(IrReceiver.decodedIRData.command, HEX);
            ui.handleButton(command);
            }
          }
            IrReceiver.resume();
        }
  
}

uint32_t last_ui_update = 0; 
uint32_t last_measurements_update = 0 ;

void loop() {
        handle_IR();              
  
        if (millis() - last_ui_update > UI_UPDATE_DELAY) {
        last_ui_update = millis();
        ui.update();      
       
        }

        if (millis() - last_measurements_update > MEASUREMENTS_UPDATE_DELAY) {
        last_measurements_update = millis();
        ui.update_measurements();      
        }
        
        yield();
        
}
