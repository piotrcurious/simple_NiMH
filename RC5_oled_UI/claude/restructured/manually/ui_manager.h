// ui_manager.h
#pragma once

class UIManager {
private:
    Adafruit_SSD1306 display;
    IRrecv& irReceiver;
    ChannelManager& channelManager;
    std::unique_ptr<Screen> screens[5];
    Screen* currentScreen;
    uint32_t lastUpdate;
    
public:
    UIManager(IRrecv& ir, ChannelManager& cm) 
        : irReceiver(ir), channelManager(cm), lastUpdate(0) {
        display = Adafruit_SSD1306(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, 
                                 &Wire, Config::OLED_RESET);
    }
    
    void begin() {
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        display.clearDisplay();
        display.setTextColor(WHITE);
        
        // Initialize screens
        screens[static_cast<int>(ScreenID::MAIN_STATUS)] = 
            std::make_unique<MainStatusScreen>(display, channelManager);
        // ... initialize other screens
        
        currentScreen = screens[0].get();
    }
    
    void update() {
        uint32_t now = millis();
        
        // Handle IR input
        if (irReceiver.decode()) {
            handleInput(static_cast<IRCommand>(irReceiver.decodedIRData.command));
            irReceiver.resume();
        }
        
        // Update current screen
        currentScreen->update();
        currentScreen->draw();
        
        lastUpdate = now;
    }
    
    void handleInput(IRCommand cmd) {
        // Implementation...
    }
};
