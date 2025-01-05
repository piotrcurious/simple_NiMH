#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

// Configuration and Constants
namespace Config {
  // Display
  constexpr uint8_t SCREEN_WIDTH = 128;
  constexpr uint8_t SCREEN_HEIGHT = 64;
  //constexpr int8_t OLED_RESET = -1;
  constexpr uint8_t SCREEN_ADDRESS = 0x3C;
  constexpr uint8_t OLED_DC = 16 ;   
//#define OLED_DC     16
  constexpr uint8_t OLED_CS = 5 ; 
//#define OLED_CS     5
  constexpr uint8_t OLED_RESET = 17;
//#define OLED_RESET  17
  constexpr uint32_t OLED_BITRATE = 80000000 ;
//#define OLED_BITRATE  80000000 // OLED SPI bitrate 80Mhz max
  
  // Hardware
  constexpr uint8_t IR_PIN = 22;
  constexpr uint8_t NUM_CHANNELS = 4;
  
  // UI Layout
  namespace Layout {
    constexpr uint8_t HEADER_HEIGHT = 10;
    constexpr uint8_t MARGIN = 2;
    constexpr uint8_t CHANNEL_ROW_HEIGHT = 12;
    constexpr uint8_t GRAPH_HEIGHT = 32;
    constexpr uint8_t STATUS_ROW_HEIGHT = 8;
    constexpr uint8_t ICON_SIZE = 8;
    
    // Areas definitions for different screens
    struct ScreenArea {
      uint8_t x, y, width, height;
    };
    
    const ScreenArea HEADER = {0, 0, SCREEN_WIDTH, HEADER_HEIGHT};
    const ScreenArea MAIN_CONTENT = {0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT};
    const ScreenArea GRAPH_AREA = {MARGIN, HEADER_HEIGHT + MARGIN, 
                                 SCREEN_WIDTH - 2*MARGIN, GRAPH_HEIGHT};
    const ScreenArea STATUS_BAR = {0, SCREEN_HEIGHT - STATUS_ROW_HEIGHT, 
                                 SCREEN_WIDTH, STATUS_ROW_HEIGHT};
  }
  
  // Battery Parameters
  constexpr uint16_t MIN_CHARGE_CURRENT = 100;   // mA
  constexpr uint16_t MAX_CHARGE_CURRENT = 2000;  // mA
  constexpr uint16_t CHARGE_CURRENT_STEP = 100;  // mA
  constexpr float MIN_CELL_VOLTAGE = 0.9;        // V
  constexpr float MAX_CELL_VOLTAGE = 1.6;        // V
  constexpr float TEMP_THRESHOLD = 45.0;         // °C
}

// IR Remote Key Definitions
namespace RemoteKeys {
  enum KeyCode {
    KEY_0 = 0x11,
    KEY_1 = 0x04,
    KEY_2 = 0x05,
    KEY_3 = 0x06,
    KEY_4 = 0x08,
    KEY_5 = 0x09,
    KEY_6 = 0x0A,
    KEY_7 = 0x0C,
    KEY_8 = 0x0D,
    KEY_9 = 0x0E,
    KEY_UP = 0x60,
    KEY_DOWN = 0x61,
    KEY_LEFT = 0x65,
    KEY_RIGHT = 0x62,
    KEY_OK = 0x68,
    KEY_MENU = 0x79,
    KEY_RED = 0x6c,
    KEY_GREEN = 0x14,
    KEY_YELLOW = 0x15,
    KEY_BLUE = 0x16,
    KEY_VOL_UP = 0x07,
    KEY_VOL_DOWN = 0x0b,
    KEY_CH_UP = 0x12,
    KEY_CH_DOWN = 0x10
  };
}

// UI Elements and Icons
namespace UI {

  // Custom Battery Symbol (8x16 pixels)
const unsigned char BATTERY_ICON16 [] PROGMEM = {
  0x3C, 0x3C, 0x42, 0x42, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x42, 0x42, 0x3C, 0x3C
};

  // 8x8 Battery Icon
  const uint8_t BATTERY_ICON[] PROGMEM = {
    0x3C, 0x42, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3C
  };
  
  // 8x8 Temperature Icon
  const uint8_t TEMP_ICON[] PROGMEM = {
    0x04, 0x0A, 0x0A, 0x0A, 0x0E, 0x1F, 0x1F, 0x0E
  };
  
  // 8x8 Current Icon
  const uint8_t CURRENT_ICON[] PROGMEM = {
    0x08, 0x0C, 0x0E, 0x1F, 0x1F, 0x0E, 0x0C, 0x08
  };
}

// Screen Management
enum class Screen {
  MAIN_MENU,
  CHANNEL_STATUS,
  CHARGE_SETTINGS,
  GRAPH_VIEW,
  MAINTENANCE,
  DIAGNOSTICS
};

// Charge Profiles
enum class ChargeProfile {
  NORMAL,
  GENTLE,
  AGGRESSIVE,
  RECOVERY
};

// Data Structures
struct BatteryMetrics {
  float voltage;
  float current;
  float temperatureC;
  float internalResistance;
  float capacityMah;
  uint32_t cycleCount;
};

struct ChargeSettings {
  float targetCurrent;
  ChargeProfile profile;
  float terminationVoltage;
  float minVoltage;
  bool enableBalancing;
  uint16_t pulseWidth;
};

class BatteryChannel {
public:
  BatteryMetrics metrics;
  ChargeSettings settings;
  bool isCharging;
  uint32_t chargeStartTime;
  float voltageHistory[Config::SCREEN_WIDTH];
  uint8_t historyIndex;
  
  BatteryChannel() {
    reset();
  }

 
  void reset() {
    metrics = {0};
    settings = {
      .targetCurrent = 500,
      .profile = ChargeProfile::NORMAL,
      .terminationVoltage = 1.45,
      .minVoltage = 1.0,
      .enableBalancing = true,
      .pulseWidth = 100
    };
    isCharging = false;
    chargeStartTime = 0;
    historyIndex = 0;
    memset(voltageHistory, 0, sizeof(voltageHistory));
  }
  
  void updateHistory(float voltage) {
    voltageHistory[historyIndex] = voltage;
    historyIndex = (historyIndex + 1) % Config::SCREEN_WIDTH;
  }
};

// Main Application Class
class BatteryChargerUI {
private:

//#define OLED_DC     16
//#define OLED_CS     5
//#define OLED_RESET  17
//#define OLED_BITRATE  80000000 // OLED SPI bitrate 80Mhz max

//Adafruit_SSD1306 display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT,
//  &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE);

  Adafruit_SSD1306 display;

  //IRrecv irrecv;
  //decode_results results;

  Screen currentScreen;
  uint8_t selectedChannel;
  BatteryChannel channels[Config::NUM_CHANNELS];
  uint32_t lastUpdate;
  bool maintenanceMode;

public:
  BatteryChargerUI() : 
//    display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, Config::OLED_RESET),
    display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &SPI, Config::OLED_DC, Config::OLED_RESET, Config::OLED_CS, Config::OLED_BITRATE),
  //  irrecv(Config::IR_PIN),

    currentScreen(Screen::MAIN_MENU),
    selectedChannel(0),
    lastUpdate(0),
    maintenanceMode(false) {}

void updateMeasurements() {
  if(millis() - lastUpdate > 1000) {
    for(int i = 0; i < 4; i++) {
      // Simulate reading measurements
      // Replace with actual ADC readings
      if(channels[i].isCharging) {
        channels[i].metrics.voltage += random(-100, 101) / 100.0;
        channels[i].metrics.current = channels[i].settings.targetCurrent + random(-500, 501);
        channels[i].metrics.temperatureC += random(-10, 20);
        
        // Update graph data
        for(int j = 0; j < Config::SCREEN_WIDTH - 1; j++) {
          channels[i].voltageHistory[j] = channels[i].voltageHistory[j + 1];
        }
        channels[i].voltageHistory[Config::SCREEN_WIDTH - 1] = channels[i].metrics.voltage * 100;
      }
    }
    lastUpdate = millis();
  }
}

void sendChargePulse(int channel) {
  // Implement actual hardware control
  channels[channel].settings.targetCurrent = 500;  // 500mA pulse
  delay(100);
  channels[channel].settings.targetCurrent = 0;
}

void sendDischargePulse(int channel) {
  // Implement actual hardware control
  channels[channel].settings.targetCurrent = -200;  // -200mA pulse
  delay(100);
  channels[channel].settings.targetCurrent = 0;
}

void drawMaintenance() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Maintenance Ch"));
  display.println(selectedChannel + 1);
  
  display.print(F("V: "));
  display.print(channels[selectedChannel].metrics.voltage, 2);
  display.println(F("V"));
  
  // Rolling voltage graph
  drawMiniVoltageGraph(0, 16, Config::SCREEN_WIDTH, 48);
  
  // Control hints
  display.setCursor(0, 56);
  display.print(F("G:Chg Y:Dis B:Exit"));
}

void drawMiniVoltageGraph(int x, int y, int w, int h) {
  // Draw mini graph frame
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  
  // Plot data points
  for(int i = 0; i < w - 2; i++) {
    if(i < Config::SCREEN_WIDTH) {
      int dataY = map(channels[selectedChannel].voltageHistory[i], 0, 2000, y + h - 2, y + 1);
      display.drawPixel(x + i + 1, dataY, SSD1306_WHITE);
    }
  }
}

void drawChannelStatus() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Channel "));
  display.println(selectedChannel + 1);
  
  display.print(F("V: "));
  display.print(channels[selectedChannel].metrics.voltage, 2);
  display.println(F("V"));
  
  display.print(F("I: "));
  display.print(channels[selectedChannel].metrics.current, 0);
  display.println(F("mA"));
  
  display.print(F("T: "));
  display.print(channels[selectedChannel].metrics.temperatureC, 1);
  display.println(F("C"));
  
  // Mini graph on right side
  drawMiniVoltageGraph(64, 0, 64, 32);
}

void drawMainMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("NiMH Charger"));
  
  for(int i = 0; i < 4; i++) {
    display.drawBitmap(0, 16 + i * 12, UI::BATTERY_ICON16, 8, 16, SSD1306_WHITE);
    display.setCursor(12, 16 + i * 12);
    display.print(F("Ch"));
    display.print(i + 1);
    display.print(F(": "));
    display.print(channels[i].metrics.voltage, 1);
    display.print(F("V "));
    if(channels[i].isCharging) {
      display.print(char(0x18));  // Up arrow
    }
  }
}

void drawGraphView() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Ch"));
  display.print(selectedChannel + 1);
  display.print(F(" Graph"));
  
  // Draw axis
  display.drawLine(10, 63, 10, 10, SSD1306_WHITE);
  display.drawLine(10, 63, 127, 63, SSD1306_WHITE);
  
  // Plot voltage history
  for(int i = 0; i < Config::SCREEN_WIDTH - 11; i++) {
    int y = map(channels[selectedChannel].voltageHistory[i], 0, 2000, 63, 10);
    display.drawPixel(i + 11, y, SSD1306_WHITE);
  }
}

void adjustCurrentSetting(int channel, int delta) {
  channels[channel].settings.targetCurrent += delta;
  if(channels[channel].settings.targetCurrent < 0) channels[channel].settings.targetCurrent = 0;
  if(channels[channel].settings.targetCurrent > 2000) channels[channel].settings.targetCurrent = 2000;
}


    
  void begin() {
    // Initialize display
    if(!display.begin(SSD1306_SWITCHCAPVCC, Config::SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Initialize IR receiver
    IrReceiver.begin(Config::IR_PIN, ENABLE_LED_FEEDBACK);

//    irrecv.enableIRIn();
    
    // Initialize channels
    for(int i = 0; i < Config::NUM_CHANNELS; i++) {
      channels[i].reset();
    }
  }
  
  void update() {
    handleIRInput();
    updateMeasurements();
    drawScreen();
  }

private:


  void handleIRInput() {
    if (IrReceiver.decode()) {
         if (IrReceiver.decodedIRData.protocol == SAMSUNG) {
            if (IrReceiver.decodedIRData.address == 0x7) {
      //debug        
            Serial.print(F("Command 0x"));
            Serial.println(IrReceiver.decodedIRData.command, HEX);

      switch(IrReceiver.decodedIRData.command) {
        case RemoteKeys::KEY_MENU:
          currentScreen = Screen::MAIN_MENU;
          break;
        case RemoteKeys::KEY_1:{
        uint8_t channel = 0;
          if(channel < Config::NUM_CHANNELS) {
            selectedChannel = channel;
            if(currentScreen == Screen::MAIN_MENU) {
              currentScreen = Screen::CHANNEL_STATUS;
            }
          }
          break;
          }
                 
        case RemoteKeys::KEY_2:{
        uint8_t channel = 1;
          if(channel < Config::NUM_CHANNELS) {
            selectedChannel = channel;
            if(currentScreen == Screen::MAIN_MENU) {
              currentScreen = Screen::CHANNEL_STATUS;
            }
          }
          break;
          }

        case RemoteKeys::KEY_3:{
        uint8_t channel = 2;
          if(channel < Config::NUM_CHANNELS) {
            selectedChannel = channel;
            if(currentScreen == Screen::MAIN_MENU) {
              currentScreen = Screen::CHANNEL_STATUS;
            }
          }
          break;
          }

        case RemoteKeys::KEY_4: {
          uint8_t channel = 3;
          if(channel < Config::NUM_CHANNELS) {
            selectedChannel = channel;
            if(currentScreen == Screen::MAIN_MENU) {
              currentScreen = Screen::CHANNEL_STATUS;
            }
          }
          break;
          }

       case RemoteKeys::KEY_OK:
//          toggleCharging();
        if(currentScreen == Screen::CHANNEL_STATUS) {
          channels[selectedChannel].isCharging = !channels[selectedChannel].isCharging;
        }
          break;

        case RemoteKeys::KEY_VOL_UP:
          if(maintenanceMode) sendChargePulse(selectedChannel);
          if(currentScreen == Screen::CHARGE_SETTINGS) {adjustCurrentSetting(selectedChannel, Config::CHARGE_CURRENT_STEP);}
 
          break;
        case RemoteKeys::KEY_VOL_DOWN:
          if(maintenanceMode) sendDischargePulse(selectedChannel);
          if(currentScreen == Screen::CHARGE_SETTINGS) {adjustCurrentSetting(selectedChannel, -Config::CHARGE_CURRENT_STEP);}
          break;

          
        case RemoteKeys::KEY_RED:
          currentScreen = Screen::CHARGE_SETTINGS;
          break;
        case RemoteKeys::KEY_GREEN:
          currentScreen = Screen::GRAPH_VIEW;
          break;
        case RemoteKeys::KEY_YELLOW:
          currentScreen = Screen::MAINTENANCE;
          break;
        case RemoteKeys::KEY_BLUE:
          currentScreen = Screen::DIAGNOSTICS;
  
        case RemoteKeys::KEY_CH_UP:
        case RemoteKeys::KEY_CH_DOWN:
 //         adjustCurrentSetting(selectedChannel,IrReceiver.decodedIRData.command == RemoteKeys::KEY_UP ? 
 //                      Config::CHARGE_CURRENT_STEP : -Config::CHARGE_CURRENT_STEP);
          break;
      }
            }
         }
      IrReceiver.resume();
    }
  }
  
  void drawScreen() {
    display.clearDisplay();
    
    // Draw header for all screens
    //drawHeader();
    
    // Draw main content
    switch(currentScreen) {
      case Screen::MAIN_MENU:
        drawMainMenu();
        break;
      case Screen::CHANNEL_STATUS:
        drawChannelStatus();
        break;
      case Screen::CHARGE_SETTINGS:
        //drawChargeSettings();
        break;
      case Screen::GRAPH_VIEW:
        drawGraphView();
        break;
      case Screen::MAINTENANCE:
        drawMaintenance();
        break;
      case Screen::DIAGNOSTICS:
        //drawDiagnostics();
        break;
    }
    
    // Draw status bar for all screens
    //drawStatusBar();
    
    display.display();
  }
  
  void drawHeader() {
    display.drawFastHLine(0, Config::Layout::HEADER_HEIGHT - 1, 
                         Config::SCREEN_WIDTH, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    switch(currentScreen) {
      case Screen::MAIN_MENU:
        display.println(F("NiMH Charger"));
        break;
      case Screen::CHANNEL_STATUS:
        display.print(F("Channel "));
        display.println(selectedChannel + 1);
        break;
      // Add other screen headers
    }
  }
  
  void drawStatusBar() {
    const auto& area = Config::Layout::STATUS_BAR;
    display.drawFastHLine(area.x, area.y, area.width, SSD1306_WHITE);
    display.setCursor(area.x + Config::Layout::MARGIN, 
                     area.y + Config::Layout::MARGIN);
    
    // Draw relevant status info based on current screen
    switch(currentScreen) {
      case Screen::MAINTENANCE:
        display.print(F("G:Chg Y:Dis R:Exit"));
        break;
      default:
        // Show basic system status
        display.print(F("M:Menu \x18\x19:Adj"));
        break;
    }
  }
  
  // Implementation of other drawing methods...
  // (drawMainMenu, drawChannelStatus, etc. following similar structured approach)


};

// Global instance
BatteryChargerUI chargerUI;

void setup() {
  Serial.begin(115200);
  chargerUI.begin();
}

void loop() {
  chargerUI.update();
}
