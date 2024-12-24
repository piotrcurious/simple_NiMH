#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

// Configuration and Constants
namespace Config {
  // Display
  constexpr uint8_t SCREEN_WIDTH = 128;
  constexpr uint8_t SCREEN_HEIGHT = 64;
  constexpr int8_t OLED_RESET = -1;
  constexpr uint8_t SCREEN_ADDRESS = 0x3C;
  
  // Hardware
  constexpr uint8_t IR_PIN = 15;
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
    KEY_0 = 0x00,
    KEY_1 = 0x01,
    KEY_2 = 0x02,
    KEY_3 = 0x03,
    KEY_4 = 0x04,
    KEY_5 = 0x05,
    KEY_6 = 0x06,
    KEY_7 = 0x07,
    KEY_8 = 0x08,
    KEY_9 = 0x09,
    KEY_UP = 0x20,
    KEY_DOWN = 0x21,
    KEY_LEFT = 0x15,
    KEY_RIGHT = 0x16,
    KEY_OK = 0x17,
    KEY_MENU = 0x30,
    KEY_RED = 0x37,
    KEY_GREEN = 0x36,
    KEY_YELLOW = 0x32,
    KEY_BLUE = 0x34
  };
}

// UI Elements and Icons
namespace UI {
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
  Adafruit_SSD1306 display;
  IRrecv irrecv;
  Screen currentScreen;
  uint8_t selectedChannel;
  BatteryChannel channels[Config::NUM_CHANNELS];
  uint32_t lastUpdate;
  bool maintenanceMode;

public:
  BatteryChargerUI() : 
    display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, Config::OLED_RESET),
    irrecv(Config::IR_PIN),
    currentScreen(Screen::MAIN_MENU),
    selectedChannel(0),
    lastUpdate(0),
    maintenanceMode(false) {}
    
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
    irrecv.enableIRIn();
    
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
    if (irrecv.decode(&results)) {
      switch(results.value) {
        case RemoteKeys::KEY_MENU:
          currentScreen = Screen::MAIN_MENU;
          break;
        case RemoteKeys::KEY_1:
        case RemoteKeys::KEY_2:
        case RemoteKeys::KEY_3:
        case RemoteKeys::KEY_4: {
          uint8_t channel = results.value - RemoteKeys::KEY_1;
          if(channel < Config::NUM_CHANNELS) {
            selectedChannel = channel;
            if(currentScreen == Screen::MAIN_MENU) {
              currentScreen = Screen::CHANNEL_STATUS;
            }
          }
          break;
        }
        case RemoteKeys::KEY_RED:
          toggleCharging();
          break;
        case RemoteKeys::KEY_GREEN:
          if(maintenanceMode) sendChargePulse();
          break;
        case RemoteKeys::KEY_YELLOW:
          if(maintenanceMode) sendDischargePulse();
          break;
        case RemoteKeys::KEY_UP:
        case RemoteKeys::KEY_DOWN:
          adjustCurrent(results.value == RemoteKeys::KEY_UP ? 
                       Config::CHARGE_CURRENT_STEP : -Config::CHARGE_CURRENT_STEP);
          break;
      }
      irrecv.resume();
    }
  }
  
  void drawScreen() {
    display.clearDisplay();
    
    // Draw header for all screens
    drawHeader();
    
    // Draw main content
    switch(currentScreen) {
      case Screen::MAIN_MENU:
        drawMainMenu();
        break;
      case Screen::CHANNEL_STATUS:
        drawChannelStatus();
        break;
      case Screen::CHARGE_SETTINGS:
        drawChargeSettings();
        break;
      case Screen::GRAPH_VIEW:
        drawGraphView();
        break;
      case Screen::MAINTENANCE:
        drawMaintenance();
        break;
      case Screen::DIAGNOSTICS:
        drawDiagnostics();
        break;
    }
    
    // Draw status bar for all screens
    drawStatusBar();
    
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
