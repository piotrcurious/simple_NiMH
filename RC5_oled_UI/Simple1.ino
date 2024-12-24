#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// IR Remote Key Mappings (typical RC5 codes)
#define KEY_0 0x00
#define KEY_1 0x01
#define KEY_2 0x02
#define KEY_3 0x03
#define KEY_4 0x04
#define KEY_5 0x05
#define KEY_6 0x06
#define KEY_7 0x07
#define KEY_8 0x08
#define KEY_9 0x09
#define KEY_UP 0x20
#define KEY_DOWN 0x21
#define KEY_LEFT 0x15
#define KEY_RIGHT 0x16
#define KEY_OK 0x17
#define KEY_MENU 0x30
#define KEY_RED 0x37
#define KEY_GREEN 0x36
#define KEY_YELLOW 0x32
#define KEY_BLUE 0x34

// Screen IDs
enum Screen {
  MAIN_MENU,
  CHANNEL_STATUS,
  CHARGE_SETTINGS,
  GRAPH_VIEW,
  MAINTENANCE,
  DIAGNOSTICS
};

// Charge Profiles
enum ChargeProfile {
  NORMAL,
  GENTLE,
  AGGRESSIVE,
  RECOVERY
};

// Battery Channel Structure
struct BatteryChannel {
  float voltage;
  float current;
  bool isCharging;
  float targetCurrent;
  ChargeProfile profile;
  float temperatureC;
  float capacityMah;
  unsigned long chargeStartTime;
  int graphData[SCREEN_WIDTH];  // Rolling voltage history
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
IRrecv irrecv(15);  // IR receiver on GPIO15
decode_results results;

Screen currentScreen = MAIN_MENU;
int selectedChannel = 0;
BatteryChannel channels[4];
unsigned long lastUpdate = 0;
bool maintenanceMode = false;
int graphUpdateInterval = 1000;

// Custom Battery Symbol (8x16 pixels)
const unsigned char batteryIcon [] PROGMEM = {
  0x3C, 0x3C, 0x42, 0x42, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x42, 0x42, 0x3C, 0x3C
};

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Initialize IR receiver
  irrecv.enableIRIn();
  
  // Initialize channels with default values
  for(int i = 0; i < 4; i++) {
    channels[i].voltage = 0;
    channels[i].current = 0;
    channels[i].isCharging = false;
    channels[i].targetCurrent = 500;  // Default 500mA
    channels[i].profile = NORMAL;
    channels[i].temperatureC = 25;
    channels[i].capacityMah = 0;
    memset(channels[i].graphData, 0, sizeof(channels[i].graphData));
  }
}

void loop() {
  handleIRInput();
  updateMeasurements();
  updateDisplay();
}

void handleIRInput() {
  if (irrecv.decode(&results)) {
    switch(results.value) {
      case KEY_MENU:
        currentScreen = MAIN_MENU;
        break;
      case KEY_1:
      case KEY_2:
      case KEY_3:
      case KEY_4:
        selectedChannel = results.value - KEY_1;
        if(currentScreen == MAIN_MENU) {
          currentScreen = CHANNEL_STATUS;
        }
        break;
      case KEY_RED:
        if(currentScreen == CHANNEL_STATUS) {
          channels[selectedChannel].isCharging = !channels[selectedChannel].isCharging;
        }
        break;
      case KEY_GREEN:
        if(currentScreen == MAINTENANCE) {
          // Send charge pulse in maintenance mode
          sendChargePulse(selectedChannel);
        }
        break;
      case KEY_YELLOW:
        if(currentScreen == MAINTENANCE) {
          // Send discharge pulse in maintenance mode
          sendDischargePulse(selectedChannel);
        }
        break;
      case KEY_UP:
      case KEY_DOWN:
        adjustCurrentSetting(selectedChannel, results.value == KEY_UP ? 100 : -100);
        break;
    }
    irrecv.resume();
  }
}

void updateDisplay() {
  display.clearDisplay();
  
  switch(currentScreen) {
    case MAIN_MENU:
      drawMainMenu();
      break;
    case CHANNEL_STATUS:
      drawChannelStatus();
      break;
    case CHARGE_SETTINGS:
      drawChargeSettings();
      break;
    case GRAPH_VIEW:
      drawGraphView();
      break;
    case MAINTENANCE:
      drawMaintenance();
      break;
    case DIAGNOSTICS:
      drawDiagnostics();
      break;
  }
  
  display.display();
}

void drawMainMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("NiMH Charger"));
  
  for(int i = 0; i < 4; i++) {
    display.drawBitmap(0, 16 + i * 12, batteryIcon, 8, 16, SSD1306_WHITE);
    display.setCursor(12, 16 + i * 12);
    display.print(F("Ch"));
    display.print(i + 1);
    display.print(F(": "));
    display.print(channels[i].voltage, 1);
    display.print(F("V "));
    if(channels[i].isCharging) {
      display.print(char(0x18));  // Up arrow
    }
  }
}

void drawChannelStatus() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Channel "));
  display.println(selectedChannel + 1);
  
  display.print(F("V: "));
  display.print(channels[selectedChannel].voltage, 2);
  display.println(F("V"));
  
  display.print(F("I: "));
  display.print(channels[selectedChannel].current, 0);
  display.println(F("mA"));
  
  display.print(F("T: "));
  display.print(channels[selectedChannel].temperatureC, 1);
  display.println(F("C"));
  
  // Mini graph on right side
  drawMiniVoltageGraph(64, 0, 64, 32);
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
  for(int i = 0; i < SCREEN_WIDTH - 11; i++) {
    int y = map(channels[selectedChannel].graphData[i], 0, 2000, 63, 10);
    display.drawPixel(i + 11, y, SSD1306_WHITE);
  }
}

void drawMaintenance() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Maintenance Ch"));
  display.println(selectedChannel + 1);
  
  display.print(F("V: "));
  display.print(channels[selectedChannel].voltage, 2);
  display.println(F("V"));
  
  // Rolling voltage graph
  drawMiniVoltageGraph(0, 16, SCREEN_WIDTH, 48);
  
  // Control hints
  display.setCursor(0, 56);
  display.print(F("G:Chg Y:Dis B:Exit"));
}

void drawMiniVoltageGraph(int x, int y, int w, int h) {
  // Draw mini graph frame
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  
  // Plot data points
  for(int i = 0; i < w - 2; i++) {
    if(i < SCREEN_WIDTH) {
      int dataY = map(channels[selectedChannel].graphData[i], 0, 2000, y + h - 2, y + 1);
      display.drawPixel(x + i + 1, dataY, SSD1306_WHITE);
    }
  }
}

void updateMeasurements() {
  if(millis() - lastUpdate > 1000) {
    for(int i = 0; i < 4; i++) {
      // Simulate reading measurements
      // Replace with actual ADC readings
      if(channels[i].isCharging) {
        channels[i].voltage += random(-10, 11) / 100.0;
        channels[i].current = channels[i].targetCurrent + random(-50, 51);
        channels[i].temperatureC += random(-1, 2);
        
        // Update graph data
        for(int j = 0; j < SCREEN_WIDTH - 1; j++) {
          channels[i].graphData[j] = channels[i].graphData[j + 1];
        }
        channels[i].graphData[SCREEN_WIDTH - 1] = channels[i].voltage * 100;
      }
    }
    lastUpdate = millis();
  }
}

void adjustCurrentSetting(int channel, int delta) {
  channels[channel].targetCurrent += delta;
  if(channels[channel].targetCurrent < 0) channels[channel].targetCurrent = 0;
  if(channels[channel].targetCurrent > 2000) channels[channel].targetCurrent = 2000;
}

void sendChargePulse(int channel) {
  // Implement actual hardware control
  channels[channel].current = 500;  // 500mA pulse
  delay(100);
  channels[channel].current = 0;
}

void sendDischargePulse(int channel) {
  // Implement actual hardware control
  channels[channel].current = -200;  // -200mA pulse
  delay(100);
  channels[channel].current = 0;
}
