#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin not used
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// IR settings
#define IR_PIN 15
IRrecv irrecv(IR_PIN);
decode_results results;

// System settings
#define NUM_CHANNELS 4

// Global variables for interface
enum Screen { MAIN_MENU, CHARGE_SETTINGS, DIAGNOSTICS, CHARGE_GRAPH, MANUAL_CONTROL };
Screen currentScreen = MAIN_MENU;

struct ChannelSettings {
  int chargeRate;     // Charge rate in mA
  int rampProfile;    // Ramp profile index
  float voltage;      // Battery voltage
  float current;      // Charge current
};

ChannelSettings channels[NUM_CHANNELS];
int selectedChannel = 0;  // Active channel for configuration

// Define remote key mappings
enum RemoteKey {
  KEY_MAIN_MENU = 0x10,
  KEY_CHARGE_SETTINGS = 0x11,
  KEY_DIAGNOSTICS = 0x12,
  KEY_CHARGE_GRAPH = 0x13,
  KEY_MANUAL_CONTROL = 0x14,
  KEY_NEXT_CHANNEL = 0x20,
  KEY_PREV_CHANNEL = 0x21,
  KEY_INCREASE_RATE = 0x30,
  KEY_DECREASE_RATE = 0x31,
  KEY_NEXT_RAMP = 0x40,
  KEY_CHARGE_PULSE = 0x50,
  KEY_DISCHARGE_PULSE = 0x51
};

// Define a structure for key-action mapping
struct RemoteAction {
  void (*action)();
};

// Declare remote action mappings
RemoteAction remoteActions[] = {
  {switchToMainMenu},
  {switchToChargeSettings},
  {switchToDiagnostics},
  {switchToChargeGraph},
  {switchToManualControl},
  {selectNextChannel},
  {selectPreviousChannel},
  {increaseChargeRate},
  {decreaseChargeRate},
  {cycleRampProfile},
  {triggerChargePulse},
  {triggerDischargePulse}
};

// Map remote keys to actions
RemoteKey keyMap[] = {
  KEY_MAIN_MENU,
  KEY_CHARGE_SETTINGS,
  KEY_DIAGNOSTICS,
  KEY_CHARGE_GRAPH,
  KEY_MANUAL_CONTROL,
  KEY_NEXT_CHANNEL,
  KEY_PREV_CHANNEL,
  KEY_INCREASE_RATE,
  KEY_DECREASE_RATE,
  KEY_NEXT_RAMP,
  KEY_CHARGE_PULSE,
  KEY_DISCHARGE_PULSE
};

// Function prototypes for each action
void switchToMainMenu();
void switchToChargeSettings();
void switchToDiagnostics();
void switchToChargeGraph();
void switchToManualControl();
void selectNextChannel();
void selectPreviousChannel();
void increaseChargeRate();
void decreaseChargeRate();
void cycleRampProfile();
void triggerChargePulse();
void triggerDischargePulse();

void setup() {
  // Initialize OLED
  if (!display.begin(SSD1306_I2C_ADDRESS, 0x3C)) {
    for (;;); // Don't proceed if the display is not connected
  }
  display.clearDisplay();
  display.display();

  // Initialize IR receiver
  irrecv.enableIRIn();

  // Initialize channels
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i].chargeRate = 500;  // Default 500mA
    channels[i].rampProfile = 0;  // Default linear ramp
    channels[i].voltage = 1.2;    // Example default voltage
    channels[i].current = 0.0;    // Default current
  }
}

void loop() {
  if (irrecv.decode(&results)) {
    handleIRInput();
    irrecv.resume();  // Prepare to receive the next code
  }

  updateScreen();
}

// Handle IR input
void handleIRInput() {
  for (size_t i = 0; i < sizeof(keyMap) / sizeof(keyMap[0]); i++) {
    if (results.value == keyMap[i]) {
      remoteActions[i].action();
      return;
    }
  }
}

// Action definitions
void switchToMainMenu() {
  currentScreen = MAIN_MENU;
}

void switchToChargeSettings() {
  currentScreen = CHARGE_SETTINGS;
}

void switchToDiagnostics() {
  currentScreen = DIAGNOSTICS;
}

void switchToChargeGraph() {
  currentScreen = CHARGE_GRAPH;
}

void switchToManualControl() {
  currentScreen = MANUAL_CONTROL;
}

void selectNextChannel() {
  selectedChannel = (selectedChannel + 1) % NUM_CHANNELS;
}

void selectPreviousChannel() {
  selectedChannel = (selectedChannel - 1 + NUM_CHANNELS) % NUM_CHANNELS;
}

void increaseChargeRate() {
  channels[selectedChannel].chargeRate += 100;
}

void decreaseChargeRate() {
  channels[selectedChannel].chargeRate -= 100;
}

void cycleRampProfile() {
  channels[selectedChannel].rampProfile = (channels[selectedChannel].rampProfile + 1) % 3;
}

void triggerChargePulse() {
  channels[selectedChannel].current += 0.5; // Example: add 0.5A current pulse
}

void triggerDischargePulse() {
  channels[selectedChannel].current -= 0.5; // Example: subtract 0.5A current pulse
}

// Update the current screen
void updateScreen() {
  display.clearDisplay();
  switch (currentScreen) {
    case MAIN_MENU: drawMainMenu(); break;
    case CHARGE_SETTINGS: drawChargeSettings(); break;
    case DIAGNOSTICS: drawDiagnostics(); break;
    case CHARGE_GRAPH: drawChargeGraph(); break;
    case MANUAL_CONTROL: drawManualControl(); break;
  }
  display.display();
}

// Screen drawing functions
void drawMainMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Main Menu");
  display.println("1: Charge Settings");
  display.println("2: Diagnostics");
  display.println("3: Charge Graph");
  display.println("4: Manual Control");
  display.println("CH+: Next Channel");
  display.println("CH-: Prev Channel");
}

void drawChargeSettings() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Charge Settings");
  display.printf("Channel: %d\n", selectedChannel + 1);
  display.printf("Rate: %d mA\n", channels[selectedChannel].chargeRate);
  display.printf("Ramp: %d\n", channels[selectedChannel].rampProfile);
}

void drawDiagnostics() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Diagnostics");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    display.printf("CH%d: %.2fV %.2fA\n", i + 1, channels[i].voltage, channels[i].current);
  }
}

void drawChargeGraph() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Charge Graph");
  display.println("Graph updates go here...");
}

void drawManualControl() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Manual Control");
  display.printf("Channel: %d\n", selectedChannel + 1);
  display.printf("Voltage: %.2fV\n", channels[selectedChannel].voltage);
  display.printf("Current: %.2fA\n", channels[selectedChannel].current);
  display.println("IR+: Charge Pulse");
  display.println("IR-: Discharge Pulse");
}
