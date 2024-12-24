#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// IR settings
#define IR_PIN 15
IRrecv irrecv(IR_PIN);
decode_results results;

// System settings
#define NUM_CHANNELS 4
#define MAX_COMPONENTS 10
#define MAX_GRAPH_POINTS 50

// Component types
enum ComponentType {
  TEXT,
  RECTANGLE,
  LINE,
  GRAPH
};

// Component structure
struct Component {
  ComponentType type;       // Type of component
  int x, y;                 // Position
  int width, height;        // Size (for RECTANGLE, GRAPH)
  const char* label;        // Label for TEXT
  int* data;                // Data array for GRAPH
  int dataCount;            // Number of points in the graph
  void (*callback)();       // Action for interactive components (optional)
};

// Layout structure
struct Layout {
  const char* name;         // Layout name
  Component components[MAX_COMPONENTS];  // Components array
  int componentCount;       // Number of components
};

// Global variables
int selectedChannel = 0;
int currentLayoutIndex = 0;
int graphData[MAX_GRAPH_POINTS] = {0};
int graphDataCount = 0;

struct ChannelSettings {
  int chargeRate;  // Charge rate in mA
  int rampProfile; // Ramp profile index
  float voltage;   // Battery voltage
  float current;   // Charge current
};

ChannelSettings channels[NUM_CHANNELS];

// Function prototypes
void drawLayout();
void drawComponent(const Component* component);
void switchLayout(int newIndex);

// Callback functions for screen-specific actions
void switchToChargeSettings();
void switchToDiagnostics();
void switchToChargeGraph();
void switchToManualControl();
void nextChannel();
void previousChannel();
void triggerChargePulse();
void triggerDischargePulse();

// Define layouts
Layout layouts[] = {
  {
    "Main Menu",
    {
      {TEXT, 0, 0, 0, 0, "Main Menu", nullptr, 0, nullptr},
      {TEXT, 0, 10, 0, 0, "1: Charge Settings", nullptr, 0, switchToChargeSettings},
      {TEXT, 0, 20, 0, 0, "2: Diagnostics", nullptr, 0, switchToDiagnostics},
      {TEXT, 0, 30, 0, 0, "3: Charge Graph", nullptr, 0, switchToChargeGraph},
      {TEXT, 0, 40, 0, 0, "4: Manual Control", nullptr, 0, switchToManualControl}
    },
    5
  },
  {
    "Charge Graph",
    {
      {TEXT, 0, 0, 0, 0, "Charge Graph", nullptr, 0, nullptr},
      {GRAPH, 10, 20, 100, 30, nullptr, graphData, MAX_GRAPH_POINTS, nullptr}
    },
    2
  },
  {
    "Manual Control",
    {
      {TEXT, 0, 0, 0, 0, "Manual Control", nullptr, 0, nullptr},
      {TEXT, 0, 10, 0, 0, "Channel: ", nullptr, 0, nullptr},
      {TEXT, 70, 10, 0, 0, nullptr, nullptr, 0, nullptr},  // Dynamic channel
      {TEXT, 0, 20, 0, 0, "Voltage: ", nullptr, 0, nullptr},
      {TEXT, 70, 20, 0, 0, nullptr, nullptr, 0, nullptr},  // Dynamic voltage
      {TEXT, 0, 30, 0, 0, "IR+: Charge Pulse", nullptr, 0, triggerChargePulse},
      {TEXT, 0, 40, 0, 0, "IR-: Discharge Pulse", nullptr, 0, triggerDischargePulse}
    },
    7
  }
};

#define NUM_LAYOUTS (sizeof(layouts) / sizeof(Layout))

void setup() {
  // Initialize OLED
  if (!display.begin(SSD1306_I2C_ADDRESS, 0x3C)) {
    for (;;);
  }
  display.clearDisplay();
  display.display();

  // Initialize IR
  irrecv.enableIRIn();

  // Initialize channels
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i].chargeRate = 500;  // Default charge rate
    channels[i].rampProfile = 0;  // Default ramp
    channels[i].voltage = 1.2;    // Example voltage
    channels[i].current = 0.0;    // Default current
  }
}

void loop() {
  if (irrecv.decode(&results)) {
    handleIRInput();
    irrecv.resume();
  }

  drawLayout();
}

// Draw the active layout
void drawLayout() {
  display.clearDisplay();
  Layout* layout = &layouts[currentLayoutIndex];

  for (int i = 0; i < layout->componentCount; i++) {
    drawComponent(&layout->components[i]);
  }

  display.display();
}

// Draw a single component
void drawComponent(const Component* component) {
  switch (component->type) {
    case TEXT:
      if (component->label != nullptr) {
        display.setCursor(component->x, component->y);
        display.setTextSize(1);
        display.print(component->label);
      }
      break;
    case RECTANGLE:
      display.drawRect(component->x, component->y, component->width, component->height, SSD1306_WHITE);
      break;
    case LINE:
      display.drawLine(component->x, component->y, component->x + component->width, component->y + component->height, SSD1306_WHITE);
      break;
    case GRAPH:
      if (component->data != nullptr && component->dataCount > 0) {
        for (int i = 0; i < component->dataCount - 1; i++) {
          int x1 = component->x + (i * component->width / component->dataCount);
          int y1 = component->y + component->height - component->data[i];
          int x2 = component->x + ((i + 1) * component->width / component->dataCount);
          int y2 = component->y + component->height - component->data[i + 1];
          display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
        }
      }
      break;
  }
}

// Switch layouts
void switchLayout(int newIndex) {
  if (newIndex >= 0 && newIndex < NUM_LAYOUTS) {
    currentLayoutIndex = newIndex;
  }
}

// IR input handling
void handleIRInput() {
  switch (results.value) {
    case 0x10: switchLayout(0); break;
    case 0x11: switchToChargeSettings(); break;
    case 0x12: switchToDiagnostics(); break;
    case 0x13: switchToChargeGraph(); break;
    case 0x14: switchToManualControl(); break;
    case 0x20: nextChannel(); break;
    case 0x21: previousChannel(); break;
  }
}
