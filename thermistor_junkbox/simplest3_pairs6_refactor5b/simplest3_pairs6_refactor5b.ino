#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <IRremote.h>
#include <cmath>     // For math functions like log, exp, pow, isnan
#include <limits>    // For std::numeric_limits
#include <vector>    // For dynamic arrays
#include <numeric>
#include <algorithm>


#include "SHT4xSensor.h"
#include "ThermistorSensor.h"


// Structure to hold measurement data
struct MeasurementData {
    float voltage;
    float current;
    double temp1;
    double temp2;
    double tempDiff;
    float t1_millivolts;
    int dutyCycle;
    unsigned long timestamp;
};


// --- Configuration Section ---
// Define TFT display pins
#define TFT_CS          15  // Chip Select pin for TFT CS
#define TFT_DC          2   // Data Command pin for TFT DC
#define TFT_RST         4   // Reset pin for TFT RST (or -1 if not used, connect to ESP32 enable pin)
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Define Thermistor pins
#define THERMISTOR_PIN_1 36 // Analog pin for Thermistor 1
//double THERMISTOR_1_OFFSET = -960.0;  // zero offset to cancel out slight thermistor differences
double THERMISTOR_1_OFFSET = 0.0;  // zero offset to cancel out slight thermistor differences and adc nonlinearity

#define THERMISTOR_VCC_PIN 35 // analog pin to measure VCC for thermistors

// Define Voltage Readout Pin
#define VOLTAGE_READ_PIN 39    // analog pin to measure battery voltage
//#define MAIN_VCC_RATIO 4.125 // multiplier to determine VCC from thermistor divider VCC
#define MAIN_VCC_RATIO 2.0 // multiplier to determine VCC from thermistor divider VCC

uint32_t voltage_last_time;
uint32_t voltage_update_interval = 250;

// Define Current Shunt Pin and Value
#define CURRENT_SHUNT_PIN 34
#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 140 // offset of current pin

// Define PWM Pin
#define PWM_PIN 19

// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const double MIN_TEMP             = 15.0;
const double MAX_TEMP             = 30.0;
const double MIN_DIFF_TEMP        = -1.0; // Difference range -3 to +3
const double MAX_DIFF_TEMP        = 1.0;

// Fixed voltage range for scaling the voltage plot
const float MIN_VOLTAGE = 0.5f;
const float MAX_VOLTAGE = 2.4f;

// Fixed current range for scaling the current plot - ADJUST IF NEEDED
const float MIN_CURRENT = 0.0f;
const float MAX_CURRENT = 0.5f; // Adjust this based on your expected current range

// Plotting parameters - Maximized graph and bottom labels - ADJUST IF NEEDED
#define PLOT_WIDTH          320           // Maximize width (almost full screen)
#define PLOT_HEIGHT         (216 - 3)     // Maximize height (leaving space for labels at bottom)
#define PLOT_X_START        0             // Adjusted start position for wider graph
#define PLOT_Y_START        0             // Adjusted start position from top

// Plot colors - ADJUST IF DESIRED
#define PLOT_X_AXIS_COLOR   TFT_WHITE
#define PLOT_Y_AXIS_COLOR   TFT_WHITE
#define PLOT_ZERO_COLOR     0x62ec

#define GRAPH_COLOR_1       TFT_RED
#define GRAPH_COLOR_2       TFT_GREEN
#define GRAPH_COLOR_DIFF    TFT_BLUE
#define GRAPH_COLOR_VOLTAGE TFT_YELLOW
#define GRAPH_COLOR_CURRENT TFT_MAGENTA // Define a color for the current plot
#define GRAPH_COLOR_RESISTANCE TFT_ORANGE
#define GRAPH_COLOR_RESISTANCE_PAIR TFT_CYAN

// Label area at the bottom
#define LABEL_Y_START       PLOT_Y_START + PLOT_HEIGHT + 3 // Start labels below graph
#define LABEL_TEXT_SIZE     1


// IR Remote setup
#define IR_RECEIVE_PIN 15

// IR Remote Key Definitions
namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0x02,

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
    KEY_UP          = 0x60,
    KEY_DOWN        = 0x61,
    KEY_LEFT        = 0x65,
    KEY_RIGHT       = 0x62,
    KEY_OK          = 0x68,
    KEY_MENU        = 0x79,
 
    KEY_RED         = 0x6c,
    KEY_GREEN       = 0x14,
    KEY_YELLOW      = 0x15,
    KEY_BLUE        = 0x16,
 
    KEY_VOL_UP      = 0x07,
    KEY_VOL_DOWN    = 0x0b,
    KEY_CH_UP       = 0x12,
    KEY_CH_DOWN     = 0x10,
    
    KEY_REWIND      = 0x45,
    KEY_PLAY        = 0x47,
    KEY_PAUSE       = 0x4A,
    KEY_FORWARD     = 0x48,
    KEY_STOP        = 0x46,

    KEY_SETTINGS    = 0x1A,
    KEY_INFO        = 0x1F,
    KEY_SUBTITLES   = 0x25,
    KEY_MUTE        = 0x0F,
    KEY_NETFLIX     = 0xF3,
    KEY_PRIME_VIDEO = 0xF4

  };
}


// --- Global variables ---
TFT_eSPI tft = TFT_eSPI(); // TFT_eSPI instance

// --- Temperature sensor objects
SHT4xSensor sht4Sensor;
ThermistorSensor thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);

// Temperature reading arrays for plotting
float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// Voltage reading array for plotting
float voltage_values[PLOT_WIDTH];

// Current reading array for plotting
float current_values[PLOT_WIDTH];

// Global variable for voltage readout
float voltage_mv = 1000.0f;

// Global variable for current readout
float current_ma = 0.0f;

// --- PWM Control Variables ---
#define PWM_FREQUENCY 1000 // define frequency
const int pwmPin = PWM_PIN;
const int pwmResolutionBits = 8; // You can adjust the resolution (e.g., 8, 10, 12)
const int pwmMaxDutyCycle = (1 << pwmResolutionBits) - 1;
const unsigned long pwmCycleDuration = 60000; // 60 seconds
const unsigned long pwmRampUpDuration = 30000; // 10 seconds
unsigned long pwmStartTime = 0;
uint32_t dutyCycle = 0 ;

// --- Internal Resistance Measurement Variables ---
const int MAX_RESISTANCE_POINTS = 50; // Maximum number of data points for internal resistance
float internalResistanceData[MAX_RESISTANCE_POINTS][2]; // [current, internal_resistance]
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2]; // [current, internal_resistance] // from consecutive pairs
int resistanceDataCountPairs = 0;
float regressedInternalResistance = 0 ; 
float regressedInternalResistancePairs = 0 ; // from consecutive pairs
bool isMeasuringResistance = false;

// --- Function Declarations ---
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map
void controlPWM();
void setupPWM();
void measureInternalResistance();
void displayInternalResistanceGraph();

// --- Function Implementations ---

// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// global variables for sht4x readouts (now managed by the class)
// float aTemperature = 25.0;
// float aHumidity = 50.0;

void task_readSHT4x(void* parameter) {
    while (true) {
        sht4Sensor.read();
        vTaskDelay(100);
    }
}

void task_readThermistor(void* parameter) {
    while (true) {
        uint32_t current_time = millis();
        while (sht4Sensor.isLocked()) {
            vTaskDelay(1);
        }; // if lock is set, wait for sht4 task to update the data
        thermistorSensor.read(sht4Sensor.getTemperature());
        // Read voltage and store in global variable
// no need repeat     //analogSetPinAttenuation(VOLTAGE_READ_PIN, ADC_11db); // Set attenuation to 11dB
//      voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - analogReadMilliVolts(VOLTAGE_READ_PIN);

        // Read current and store in global variable
//        double voltageAcrossShunt = analogReadMilliVolts(CURRENT_SHUNT_PIN) - CURRENT_SHUNT_PIN_ZERO_OFFSET ;
    int task_current_numSamples = 256; 
    double sumAnalogValues = 0;
    for (int i = 0; i < task_current_numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(CURRENT_SHUNT_PIN);
        sumAnalogValues += analogValue;
    }
    double voltageAcrossShunt = (sumAnalogValues / task_current_numSamples) - CURRENT_SHUNT_PIN_ZERO_OFFSET;

        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE); // Convert to mA
//  current_ma = voltageAcrossShunt ; // Convert to mA

    if ((current_time - voltage_last_time) > voltage_update_interval) {
        //analogWrite(pwmPin, 0); // set current to 0 to not interfere with voltage reading
        //vTaskDelay(100);
//        voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - analogReadMilliVolts(VOLTAGE_READ_PIN);

    int task_voltage_numSamples = 256; 
    double sumAnalogValues = 0;
    for (int i = 0; i < task_voltage_numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(VOLTAGE_READ_PIN);
        sumAnalogValues += analogValue;
    }
    voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - (sumAnalogValues / task_voltage_numSamples);

        
        //analogWrite(pwmPin, dutyCycle);
        voltage_last_time = current_time;
    }

        vTaskDelay(50);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize TFT
    tft.init();
    tft.setRotation(1); // Adjust rotation as needed
    // Clear screen and set background color
    tft.fillScreen(TFT_BLACK);

    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);     

    // Initialize temperature, voltage, and current arrays to a default value
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f; // Initialize voltage array
        current_values[i] = 0.0f;     // Initialize current array
    }

    sht4Sensor.begin(); // Initialize SHT4x sensor
    thermistorSensor.begin(); // Initialize thermistor sensor

//  analogSetAttenuation(ADC_0db);


    // setting of per pin attentuation is broken in the IDF...

//  analogSetPinAttenuation(VOLTAGE_READ_PIN, ADC_11db) ; // Set attenuation to 11dB
    //analogSetPinAttenuation(CURRENT_SHUNT_PIN, ADC_0db); // Set attenuation for current shunt pin
    //analogSetPinAttenuation(THERMISTOR_PIN_1, ADC_0db);
    //analogSetPinAttenuation(thermistor2Pin, ADC_0db);
    //analogSetPinAttenuation(THERMISTOR_VCC_PIN, ADC_0db);

    xTaskCreate(task_readSHT4x,     "SHT4",  4096, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 4096, NULL, 1, NULL); // Create the new thermistor reading task

    setupPWM(); // Initialize PWM

    Serial.println("TFT and Temperature Sensor Example Started");

    // Start internal resistance measurement at startup for demonstration
   // isMeasuringResistance = true;
}

void setupPWM() {
    analogWriteResolution(pwmPin,pwmResolutionBits);
    analogWriteFrequency(pwmPin,PWM_FREQUENCY); // Set the PWM frequency (you can adjust this)
    pinMode(pwmPin, OUTPUT);
    pwmStartTime = millis(); // Initialize the start time for the PWM cycle
    analogWrite(pwmPin, 0); // Initialize PWM with 0 duty cycle
}

void controlPWM() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - pwmStartTime;

    if (elapsedTime < pwmRampUpDuration) {
        // Increase duty cycle from 0 to max over pwmRampUpDuration
        dutyCycle = map(elapsedTime, 0, pwmRampUpDuration, 0, pwmMaxDutyCycle);
        analogWrite(pwmPin, dutyCycle);
    } else if (elapsedTime < pwmCycleDuration) {
        // Keep duty cycle at max for the remaining time until pwmCycleDuration
        //analogWrite(pwmPin, pwmMaxDutyCycle);
        // If you want it to go back to 0 for the second half, uncomment the next line
        // dutyCycle = map(elapsedTime - pwmRampUpDuration, 0, pwmCycleDuration - pwmRampUpDuration, pwmMaxDutyCycle, 0);
        dutyCycle = 0 ;
        analogWrite(pwmPin, dutyCycle);
    } else {
        // Start a new cycle
        pwmStartTime = currentTime;
        analogWrite(pwmPin, 0); // Set duty cycle to 0 at the beginning of the new cycle
    }
}

// Bubble Sort implementation
void bubbleSort(float data[][2], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                // Swap data[j] and data[j+1]
                float temp0 = data[j][0];
                float temp1 = data[j][1];
                data[j][0] = data[j + 1][0];
                data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0;
                data[j + 1][1] = temp1;
            }
        }
    }
}

// ---------------measure Rint

// --- Configuration ---
//const int MAX_RESISTANCE_POINTS = 50; // Define as a constant
const float MEASURABLE_CURRENT_THRESHOLD = 0.04f; // Adjust as needed (40mA)
const int MIN_DUTY_CYCLE_START = 10;
const int MAX_DUTY_CYCLE = 255;
const int DUTY_CYCLE_INCREMENT_FIND_MIN = 5;
const int STABILIZATION_DELAY_MS = 2000;
const int STABILIZATION_PAIRS_FIND_DELAY_MS = 1000; // two task current measurement cycles minimum
const int UNLOADED_VOLTAGE_DELAY_MS = 4000;
const int MIN_DUTY_CYCLE_ADJUSTMENT_STEP = 5;
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.01f;
const float MIN_VALID_RESISTANCE = 0.0f; // Threshold for considering a resistance value valid

// Function to take a measurement
MeasurementData takeMeasurement(int dc,uint32_t stabilization_delay) {
    dutyCycle = dc;
    analogWrite(pwmPin, dc);
//    delay(STABILIZATION_DELAY_MS);
    delay(stabilization_delay);
    MeasurementData data;
    getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    data.dutyCycle = dc;
    data.timestamp = millis();
    return data;
}

// Function to stop the load (duty cycle 0) and wait
void stopLoad() {
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    delay(UNLOADED_VOLTAGE_DELAY_MS);
}

// Function to get an unloaded voltage measurement
MeasurementData getUnloadedVoltageMeasurement() {
    stopLoad();
    MeasurementData data;
    getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    data.dutyCycle = 0;
    data.timestamp = millis();
    return data;
}

// Function to process and store resistance data
void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count < MAX_RESISTANCE_POINTS && resistance > MIN_VALID_RESISTANCE) {
        dataArray[count][0] = current;
        dataArray[count][1] = resistance;
        count++;
    }
}

// Function to print and update thermistor data on serial and TFT
void processThermistorData(const MeasurementData& data, const String& measurementType = "") {
    printThermistorSerial(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    updateTemperatureHistory(data.temp1, data.temp2, data.tempDiff, data.voltage, data.current);
    prepareTemperaturePlot();
    plotTemperatureData();
    displayTemperatureLabels(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
    tft.print(measurementType);
}

// Colors
const uint16_t BLACK = TFT_BLACK;
const uint16_t WHITE = TFT_WHITE;
const uint16_t RED = TFT_RED;
const uint16_t GREEN = TFT_GREEN;
const uint16_t BLUE = TFT_BLUE;
const uint16_t GREY = TFT_DARKGREY;
const uint16_t YELLOW = TFT_YELLOW;

// Visualization parameters
const int DUTY_CYCLE_BAR_Y = 10;
const int DUTY_CYCLE_BAR_HEIGHT = 10;
const int DUTY_CYCLE_BAR_START_X = 30;
const int DUTY_CYCLE_BAR_END_X = SCREEN_WIDTH - 2;

// Function to draw the duty cycle bar
void drawDutyCycleBar(int low, int high, int mid, float current, float threshold) {
  tft.fillRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y, DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1, DUTY_CYCLE_BAR_HEIGHT, GREY);
  tft.drawRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y, DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1, DUTY_CYCLE_BAR_HEIGHT, WHITE);

  // Calculate positions for low, high, and mid markers
  int range = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START;
  if (range == 0) range = 1; // Avoid division by zero

  int low_x = map(low, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
  int high_x = map(high, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
  int mid_x = map(mid, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);

  // Draw low and high markers
  tft.drawLine(low_x, DUTY_CYCLE_BAR_Y - 10, low_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT + 10, WHITE);
  tft.drawLine(high_x, DUTY_CYCLE_BAR_Y - 10, high_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT + 10, WHITE);

  // Draw mid marker with color indicating result
  uint16_t mid_color = (current >= threshold) ? GREEN : RED;
  tft.fillCircle(mid_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT / 2, 8, mid_color);

  // Display current measurement information
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.printf("Low: %d%%, High: %d%%, Mid: %d%%\n", low, high, mid);
  tft.printf("Current: %.3f A (Threshold: %.3f A)\n", current, threshold);
  tft.printf("Measurable: %s\n", (current >= threshold) ? "Yes" : "No");
}

// Function to clear a specific area for updating text
void clearTextArea(int y_start, int height) {
  tft.fillRect(0, y_start, SCREEN_WIDTH, height, BLACK);
}

// Step 1: Find the minimal duty cycle for measurable current using binary search
int findMinimalDutyCycle() {
  Serial.println("Finding minimal duty cycle for measurable current using binary search...");
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.drawString("Finding Min Duty Cycle", 10, 10);

  int low = MIN_DUTY_CYCLE_START;
  int high = MAX_DUTY_CYCLE;
  int minimalDutyCycle = 0; // Initialize to indicate not found yet
  int iteration = 0;

  // --- New variables for the graph ---
  const int GRAPH_X = 30;
  const int GRAPH_Y = 50; // Adjust Y position as needed below the search bar
  const int GRAPH_WIDTH = SCREEN_WIDTH - 2;
  const int GRAPH_HEIGHT = SCREEN_HEIGHT-90;
  const int MAX_GRAPH_POINTS = 100; // Maximum number of points to store
  int dutyCyclePoints[MAX_GRAPH_POINTS];
  float currentPoints[MAX_GRAPH_POINTS];
  int numPoints = 0;
  float maxCurrent = 0.0; // To scale the Y-axis of the graph
  // --- End of new variables ---

  while (low <= high) {
    iteration++;
    int mid = low + (high - low) / 2; // Calculate middle duty cycle

    Serial.printf("Iteration: %d, Testing duty cycle: %d\n", iteration, mid);
    MeasurementData data = takeMeasurement(mid, STABILIZATION_DELAY_MS);
    Serial.printf("Measured current at %d%% duty cycle: %.3f A\n", mid, data.current);
    stopLoad(); // Stop load after each measurement

    // Clear the area for the duty cycle bar and text
    tft.fillRect(0, DUTY_CYCLE_BAR_Y , SCREEN_WIDTH, SCREEN_HEIGHT - DUTY_CYCLE_BAR_Y, BLACK);

    // Draw the duty cycle bar and information
    drawDutyCycleBar(low, high, mid, data.current, MEASURABLE_CURRENT_THRESHOLD);

    // --- Store data for the graph ---
    if (numPoints < MAX_GRAPH_POINTS) {
      dutyCyclePoints[numPoints] = mid;
      currentPoints[numPoints] = data.current;
      numPoints++;
      if (data.current > maxCurrent) {
        maxCurrent = data.current;
      }
    } else {
      // If we reach the maximum number of points, shift the array to make space for the new point
      for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
        dutyCyclePoints[i] = dutyCyclePoints[i + 1];
        currentPoints[i] = currentPoints[i + 1];
      }
      dutyCyclePoints[MAX_GRAPH_POINTS - 1] = mid;
      currentPoints[MAX_GRAPH_POINTS - 1] = data.current;
      if (data.current > maxCurrent) {
        maxCurrent = data.current;
      } else {
        // Recalculate maxCurrent if the newest point is not the maximum
        maxCurrent = 0.0;
        for (int i = 0; i < MAX_GRAPH_POINTS; i++) {
          if (currentPoints[i] > maxCurrent) {
            maxCurrent = currentPoints[i];
          }
        }
      }
    }
    // --- End of storing data ---

    // --- Draw the graph ---
    drawGraph(dutyCyclePoints, currentPoints, numPoints, maxCurrent, GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT);
    // --- End of drawing the graph ---

    if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
      // Found a duty cycle with measurable current.
      // It might be the minimal, so store it and try lower values.
      minimalDutyCycle = mid;
      high = mid - 1; // Search in the lower half
      Serial.printf("Found measurable current at %d%%, trying lower values.\n", mid);
    } else {
      // Current is below the threshold, need a higher duty cycle.
      low = mid + 1; // Search in the upper half
      Serial.printf("Current too low at %d%%, trying higher values.\n", mid);
    }
  }

  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);

  if (minimalDutyCycle > 0) {
    Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
    tft.printf("Minimal Duty Cycle Found:\n%d%%\n", minimalDutyCycle);
    return minimalDutyCycle;
  } else {
    Serial.println("Warning: Could not find a duty cycle producing measurable current.");
    tft.println("Warning: Could not find\nmeasurable current.");
    return 0; // Indicate failure
  }
}

// --- Helper function to draw the graph ---
void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent, int x, int y, int width, int height) {
  if (numPoints <= 1) return; // Need at least two points to draw a line

  // Clear the graph area
  tft.fillRect(x, y, width, height, BLACK);

  // Draw axes
  tft.drawRect(x, y, width, height, WHITE);

  // Calculate scaling factors
  float xScale = (float)width / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START);
  float yScale = (height > 0 && maxCurrent > 0) ? (float)height / maxCurrent : 0;

  // Draw data points and connect them with lines
  for (int i = 0; i < numPoints - 1; i++) {
    int x1 = x + (dutyCycles[i] - MIN_DUTY_CYCLE_START) * xScale;
    int y1 = y + height - currents[i] * yScale;
    int x2 = x + (dutyCycles[i + 1] - MIN_DUTY_CYCLE_START) * xScale;
    int y2 = y + height - currents[i + 1] * yScale;
    tft.drawLine(x1, y1, x2, y2, GREEN); // Use a distinct color for the graph
  }

  // Optionally, draw axis labels and values
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(1);
  tft.drawString("Duty Cycle (%)", x + width / 2 - 40, y + height + 20);
  tft.drawString("(A)", x - 30, y + height / 2);

  // Draw some tick marks on the axes (optional)
  for (int i = MIN_DUTY_CYCLE_START; i <= MAX_DUTY_CYCLE; i += (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START) / 5) {
    int xTick = x + (i - MIN_DUTY_CYCLE_START) * xScale;
    tft.drawLine(xTick, y + height, xTick, y + height + 5, WHITE);
     tft.drawNumber(i, xTick - 10, y + height + 5); // Uncomment if you have a drawNumber function
  }

  if (maxCurrent > 0) {
    for (float i = 0; i <= maxCurrent; i += maxCurrent / 3) {
      int yTick = y + height - i * yScale;
      tft.drawLine(x - 5, yTick, x, yTick, WHITE);
       tft.drawFloat(i, 2, x - 30, yTick - 5); // Uncomment if you have a drawFloat function
    }
  }
  tft.setTextColor(WHITE); // Restore text color
  tft.setTextSize(2); // Restore text size
}



// Improved function to generate duty cycle pairs with approximately linear spacing
// across highDc current increments using binary search
std::vector<std::pair<int, int>> generateDutyCyclePairs(int minDutyCycle) {
    Serial.println("Generating duty cycle pairs (improved for linear current spacing using binary search)...");
    std::vector<std::pair<int, int>> pairs;
    if (minDutyCycle == 0) {
        return pairs; // Return empty if no min duty cycle found
    }

    int numPairs = MAX_RESISTANCE_POINTS / 2;
    if (numPairs < 1) {
        Serial.println("MAX_RESISTANCE_POINTS should be at least 2 for paired measurement.");
        return pairs;
    }

    // First, let's estimate the current range by measuring at min and max duty cycles
    MeasurementData minCurrentData = takeMeasurement(minDutyCycle, STABILIZATION_DELAY_MS);
    processThermistorData(minCurrentData, "Estimating Min Current");
    float minCurrent = minCurrentData.current;

    MeasurementData maxCurrentData = takeMeasurement(MAX_DUTY_CYCLE, STABILIZATION_DELAY_MS);
    processThermistorData(maxCurrentData, "Estimating Max Current");
    float maxCurrent = maxCurrentData.current;

    if (maxCurrent <= minCurrent) {
        Serial.println("Warning: Maximum current is not greater than minimum current. Cannot ensure linear spacing. Falling back to a simpler approach.");
        // Fallback to a simpler linear duty cycle spacing (modified from original)
        int highDc = MAX_DUTY_CYCLE;
        int lowDc = minDutyCycle;
        int dutyCycleStep = (MAX_DUTY_CYCLE - minDutyCycle) / numPairs;
        for (int i = 0; i < numPairs; ++i) {
            if (highDc < lowDc) break;
            MeasurementData lowData = takeMeasurement(lowDc, STABILIZATION_DELAY_MS);
            processThermistorData(lowData, "Generating Pairs (Fallback)");
            // if (tft) {
            //     tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            //     tft.printf("progress: %.1f ", ((float)i / numPairs) * 100.0);
            // }
            if (lowData.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < highDc) {
                int adjustedLowDc = lowDc;
                for (int j = 0; j < 10; ++j) {
                    adjustedLowDc += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                    if (adjustedLowDc > MAX_DUTY_CYCLE) break;
                    MeasurementData checkData = takeMeasurement(adjustedLowDc, STABILIZATION_DELAY_MS);
                    if (checkData.current >= MEASURABLE_CURRENT_THRESHOLD) {
                        lowDc = adjustedLowDc;
                        Serial.printf("Adjusting low duty cycle to %d due to low current (fallback).\n", lowDc);
                        break;
                    }
                }
                if (lowData.current < MEASURABLE_CURRENT_THRESHOLD) {
                    Serial.println("Warning: Could not adjust low duty cycle to achieve measurable current (fallback).");
                }
            } else if (lowDc > MAX_DUTY_CYCLE) {
                Serial.println("Warning: Low duty cycle exceeded maximum value (fallback).");
                break;
            }
            pairs.push_back({lowDc, highDc});
            highDc -= dutyCycleStep;
            lowDc += dutyCycleStep; // Increment lowDc as well for a more even spread
            if (highDc < minDutyCycle) highDc = minDutyCycle + 1;
            if (lowDc > MAX_DUTY_CYCLE) lowDc = MAX_DUTY_CYCLE - 1;
        }
        Serial.printf("Generated %zu duty cycle pairs (fallback).\n", pairs.size());
        return pairs;
    }

    float totalCurrentRange = maxCurrent - minCurrent;
    float desiredCurrentIncrement = totalCurrentRange / numPairs;

    int lowDc = minDutyCycle;
    int previousHighDc = MAX_DUTY_CYCLE;

    for (int i = 0; i < numPairs; ++i) {
        float targetHighCurrent = maxCurrent - (i * desiredCurrentIncrement);
        int bestHighDc = -1;
        float minCurrentDifference = 1e9; // Initialize with a large value

        int lowBound = minDutyCycle;
        int highBound = previousHighDc;

        while (lowBound <= highBound) {
            int midDc = lowBound + (highBound - lowBound) / 2;
            MeasurementData midData = takeMeasurement(midDc, STABILIZATION_PAIRS_FIND_DELAY_MS);
            processThermistorData(midData, "Searching High DC (Binary)");
            float currentDifference = std::abs(midData.current - targetHighCurrent);

            if (currentDifference < minCurrentDifference) {
                minCurrentDifference = currentDifference;
                bestHighDc = midDc;
            }

            if (midData.current > targetHighCurrent) {
                highBound = midDc - 1; // Search in the lower half
            } else {
                lowBound = midDc + 1; // Search in the upper half
            }
        }

        int currentHighDc = bestHighDc != -1 ? bestHighDc : previousHighDc;
        if (currentHighDc < minDutyCycle) currentHighDc = minDutyCycle;

        // Adjust lowDc (same logic as before)
        MeasurementData lowData = takeMeasurement(lowDc, STABILIZATION_DELAY_MS);
        processThermistorData(lowData, "Generating Pairs");
        // if (tft) {
        //     tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
        //     tft.printf("progress: %.1f ", ((float)i / numPairs) * 100.0);
        // }

        if (lowData.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < currentHighDc) {
            int adjustedLowDc = lowDc;
            for (int j = 0; j < 5; ++j) {
                adjustedLowDc += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                if (adjustedLowDc > MAX_DUTY_CYCLE) break;
                MeasurementData checkData = takeMeasurement(adjustedLowDc, STABILIZATION_DELAY_MS);
                if (checkData.current >= MEASURABLE_CURRENT_THRESHOLD) {
                    lowDc = adjustedLowDc;
                    Serial.printf("Adjusting low duty cycle to %d due to low current.\n", lowDc);
                    break;
                }
            }
            if (lowData.current < MEASURABLE_CURRENT_THRESHOLD) {
                Serial.println("Warning: Could not adjust low duty cycle to achieve measurable current.");
                //break; // Consider if breaking here is appropriate
            }
        } else if (lowDc > MAX_DUTY_CYCLE) {
            Serial.println("Warning: Low duty cycle exceeded maximum value.");
            break;
        }

        if (currentHighDc < lowDc) {
            currentHighDc = lowDc + 1;
            break;
        }; // Ensure highDc is not lower than lowDc

        pairs.push_back({lowDc, currentHighDc});
        previousHighDc = currentHighDc - 1; // Start the next search from slightly below the current highDc

        // Optional: Print debug information
        // MeasurementData actualHighData = takeMeasurement(currentHighDc,STABILIZATION_DELAY_MS);
        // Serial.printf("Pair %d: lowDc=%d, highDc=%d, targetCurrent=%.2f, actualCurrent=%.2f\n",
        //                 i, lowDc, currentHighDc, targetHighCurrent, actualHighData.current);

        if (previousHighDc < minDutyCycle) break;
    }

    Serial.printf("Generated %zu duty cycle pairs (for linear current spacing using binary search).\n", pairs.size());
    return pairs;
}




// Step 3: Measure internal resistance using loaded/unloaded method
void measureInternalResistanceLoadedUnloaded(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& voltagesLoaded, std::vector<float>& currentsLoaded, std::vector<float>& dutyCycles) {
    Serial.println("\n--- Measuring Internal Resistance (Loaded/Unloaded) ---");
    for (const auto& pair : dutyCyclePairs) {
        int dc = pair.second; // Use the high duty cycle for loaded measurement here
        Serial.printf("--- Duty Cycle (Loaded/Unloaded): %d ---\n", dc);

        // Measure voltage with duty cycle applied (loaded)
        MeasurementData loadedData = takeMeasurement(dc,STABILIZATION_DELAY_MS);
        processThermistorData(loadedData, "Rint L/UL");

        voltagesLoaded.push_back(loadedData.voltage);
        currentsLoaded.push_back(loadedData.current);
        dutyCycles.push_back(static_cast<float>(dc));

        Serial.printf("Duty Cycle ON (%d): Voltage: %.3f V, Current: %.3f A\n", dc, loadedData.voltage, loadedData.current);

        // Measure voltage with duty cycle off (unloaded) immediately after
        MeasurementData unloadedData = getUnloadedVoltageMeasurement();
        Serial.printf("Duty Cycle OFF: Voltage: %.3f V, Current: %.3f A\n", unloadedData.voltage, unloadedData.current);

        // Calculate internal resistance for this step using the immediately following unloaded voltage
        if (loadedData.current > 0.01f) {
            float internalResistance = (unloadedData.voltage - loadedData.voltage) / loadedData.current;
            storeResistanceData(loadedData.current, std::abs(internalResistance), internalResistanceData, resistanceDataCount);
            Serial.printf("Calculated Internal Resistance (Loaded-Unloaded): %.3f Ohm\n", std::abs(internalResistance));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            tft.printf("(L/UL): %.3f ", std::abs(internalResistance));
        } else {
            Serial.println("Warning: Current is too low to reliably calculate internal resistance (Loaded-Unloaded).");
            storeResistanceData(loadedData.current, -1.0f, internalResistanceData, resistanceDataCount); // Indicate invalid
        }
        Serial.println("---------------------------\n");
    }
}

// Step 4: Measure internal resistance using duty cycle pairs
void measureInternalResistancePairs(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& consecutiveInternalResistances) {
    Serial.println("\n--- Measuring Internal Resistance using Duty Cycle Pairs ---");
    for (const auto& pair : dutyCyclePairs) {
        int dcLow = pair.first;
        int dcHigh = pair.second;

        Serial.printf("--- Duty Cycle Pair: Low=%d, High=%d ---\n", dcLow, dcHigh);

        // Measure voltage and current at low duty cycle
        MeasurementData lowData = takeMeasurement(dcLow,STABILIZATION_DELAY_MS);
        Serial.printf("Duty Cycle Low (%d): Voltage: %.3f V, Current: %.3f A\n", dcLow, lowData.voltage, lowData.current);

        // Measure voltage and current at high duty cycle
        MeasurementData highData = takeMeasurement(dcHigh,STABILIZATION_DELAY_MS);
        processThermistorData(highData, "Rint Pair");
        Serial.printf("Duty Cycle High (%d): Voltage: %.3f V, Current: %.3f A\n", dcHigh, highData.voltage, highData.current);

        // Calculate internal resistance using the pair
        if (highData.current > lowData.current + MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
            float internalResistanceConsecutive = (lowData.voltage - highData.voltage) / (highData.current - lowData.current);
            consecutiveInternalResistances.push_back(std::abs(internalResistanceConsecutive));
            storeResistanceData(highData.current, std::abs(internalResistanceConsecutive), internalResistanceDataPairs, resistanceDataCountPairs);
            Serial.printf("Calculated Internal Resistance (Pair): %.3f Ohm\n", std::abs(internalResistanceConsecutive));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 50);
            tft.printf("(Pair): %.3f ", std::abs(internalResistanceConsecutive));
        } else {
            Serial.println("Warning: Current difference is too small to reliably calculate internal resistance (Pair).");
            consecutiveInternalResistances.push_back(-1.0f); // Indicate invalid
            storeResistanceData(highData.current, -1.0f, internalResistanceDataPairs, resistanceDataCountPairs);
        }
        Serial.println("---------------------------\n");
    }
}


// Step 5: Calculate average internal resistance from pairs
float calculateAverageInternalResistance(const std::vector<float>& resistances) {
    float sum = 0;
    int count = 0;
    for (float rint : resistances) {
        if (rint > MIN_VALID_RESISTANCE) {
            sum += rint;
            count++;
        }
    }
    if (count > 0) {
        float average = sum / count;
        Serial.printf("\nAverage Internal Resistance (Pairs): %.3f Ohm (%d valid points)\n", average, count);
        return average;
    } else {
        Serial.println("\nNo valid internal resistance measurements from pairs.");
        return -1.0f;
    }
}

// Step 6: Perform linear regression on loaded data
void performLinearRegression(const std::vector<float>& voltages, const std::vector<float>& currents) {
    if (voltages.size() >= 2) {
        Serial.println("Calculating overall internal resistance using linear regression (Loaded Data)...");

        double sumI = std::accumulate(currents.begin(), currents.end(), 0.0);
        double sumV = std::accumulate(voltages.begin(), voltages.end(), 0.0);
        double sumII = std::inner_product(currents.begin(), currents.end(), currents.begin(), 0.0);
        double sumIV = 0.0;
        for (size_t i = 0; i < currents.size(); ++i) {
            sumIV += currents[i] * voltages[i];
        }

        int n = voltages.size();
        double denominator = (n * sumII - sumI * sumI);

        if (std::abs(denominator) > 1e-9) {
            double overallInternalResistance = (n * sumIV - sumI * sumV) / denominator;
            double openCircuitVoltage = (sumV - overallInternalResistance * sumI) / n;

            regressedInternalResistance = overallInternalResistance;
            Serial.printf("Calculated Overall Internal Resistance (Linear Regression on Loaded Data): %.3f Ohm\n", std::abs(overallInternalResistance));
            Serial.printf("Estimated Open Circuit Voltage: %.3f V\n", openCircuitVoltage);
        } else {
            Serial.println("Error: Could not calculate overall internal resistance (division by zero in regression).");
        }
    } else {
        Serial.println("Not enough data points to perform overall linear regression on loaded data.");
    }
}

// ---------------measure Rint function itself

void measureInternalResistance() {
    if (!isMeasuringResistance) return;

    Serial.println("Starting improved internal resistance measurement...");

    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;

    std::vector<float> voltagesLoaded;
    std::vector<float> currentsLoaded;
    std::vector<float> dutyCycles;
    std::vector<float> consecutiveInternalResistances;
    std::vector<float> unloadedVoltagesHistory;
    std::vector<unsigned long> unloadedVoltageTimestamps;

    // Measure initial unloaded voltage
    MeasurementData initialUnloaded = getUnloadedVoltageMeasurement();
    unloadedVoltagesHistory.push_back(initialUnloaded.voltage);
    unloadedVoltageTimestamps.push_back(initialUnloaded.timestamp);
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);

    // Find minimal duty cycle
    int minDutyCycle = findMinimalDutyCycle();
    if (minDutyCycle == 0) {
        return;
    }

    // Generate duty cycle pairs
    std::vector<std::pair<int, int>> dutyCyclePairs = generateDutyCyclePairs(minDutyCycle);

    // Measure internal resistance using loaded/unloaded method
    measureInternalResistanceLoadedUnloaded(dutyCyclePairs, voltagesLoaded, currentsLoaded, dutyCycles);

    // Measure internal resistance using duty cycle pairs
    measureInternalResistancePairs(dutyCyclePairs, consecutiveInternalResistances);

    // Measure final unloaded voltage
    MeasurementData finalUnloaded = getUnloadedVoltageMeasurement();
    unloadedVoltagesHistory.push_back(finalUnloaded.voltage);
    unloadedVoltageTimestamps.push_back(finalUnloaded.timestamp);
    Serial.printf("Final Unloaded Voltage: %.3f V\n", finalUnloaded.voltage);

    // Sort the collected data points by current
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    // Calculate average internal resistance from consecutive steps (pairs)
    calculateAverageInternalResistance(consecutiveInternalResistances);

    // Optional: Perform Linear Regression on the loaded voltage and current data
    performLinearRegression(voltagesLoaded, currentsLoaded);

    // Measurement complete
    Serial.printf("Internal resistance measurement complete. %d loaded/unloaded points, %d pair points collected.\n", resistanceDataCount, resistanceDataCountPairs);
    isMeasuringResistance = false; // Only measure once (adjust logic as needed)
}



// --------------------draw Rint graph

void displayInternalResistanceGraph() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    // --- Process the first resistance dataset (internalResistanceData) ---
    std::vector<std::pair<float, float>> validResistanceData;
    if (internalResistanceData != nullptr) { // Add null check for safety, although the problem states it's never null
        for (int i = 0; i < resistanceDataCount; ++i) {
            if (internalResistanceData[i][1] != -1.0f) {
                validResistanceData.push_back({internalResistanceData[i][0], internalResistanceData[i][1]});
            }
        }
    }

    // --- Process the second internal resistance dataset (internalResistanceDataPairs) ---
    std::vector<std::pair<float, float>> validInternalResistancePairs;
    if (internalResistanceDataPairs != nullptr) { // Add null check for safety
        for (int i = 0; i < resistanceDataCountPairs; ++i) {
            if (internalResistanceDataPairs[i][1] != -1.0f) {
                validInternalResistancePairs.push_back({internalResistanceDataPairs[i][0], internalResistanceDataPairs[i][1]});
            }
        }
    }

    if (validResistanceData.size() < 2 && validInternalResistancePairs.size() < 2) {
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.println("Not enough valid data");
        return;
    }

    // Find min/max current and resistance from both valid datasets
    float minCurrent = std::numeric_limits<float>::max();
    float maxCurrent = std::numeric_limits<float>::min();
    float minResistance = std::numeric_limits<float>::max();
    float maxResistance = std::numeric_limits<float>::min();

    bool dataFound = false;

    for (const auto& dataPoint : validResistanceData) {
        minCurrent = std::min(minCurrent, dataPoint.first);
        maxCurrent = std::max(maxCurrent, dataPoint.first);
        minResistance = std::min(minResistance, dataPoint.second);
        maxResistance = std::max(maxResistance, dataPoint.second);
        dataFound = true;
    }

    for (const auto& dataPoint : validInternalResistancePairs) {
        minCurrent = std::min(minCurrent, dataPoint.first);
        maxCurrent = std::max(maxCurrent, dataPoint.first);
        minResistance = std::min(minResistance, dataPoint.second);
        maxResistance = std::max(maxResistance, dataPoint.second);
        dataFound = true;
    }

    if (!dataFound) {
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.println("No valid data to plot");
        return;
    }

    // Add some padding for better visualization
    float currentRange = maxCurrent - minCurrent;
    float resistanceRange = maxResistance - minResistance;
    if (currentRange > 0) {
        //minCurrent -= currentRange * 0.1f;
        //maxCurrent += currentRange * 0.1f;
    } else {
        // Handle cases with zero current range
        if (maxCurrent == minCurrent) {
            minCurrent -= 0.1f;
            maxCurrent += 0.1f;
        }
    }
    if (resistanceRange > 0) {
        minResistance -= resistanceRange * 0.01f;
        maxResistance += resistanceRange * 0.01f;
    } else {
        // Handle cases with zero resistance range
        if (maxResistance == minResistance) {
            minResistance -= 0.1f;
            maxResistance += 0.1f;
        }
    }

    // Ensure min is not greater than max
    if (minCurrent > maxCurrent) std::swap(minCurrent, maxCurrent);
    if (minResistance > maxResistance) std::swap(minResistance, maxResistance);

    // Define graph boundaries
    int graphXStart = PLOT_X_START + 40;
    int graphYStart = PLOT_Y_START + 20;
    int graphXEnd = PLOT_X_START + PLOT_WIDTH - 20;
    int graphYEnd = PLOT_Y_START + PLOT_HEIGHT - 40;
    int graphWidth = graphXEnd - graphXStart;
    int graphHeight = graphYEnd - graphYStart;

    // Draw axes
    tft.drawLine(graphXStart, graphYEnd, graphXEnd, graphYEnd, PLOT_X_AXIS_COLOR); // X-axis
    tft.drawLine(graphXStart, graphYEnd, graphXStart, graphYStart, PLOT_Y_AXIS_COLOR); // Y-axis

    // Plot the first resistance dataset (internalResistanceData)
    tft.setTextColor(GRAPH_COLOR_RESISTANCE);
    if (validResistanceData.size() >= 2) {
        for (size_t i = 0; i < validResistanceData.size() - 1; ++i) {
            float current1 = validResistanceData[i].first;
            float resistance1 = validResistanceData[i].second;
            float current2 = validResistanceData[i + 1].first;
            float resistance2 = validResistanceData[i + 1].second;

            int x1 = mapf(current1, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y1 = mapf(resistance1, minResistance, maxResistance, graphYEnd, graphYStart); // Inverted Y-axis

            int x2 = mapf(current2, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y2 = mapf(resistance2, minResistance, maxResistance, graphYEnd, graphYStart); // Inverted Y-axis

            tft.drawLine(x1, y1, x2, y2, GRAPH_COLOR_RESISTANCE);
            //tft.fillCircle(x1, y1, 2, GRAPH_COLOR_RESISTANCE);
            if (i == validResistanceData.size() - 2) {
                //tft.fillCircle(x2, y2, 2, GRAPH_COLOR_RESISTANCE);
            }
        }
    }

    // Plot the second internal resistance dataset (internalResistanceDataPairs)
    tft.setTextColor(GRAPH_COLOR_RESISTANCE_PAIR);
    if (validInternalResistancePairs.size() >= 2) {
        for (size_t i = 0; i < validInternalResistancePairs.size() - 1; ++i) {
            float current1 = validInternalResistancePairs[i].first;
            float resistance1 = validInternalResistancePairs[i].second;
            float current2 = validInternalResistancePairs[i + 1].first;
            float resistance2 = validInternalResistancePairs[i + 1].second;

            int x1 = mapf(current1, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y1 = mapf(resistance1, minResistance, maxResistance, graphYEnd, graphYStart); // Inverted Y-axis

            int x2 = mapf(current2, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y2 = mapf(resistance2, minResistance, maxResistance, graphYEnd, graphYStart); // Inverted Y-axis

            tft.drawLine(x1, y1, x2, y2, GRAPH_COLOR_RESISTANCE_PAIR);
            //tft.fillCircle(x1, y1, 2, GRAPH_COLOR_RESISTANCE_PAIR);
            if (i == validInternalResistancePairs.size() - 2) {
                //tft.fillCircle(x2, y2, 2, GRAPH_COLOR_RESISTANCE_PAIR);
            }
        }
    }

    // Add axis labels
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(graphXStart + graphWidth / 2 - 30, graphYEnd + 10);
    tft.println("Current (A)");

    tft.setCursor(PLOT_X_START + 5, graphYStart + graphHeight / 2 - 5);
    tft.print("R");
    //  tft.print((char)937); //ohm

    // Display min/max values for reference
    tft.setCursor(graphXEnd - 50, graphYEnd + 10);
    tft.printf("%.2fA", maxCurrent);
    tft.setCursor(graphXStart + 5, graphYEnd + 10);
    tft.printf("%.2fA", minCurrent);

    tft.setCursor(graphXStart - 30, graphYStart - 10);
    tft.printf("%.2f", maxResistance);
    tft.setCursor(graphXStart - 30, graphYEnd - 10);
    tft.printf("%.2f", minResistance);

    // --- Add the overall internal resistance line and label ---
    if (regressedInternalResistance >= minResistance && regressedInternalResistance <= maxResistance) {
        int overallResistanceY = mapf(regressedInternalResistance, minResistance, maxResistance, graphYEnd, graphYStart);

        // Draw a horizontal line
        tft.drawLine(graphXStart, overallResistanceY, graphXEnd, overallResistanceY, TFT_WHITE);

        // Add a label for the line
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Rint: %.2f", regressedInternalResistance);
        tft.setCursor(graphXEnd - 50, overallResistanceY - 20); // Adjust position as needed
        tft.print(buffer);
        //  tft.print((char)937); //ohm
    } else if (regressedInternalResistance < minResistance && dataFound) {
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(graphXEnd - 50, graphYEnd - 10); // Adjust position as needed
        tft.printf("R_int < %.2f", minResistance);
    } else if (regressedInternalResistance > maxResistance && dataFound) {
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(graphXEnd - 50, graphYStart + 10); // Adjust position as needed
        tft.printf("R_int > %.2f", maxResistance);
    }

    // --- Add a legend ---
    tft.setTextSize(1);
    tft.setTextColor(GRAPH_COLOR_RESISTANCE);
    tft.setCursor(graphXStart + 10, PLOT_Y_START + 10);
    tft.print("R_int(1)");

    tft.setTextColor(GRAPH_COLOR_RESISTANCE_PAIR);
    tft.setCursor(graphXStart + 10, PLOT_Y_START + 20);
    tft.print("R_int(2)");
}


// Function to read and retrieve thermistor, voltage, and current data
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    while (thermistorSensor.isLocked()) {
        yield();
    } // Wait for thermistor task to update data
    temp1 = sht4Sensor.getTemperature(); // Thermistor 1 (Top) now comes from SHT4x
    temp2 = thermistorSensor.getTemperature2();
    tempDiff = thermistorSensor.getDifference();
    t1_millivolts = thermistorSensor.getRawMillivolts1();
    voltage = voltage_mv / 1000.0f; // Convert mV to Volts
    current = current_ma / 1000.0f; // Convert mA to Amps
}

// Function to print thermistor, voltage, and current data to the serial monitor
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current) {
    Serial.print("Thermistor 1 (Top, SHT4x): ");
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.3f °C", temp1);
    Serial.print(", Thermistor 2 (Bottom): ");
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.3f °C", temp2);
    Serial.print(", Diff (T1-T2): ");
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.3f °C", tempDiff);
    Serial.printf(", Voltage : %.3f V", voltage);
    Serial.printf(", Current : %.3f A", current);
    Serial.println(" °C");
}

// Function to update the temperature, voltage, and current history arrays for plotting
void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current) {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        temp1_values[i] = temp1_values[i + 1];
        temp2_values[i] = temp2_values[i + 1];
        diff_values[i] = diff_values[i + 1];
        voltage_values[i] = voltage_values[i + 1]; // Update voltage history
        current_values[i] = current_values[i + 1];     // Update current history
    }
    temp1_values[PLOT_WIDTH - 1] = temp1;
    temp2_values[PLOT_WIDTH - 1] = temp2;
    diff_values[PLOT_WIDTH - 1] = tempDiff;
    voltage_values[PLOT_WIDTH - 1] = voltage; // Add new voltage reading
    current_values[PLOT_WIDTH - 1] = current;     // Add new current reading
}

// Function to clear the plot area and draw the zero line
void prepareTemperaturePlot() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    tft.drawFastHLine(PLOT_X_START, PLOT_Y_START + (PLOT_HEIGHT / 2), PLOT_WIDTH, PLOT_ZERO_COLOR);
}

// Function to plot the temperature, voltage, and current data on the TFT display
void plotTemperatureData() {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(temp1_values[i]) && !isnan(temp1_values[i + 1])) {
            int y1_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp1_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y1_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp1_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y1_prev, PLOT_X_START + i + 1, y1_current, GRAPH_COLOR_1);
        }
        if (!isnan(temp2_values[i]) && !isnan(temp2_values[i + 1])) {
            int y2_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp2_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y2_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp2_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y2_prev, PLOT_X_START + i + 1, y2_current, GRAPH_COLOR_2);
        }
        if (!isnan(diff_values[i]) && !isnan(diff_values[i + 1])) {
            int y_diff_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(diff_values[i], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            int y_diff_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(diff_values[i + 1], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_diff_prev, PLOT_X_START + i + 1, y_diff_current, GRAPH_COLOR_DIFF);
        }
        if (!isnan(voltage_values[i]) && !isnan(voltage_values[i + 1])) {
            int y_voltage_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(voltage_values[i], MIN_VOLTAGE, MAX_VOLTAGE, 0, PLOT_HEIGHT);
            int y_voltage_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(voltage_values[i + 1], MIN_VOLTAGE, MAX_VOLTAGE, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_voltage_prev, PLOT_X_START + i + 1, y_voltage_current, GRAPH_COLOR_VOLTAGE);
        }
        if (!isnan(current_values[i]) && !isnan(current_values[i + 1])) {
            int y_current_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(current_values[i], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            int y_current_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(current_values[i + 1], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_current_prev, PLOT_X_START + i + 1, y_current_current, GRAPH_COLOR_CURRENT);
        }
    }
}

// Function to display current temperature, voltage, and current values and other labels on the TFT display
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    int label_line_height = 8;

    // T1 Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START);
    tft.print("T1: ");
    if (!isnan(temp1)) {
        tft.printf("%.2f C", temp1);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);
    tft.print(" R");

    // Voltage Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START);
    tft.print("V: ");
    if (!isnan(voltage)) {
        tft.printf("%.3f V", voltage);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_VOLTAGE, TFT_BLACK);
    tft.print(" Y");

    // Current Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START+label_line_height * 1);
    tft.print("I: ");
    if (!isnan(current)) {
        tft.printf("%.3f A", current);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_CURRENT, TFT_BLACK);
    tft.print(" M");

    // VCC Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 0);
    tft.print("VCC:");
    if (!isnan(thermistorSensor.getVCC())) {
        tft.printf("%.2f mV", thermistorSensor.getVCC());
    } else {
        tft.print("Error");
    }

    // T2 Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START + label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("T2: ");
    if (!isnan(temp2)) {
        tft.printf("%.2f C", temp2);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);
    tft.print(" G");

    // Raw Millivolts Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 1);
    tft.print("mV :");
    if (!isnan(t1_millivolts)) {
        tft.printf("%.2f mV", t1_millivolts);
    } else {
        tft.print("Error");
    }

     // duty cycle label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 2);
    tft.print("%  :");
    if (!isnan(dutyCycle)) {
        tft.printf("%u  ", dutyCycle);
    } else {
        tft.print("Error");
    }

    // Diff Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("dT: ");
    if (!isnan(tempDiff)) {
        tft.printf("%.2f C", tempDiff);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);
    tft.print(" B");
}

void loop() {
    double temp1, temp2, tempDiff;
    float t1_millivolts;
    float voltage;
    float current;

    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    printThermistorSerial(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    updateTemperatureHistory(temp1, temp2, tempDiff, voltage, current);
    prepareTemperaturePlot();
    plotTemperatureData();
    displayTemperatureLabels(temp1, temp2, tempDiff, t1_millivolts, voltage, current);

    //controlPWM(); // Control the PWM on pin 19

    //if (!isMeasuringResistance) {
    //    displayInternalResistanceGraph(); // Display the internal resistance graph after measurement
    //}

    // Handle IR remote
    if (IrReceiver.decode()) {
        handleIRCommand();
        IrReceiver.resume();
    }

    delay(500); // Reduced delay to make PWM more responsive, PWM timing is handled in controlPWM
}

void handleIRCommand() {
      if (IrReceiver.decodedIRData.protocol == SAMSUNG) {
            if (IrReceiver.decodedIRData.address == 0x7) {
      //debug        
            Serial.print(F("Command 0x"));
            Serial.println(IrReceiver.decodedIRData.command, HEX);
        switch(IrReceiver.decodedIRData.command) {
        case RemoteKeys::KEY_PLAY:{        
        isMeasuringResistance = true; 
        measureInternalResistance();
        break;
        }
        case RemoteKeys::KEY_INFO:{        
          if (!isMeasuringResistance) {
          displayInternalResistanceGraph(); // Display the internal resistance graph after measurement
          delay(10000); // wait 10 seconds
        }
        break;
        }    
           
               
        }// switch
            }
      }
}
