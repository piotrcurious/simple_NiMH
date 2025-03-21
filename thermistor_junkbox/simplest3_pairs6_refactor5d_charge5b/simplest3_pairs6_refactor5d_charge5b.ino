#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <IRremote.h>
#include <cmath>     // For math functions like log, exp, pow, isnan
#include <limits>    // For std::numeric_limits
#include <vector>    // For dynamic arrays
#include <numeric>
#include <algorithm>

#include <Arduino.h>
#include "analog.h"


// Define additional constants for charging (adjust as needed)
//#define STABILIZATION_DELAY_MS 100    // Delay after setting a duty cycle to allow voltage stabilization
#define CHARGING_UPDATE_INTERVAL_MS 2000  // Interval between re-evaluation during charging
#define PWM_MAX 255                   // Maximum PWM value (for analogWrite)
#define TEST_DUTY_CYCLE 128           // Initial test duty cycle value
#define VOLTAGE_TOLERANCE 0.05        // Voltage tolerance (in volts) for convergence
#define TOTAL_TIMEOUT (20UL * 60 * 60 * 1000) // 20h total timeout after charge stops for safety reasons

//end charge




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


/*
// Structure to hold measurement data
struct {
    float voltage;
    float current;
    double temp1;
    double temp2;
    double tempDiff;
    float t1_millivolts;
    int dutyCycle;
    unsigned long timestamp;
} MeasurementData;
*/


#include "SHT4xSensor.h"
#include "ThermistorSensor.h"


// --- Configuration Section ---
// Define TFT display pins
#define TFT_CS          15  // Chip Select pin for TFT CS
#define TFT_DC          2   // Data Command pin for TFT DC
#define TFT_RST         4   // Reset pin for TFT RST (or -1 if not used, connect to ESP32 enable pin)
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Define Thermistor pins
#define THERMISTOR_PIN_1 36 // Analog pin for Thermistor 1
#define THERMISTOR_PIN_1_ATTENUATION ADC_ATTEN_DB_11
#define THERMISTOR_PIN_1_OVERSAMPLING 16

//double THERMISTOR_1_OFFSET = -960.0;  // zero offset to cancel out slight thermistor differences
double THERMISTOR_1_OFFSET = 0.0;  // zero offset to cancel out slight thermistor differences and adc nonlinearity

#define THERMISTOR_VCC_PIN 35 // analog pin to measure VCC for thermistors
#define THERMISTOR_VCC_ATTENUATION ADC_ATTEN_DB_11
#define THERMISTOR_VCC_OVERSAMPLING 16


// Define Voltage Readout Pin
#define VOLTAGE_READ_PIN 39    // analog pin to measure battery voltage
#define VOLTAGE_ATTENUATION ADC_ATTEN_DB_11
#define VOLTAGE_OVERSAMPLING 16

//#define MAIN_VCC_RATIO 4.125 // multiplier to determine VCC from thermistor divider VCC
#define MAIN_VCC_RATIO 2.0 // multiplier to determine VCC from thermistor divider VCC

volatile uint32_t voltage_last_time;
volatile uint32_t voltage_update_interval = 250;

volatile float mAh_charged = 0.0f;
volatile bool resetAh = false; // Semaphore flag to reset mAh
volatile uint32_t mAh_last_time = 0; // Keep track of the last time mAh was updated

// Define Current Shunt Pin and Value
#define CURRENT_SHUNT_PIN 34
#define CURRENT_SHUNT_ATTENUATION ADC_ATTEN_DB_0
#define CURRENT_SHUNT_OVERSAMPLING 16

#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 75 // offset of current pin

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
    KEY_POWER = 0xE6,

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
    KEY_PRIME_VIDEO = 0xF4,
    KEY_GUIDE       = 0x4F

  };
}


// --- Global variables ---



// Global variables for charging state
bool isCharging = false;
unsigned long lastChargeEvaluationTime = 0;
float mhElectrodeVoltage = 0.0f;
int chargingDutyCycle = 0;

// Structure to hold MH electrode voltage measurement data
struct MHElectrodeData {
  float unloadedVoltage;
  float loadedVoltage;
 // float mhElectrodeRatio;
  float targetVoltage; // Added field for the target voltage
  float voltageDifference; // Added field for the difference between loaded and target voltage
  float current;
  uint32_t dutyCycle; // duty cycle
};



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
volatile float voltage_mv = 1000.0f;

// Global variable for current readout
volatile float current_ma = 0.0f;

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
    mAh_last_time = millis(); // Initialize the last update time for mAh

    while (true) {
        uint32_t current_time = millis();

        // Wait for SHT4 sensor data if locked (assuming this is handled by a mutex or similar)
         while (sht4Sensor.isLocked()) {
             vTaskDelay(10);
         };

        // Read thermistor data (assuming this also updates VCC)
         thermistorSensor.read(sht4Sensor.getTemperature());

        // Read current and store in global variable
        int task_current_numSamples = 256;
        double sumAnalogValuesCurrent = 0;
        for (int i = 0; i < task_current_numSamples; ++i) {
            uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN, CURRENT_SHUNT_ATTENUATION, CURRENT_SHUNT_OVERSAMPLING);
            sumAnalogValuesCurrent += analogValue;
        }
        double voltageAcrossShunt = (sumAnalogValuesCurrent / task_current_numSamples) - CURRENT_SHUNT_PIN_ZERO_OFFSET;
        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE); // Convert to mA (if resistance was in Ohms)

        // Read voltage periodically
        if ((current_time - voltage_last_time) > voltage_update_interval) {
            int task_voltage_numSamples = 256;
            double sumAnalogValuesVoltage = 0;
            for (int i = 0; i < task_voltage_numSamples; ++i) {
                uint32_t analogValue = analogReadMillivolts(VOLTAGE_READ_PIN, VOLTAGE_ATTENUATION, VOLTAGE_OVERSAMPLING);
                sumAnalogValuesVoltage += analogValue;
            }
            // Assuming thermistorSensor.getVCC() provides the ESP32 VCC in mV
            // Adjust MAIN_VCC_RATIO based on your voltage divider if needed
            // The line below assumes you are measuring the battery voltage directly (after any potential divider)
            voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - (sumAnalogValuesVoltage / task_voltage_numSamples);
            voltage_last_time = current_time;
        }

        // Measure mAh
        uint32_t time_elapsed = current_time - mAh_last_time; // Time elapsed in milliseconds

        // Convert current to Amps and time to hours
   //     double current_amps = current_ma ;
        double time_elapsed_hours = (double)time_elapsed / (1000.0 * 3600.0);

        // Calculate mAh charged during this interval
        mAh_charged += (current_ma ) * time_elapsed_hours;

        mAh_last_time = current_time;

        // Check and reset mAh if the flag is set
        if (resetAh) {
            mAh_charged = 0.0f;
            resetAh = false; // Clear the flag after resetting
            Serial.println("mAh counter reset.");
        }

        vTaskDelay(50); // Adjust delay as needed
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

    // analogSetPinAttenuation(VOLTAGE_READ_PIN, ADC_11db) ; // Set attenuation to 11dB
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
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.02f;
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
    tft.fillRect(0, DUTY_CYCLE_BAR_Y , SCREEN_WIDTH-DUTY_CYCLE_BAR_START_X, SCREEN_HEIGHT - DUTY_CYCLE_BAR_Y, BLACK);

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
            processThermistorData(midData, "Binary Searching High DC");
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
             tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
             tft.printf("progress: %.0f ", ((float)i / numPairs) * 100.0);
       
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


namespace { // Anonymous namespace to limit scope of helper functions

// Structure to represent a data point (current, resistance)
struct DataPoint {
    float current;
    float resistance;
};

// Function to extract valid resistance data from a raw data array
std::vector<DataPoint> extractValidData(float (*rawData)[2], int dataCount) {
    std::vector<DataPoint> validData;
    if (rawData != nullptr) {
        for (int i = 0; i < dataCount; ++i) {
            if (rawData[i][1] != -1.0f) {
                validData.push_back({rawData[i][0], rawData[i][1]});
            }
        }
    }
    return validData;
}

// Function to find the minimum and maximum current and resistance from a vector of data points
void findMinMax(const std::vector<DataPoint>& dataPoints, float& minCurrent, float& maxCurrent, float& minResistance, float& maxResistance) {
    for (const auto& dataPoint : dataPoints) {
        minCurrent = std::min(minCurrent, dataPoint.current);
        maxCurrent = std::max(maxCurrent, dataPoint.current);
        minResistance = std::min(minResistance, dataPoint.resistance);
        maxResistance = std::max(maxResistance, dataPoint.resistance);
    }
}

// Function to plot a single resistance dataset
void plotResistanceData(const std::vector<DataPoint>& dataPoints, uint16_t color, float minCurrent, float maxCurrent, float minResistance, float maxResistance, int graphXStart, int graphYStart, int graphXEnd, int graphYEnd) {
    tft.setTextColor(color);
    if (dataPoints.size() >= 2) {
        for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
            float current1 = dataPoints[i].current;
            float resistance1 = dataPoints[i].resistance;
            float current2 = dataPoints[i + 1].current;
            float resistance2 = dataPoints[i + 1].resistance;

            int x1 = mapf(current1, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y1 = mapf(resistance1, minResistance, maxResistance, graphYEnd, graphYStart); // Inverted Y-axis

            int x2 = mapf(current2, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y2 = mapf(resistance2, minResistance, maxResistance, graphYEnd, graphYStart); // Inverted Y-axis

            tft.drawLine(x1, y1, x2, y2, color);
            // tft.fillCircle(x1, y1, 2, color);
            // if (i == dataPoints.size() - 2) {
            //     tft.fillCircle(x2, y2, 2, color);
            // }
        }
    }
}

} // namespace

void displayInternalResistanceGraph() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    // --- Process resistance datasets ---
    std::vector<DataPoint> validResistanceData = extractValidData(internalResistanceData, resistanceDataCount);
    std::vector<DataPoint> validInternalResistancePairs = extractValidData(internalResistanceDataPairs, resistanceDataCountPairs);

    // Check if there is enough data to plot
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

    if (!validResistanceData.empty()) {
        findMinMax(validResistanceData, minCurrent, maxCurrent, minResistance, maxResistance);
        dataFound = true;
    }
    if (!validInternalResistancePairs.empty()) {
        findMinMax(validInternalResistancePairs, minCurrent, maxCurrent, minResistance, maxResistance);
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
    auto adjustRange =[](float& minVal, float& maxVal, float paddingFactor) {
        float range = maxVal - minVal;
        if (range > 0) {
            minVal -= range * paddingFactor;
            maxVal += range * paddingFactor;
        } else if (maxVal == minVal) {
            minVal -= 0.1f;
            maxVal += 0.1f;
        }
    };

    
    adjustRange(minCurrent, maxCurrent, 0.1f);
    adjustRange(minResistance, maxResistance, 0.01f);

    // Ensure min is not greater than max (shouldn't happen with the padding logic, but for safety)
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

    // Plot the resistance datasets
    plotResistanceData(validResistanceData, GRAPH_COLOR_RESISTANCE, minCurrent, maxCurrent, minResistance, maxResistance, graphXStart, graphYStart, graphXEnd, graphYEnd);
    plotResistanceData(validInternalResistancePairs, GRAPH_COLOR_RESISTANCE_PAIR, minCurrent, maxCurrent, minResistance, maxResistance, graphXStart, graphYStart, graphXEnd, graphYEnd);

    // Add axis labels
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(graphXStart + graphWidth / 2 - 30, graphYEnd + 10);
    tft.println("Current (A)");

    tft.setCursor(PLOT_X_START + 5, graphYStart + graphHeight / 2 - 5);
    tft.print("R");
    // tft.print((char)937); //ohm

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
    if (dataFound) {
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
            // tft.print((char)937); //ohm
        } else if (regressedInternalResistance < minResistance) {
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(graphXEnd - 50, graphYEnd - 10); // Adjust position as needed
            tft.printf("R_int < %.2f", minResistance);
        } else if (regressedInternalResistance > maxResistance) {
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(graphXEnd - 50, graphYStart + 10); // Adjust position as needed
            tft.printf("R_int > %.2f", maxResistance);
        }
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


// charge
//      processThermistorData(minCurrentData, "CHARGING");

 // Constants for battery charging
const float MAX_TEMP_DIFF_THRESHOLD = 0.7f;        // Maximum temperature difference in Celsius before stopping charge
const float MH_ELECTRODE_RATIO = 0.5f;             // Target ratio for MH electrode voltage (half value)
const int CHARGE_EVALUATION_INTERVAL_MS = 120000;   // Re-evaluate charging parameters every 2 minutes
const int CHARGE_CURRENT_STEP = 5;                 // Step size for PWM duty cycle adjustment
const int MAX_CHARGE_DUTY_CYCLE = 230;             // Maximum duty cycle for charging
const int MIN_CHARGE_DUTY_CYCLE = 10;              // Minimum duty cycle for charging

// Helper function to perform binary search to find the insertion point or closest element
int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;
    int low = 0;
    int high = count - 1;
    int mid;

    while (low <= high) {
        mid = low + (high - low) / 2;
        if (data[mid][0] == targetCurrent) {
            return mid;
        } else if (data[mid][0] < targetCurrent) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    // If not found, return the index where it should be inserted or the closest index
    if (low < count) {
        if (high >= 0 && (targetCurrent - data[high][0] < data[low][0] - targetCurrent)) {
            return high;
        } else {
            return low;
        }
    } else {
        return high;
    }
}

// Helper function to store or average resistance data with overflow handling
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (resistance <= 0) return; // Ignore invalid or negative resistance

    if (count < MAX_RESISTANCE_POINTS) {
        data[count][0] = current;
        data[count][1] = resistance;
        count++;
    } else {
        int closestIndex = findClosestIndex(data, count, current);
        if (closestIndex >= 0 && closestIndex < MAX_RESISTANCE_POINTS) {
            // Average the new data with the closest point
            data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
            data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
        }
    }
}

/**
 * Measures the MH electrode voltage by comparing unloaded vs loaded voltages
 * Returns the data structure with measurements
 */
MHElectrodeData measureMHElectrodeVoltage(int testDutyCycle) {
    MHElectrodeData data;

    // Measure unloaded voltage
    MeasurementData unloadedData = getUnloadedVoltageMeasurement();
    processThermistorData(unloadedData, "MH idle");
    data.unloadedVoltage = unloadedData.voltage;

    // Measure loaded voltage with the specified duty cycle
    MeasurementData loadedData = takeMeasurement(testDutyCycle, STABILIZATION_DELAY_MS);
    processThermistorData(loadedData, "MH loaded");
    data.loadedVoltage = loadedData.voltage;
    data.current = loadedData.current;
    data.dutyCycle = testDutyCycle;

    // Calculate the target voltage
    data.targetVoltage = data.unloadedVoltage + (data.loadedVoltage - data.unloadedVoltage) * MH_ELECTRODE_RATIO;

    // Calculate the difference between loaded and target voltage
    data.voltageDifference = data.loadedVoltage - data.targetVoltage;

    return data;
}

/**
 * Finds the optimal charging duty cycle that maintains the MH electrode voltage
 * at the target voltage (half way between unloaded and loaded).
 */
int findOptimalChargingDutyCycle() {
    Serial.println("Finding optimal charging duty cycle...");

    int optimalDutyCycle = MIN_CHARGE_DUTY_CYCLE;
    float closestVoltageDifference = 1000.0f; // Initialize with a large value
    float targetVoltage = 0.0f; // Will be updated in the loop

    // Start with a binary search approach to narrow down the range
    int lowDC = MIN_CHARGE_DUTY_CYCLE;
    int highDC = MAX_CHARGE_DUTY_CYCLE;

    // First, measure the unloaded voltage once
    MeasurementData initialUnloadedData = getUnloadedVoltageMeasurement();
    processThermistorData(initialUnloadedData, "MH idle (initial)");
    float initialUnloadedVoltage = initialUnloadedData.voltage;

    // Recalculate target voltage based on the *initial* unloaded voltage and max duty cycle possible
    MHElectrodeData dataHigh = measureMHElectrodeVoltage(highDC);
    targetVoltage = initialUnloadedVoltage + (dataHigh.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

    MHElectrodeData previousData;
    bool firstIteration = true;

    while (highDC - lowDC > CHARGE_CURRENT_STEP * 2) {
        int midDC = (lowDC + highDC) / 2;

        MHElectrodeData currentData = measureMHElectrodeVoltage(midDC);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            storeOrAverageResistanceData(currentData.current, std::abs(internalResistanceLU), internalResistanceData, resistanceDataCount);
        }

        // Calculate and store internal resistance (Pair)
        if (!firstIteration && (std::abs(currentData.current - previousData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR)) {
            float internalResistancePair = (previousData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - previousData.current);
            storeOrAverageResistanceData(currentData.current, std::abs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
        }

        // Sort the data after each addition (optional, but keeps data sorted)
        bubbleSort(internalResistanceData, resistanceDataCount);
        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

        float currentVoltageDifference = abs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Binary Search - Duty Cycle: %d, Unloaded: %.3fV, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      midDC, currentData.unloadedVoltage, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentData.loadedVoltage < targetVoltage) {
            // Loaded voltage is below target, increase duty cycle to increase load (and voltage)
            lowDC = midDC;
        } else {
            // Loaded voltage is above target, decrease duty cycle to decrease load (and voltage)
            highDC = midDC;
        }

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = midDC;
        }

        previousData = currentData;
        firstIteration = false;
    }

    // Fine tune with small steps around the approximate value
    for (int dc = max(MIN_CHARGE_DUTY_CYCLE, optimalDutyCycle - CHARGE_CURRENT_STEP * 3);
         dc <= min(MAX_CHARGE_DUTY_CYCLE, optimalDutyCycle + CHARGE_CURRENT_STEP * 3);
         dc += CHARGE_CURRENT_STEP) {

        MHElectrodeData currentData = measureMHElectrodeVoltage(dc);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            storeOrAverageResistanceData(currentData.current, std::abs(internalResistanceLU), internalResistanceData, resistanceDataCount);
        }

        // Calculate and store internal resistance (Pair)
        if (std::abs(currentData.current - previousData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
            float internalResistancePair = (previousData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - previousData.current);
            storeOrAverageResistanceData(currentData.current, std::abs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
        }

        // Sort the data after each addition (optional, but keeps data sorted)
        bubbleSort(internalResistanceData, resistanceDataCount);
        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

        float currentVoltageDifference = abs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Fine-tuning - Duty Cycle: %d, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      dc, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = dc;
        }
        previousData = currentData;
    }

    // Final measurement at the optimal duty cycle
    MHElectrodeData finalData = measureMHElectrodeVoltage(optimalDutyCycle);
    targetVoltage = initialUnloadedVoltage + (finalData.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

    Serial.printf("Optimal charging duty cycle found: %d (loaded: %.3fV, target: %.3fV, diff: %.3fV)\n",
                  optimalDutyCycle, finalData.loadedVoltage, targetVoltage, abs(finalData.loadedVoltage - targetVoltage));

    return optimalDutyCycle;
}

/**
 * Main charging function - starts, monitors, and stops battery charging
 * Returns true if charging is in progress, false if charging has stopped
 */
uint32_t chargingStartTime = 0 ; 
bool chargeBattery() {
    // If not currently charging, start the charging process
    if (!isCharging) {
        Serial.println("Starting battery charging process...");
        isCharging = true;
        lastChargeEvaluationTime = millis();

        // Find the initial optimal charging duty cycle
        chargingDutyCycle = findOptimalChargingDutyCycle();

        // Apply the charging current
        dutyCycle = chargingDutyCycle;
        analogWrite(pwmPin, chargingDutyCycle);

        // Store initial data for monitoring
        MeasurementData initialData;
        getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff,
                              initialData.t1_millivolts, initialData.voltage, initialData.current);
        processThermistorData(initialData, "intial readings");
        Serial.printf("Charging started - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                      chargingDutyCycle, initialData.current, initialData.temp1, initialData.temp2, initialData.tempDiff);

        return true;
    }


 // Check for total charging timeout (10 hours = 10 * 60 * 60 * 1000 milliseconds)
    unsigned long currentTime = millis();
//    if (currentTime - chargingStartTime >= 10UL * 60 * 60 * 1000) {
    if (currentTime - chargingStartTime >= TOTAL_TIMEOUT) {

        Serial.println("Charging timeout, stopping charging for safety reasons. ");

        // Stop charging
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        isCharging = false;

        return false;
    }

    
  
  // Get current temperature and voltage readings
//  float temp1, temp2, tempDiff, t1_millivolts, voltage, current;
    double temp1, temp2, tempDiff;
    float t1_millivolts;
    float voltage;
    float current;
  
  getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
  // Check if temperature difference exceeds threshold - stop charging if it does
  if (tempDiff > MAX_TEMP_DIFF_THRESHOLD) {
    Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C), stopping charging\n", 
                  tempDiff, MAX_TEMP_DIFF_THRESHOLD);
    
    // Stop charging
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    isCharging = false;
    
    return false;
  }
  
  // Periodically re-evaluate the MH electrode voltage and adjust charging current
  currentTime = millis();
  if (currentTime - lastChargeEvaluationTime > CHARGE_EVALUATION_INTERVAL_MS) {
    Serial.println("Re-evaluating charging parameters...");
    
    // Temporarily pause charging to measure unloaded voltage
    analogWrite(pwmPin, 0);
    delay(UNLOADED_VOLTAGE_DELAY_MS);
    
    // Find the optimal charging duty cycle again
    chargingDutyCycle = findOptimalChargingDutyCycle();
    
    // Apply the updated charging current
    dutyCycle = chargingDutyCycle;
    analogWrite(pwmPin, chargingDutyCycle);
    
    lastChargeEvaluationTime = currentTime;
    
    Serial.printf("Charging parameters updated - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                  chargingDutyCycle, current, temp1, temp2, tempDiff);
  }
  
  // Display charging status on TFT
//  tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
//  tft.printf("DC: %d, I: %.2fA", chargingDutyCycle, current);
//  tft.setCursor(14*10, PLOT_Y_START + PLOT_HEIGHT + 40);
//  tft.printf("T1: %.2fC, T2: %.2fC, dT: %.2fC", temp1, temp2, tempDiff);
  
  return true; // Charging is still in progress
}

/**
 * Start charging with a single call
 */
void startCharging() {
  if (!isCharging) {
    Serial.println("Initiating battery charging...");
    tft.setTextColor(TFT_RED,TFT_BLACK);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("CHARGING           ");
    isCharging = true;
    chargeBattery();
  } else {
    Serial.println("Charging already in progress");
  }
}

/**
 * Stop charging with a single call
 */
void stopCharging() {
  if (isCharging) {
    Serial.println("Manually stopping charging");
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    isCharging = false;
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("STOPPED             ");
  }
}

/**
 * Loop function to be called in the main loop to monitor and maintain charging
 */
void handleBatteryCharging() {
  if (isCharging) {
    // Update charging status and check if it's still charging
    if (!chargeBattery()) {
      // Charging has stopped
      tft.setTextColor(TFT_GREEN);
      tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
      tft.printf("COMPLETE  ");
      
    }
  }
}

//end charge


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
    tft.setTextColor(TFT_WHITE,TFT_BLACK); //with wipe underneath
    tft.setCursor(19*10, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.print(mAh_charged,3);
    tft.print(" mAh");
    //controlPWM(); // Control the PWM on pin 19

    //if (!isMeasuringResistance) {
    //    displayInternalResistanceGraph(); // Display the internal resistance graph after measurement
    //}

    // Handle IR remote
    if (IrReceiver.decode()) {
        handleIRCommand();
        IrReceiver.resume();
    }

    handleBatteryCharging();

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

        case RemoteKeys::KEY_POWER:{        
        resetAh = true; // reset mAh counter
        startCharging(); // slow charge battery
        break;
        }    
  
               
        }// switch
            }//address 7
      }//samsung
}// handle IR
