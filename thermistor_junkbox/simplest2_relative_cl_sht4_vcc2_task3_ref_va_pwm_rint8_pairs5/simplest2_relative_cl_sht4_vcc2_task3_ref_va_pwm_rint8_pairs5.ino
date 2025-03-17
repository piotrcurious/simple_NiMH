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

// --- Configuration Section ---
// Define TFT display pins
#define TFT_CS          15  // Chip Select pin for TFT CS
#define TFT_DC          2   // Data Command pin for TFT DC
#define TFT_RST         4   // Reset pin for TFT RST (or -1 if not used, connect to ESP32 enable pin)

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
        double voltageAcrossShunt = analogReadMilliVolts(CURRENT_SHUNT_PIN) - CURRENT_SHUNT_PIN_ZERO_OFFSET ;
        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE); // Convert to mA
//  current_ma = voltageAcrossShunt ; // Convert to mA

    if ((current_time - voltage_last_time) > voltage_update_interval) {
        //analogWrite(pwmPin, 0); // set current to 0 to not interfere with voltage reading
        //vTaskDelay(100);
        voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - analogReadMilliVolts(VOLTAGE_READ_PIN);
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

void measureInternalResistance() {
    if (!isMeasuringResistance) return;

    Serial.println("Starting improved internal resistance measurement with consecutive steps...");

    resistanceDataCount = 0;

    std::vector<float> voltages_loaded;
    std::vector<float> currents_loaded;
    std::vector<float> voltages_unloaded;
    std::vector<float> duty_cycles;
    std::vector<float> internal_resistances_loaded_unloaded;
    std::vector<float> consecutive_internal_resistances; // Store results from consecutive steps
    std::vector<float> unloaded_voltages_history;
    std::vector<unsigned long> unloaded_voltage_timestamps;

    int numDutyCycles = MAX_RESISTANCE_POINTS;
    if (numDutyCycles < 2) {
        Serial.println("MAX_RESISTANCE_POINTS should be at least 2 for improved measurement.");
        return;
    }

    Serial.println("Collecting voltage and current data at different duty cycles...");

    // Generate duty cycles in an alternating pattern
    std::vector<int> duty_cycle_order;
    for (int i = 0; i < numDutyCycles; ++i) {
        duty_cycle_order.push_back(round(static_cast<float>(i) * 255.0f / (numDutyCycles - 1)));
    }
    std::sort(duty_cycle_order.begin(), duty_cycle_order.end()); // Ensure they are sorted first

    std::vector<int> truly_alternating_order;
    int left = 0;
    int right = duty_cycle_order.size() - 1;
    while (left <= right) {
        if (left == right) {
            truly_alternating_order.push_back(duty_cycle_order[left]);
            break;
        }
        truly_alternating_order.push_back(duty_cycle_order[left++]);
        truly_alternating_order.push_back(duty_cycle_order[right--]);
    }
    duty_cycle_order = truly_alternating_order;

    // Measure initial unloaded voltage
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    delay(2000);
    float initial_voltage_unloaded, initial_current_unloaded;
    double temp1_unloaded_initial, temp2_unloaded_initial, tempDiff_unloaded_initial;
    float t1_millivolts_unloaded_initial;
    getThermistorReadings(temp1_unloaded_initial, temp2_unloaded_initial, tempDiff_unloaded_initial, t1_millivolts_unloaded_initial, initial_voltage_unloaded, initial_current_unloaded);
    unloaded_voltages_history.push_back(initial_voltage_unloaded);
    unloaded_voltage_timestamps.push_back(millis());
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", initial_voltage_unloaded);

    for (size_t i = 0; i < duty_cycle_order.size(); ++i) {
        int dc = duty_cycle_order[i];
        Serial.printf("--- Duty Cycle Step (%d/%zu): %d ---\n", static_cast<int>(i) + 1, duty_cycle_order.size(), dc);

        // Measure voltage with duty cycle applied (loaded)
        dutyCycle = dc;
        analogWrite(pwmPin, dc);
        delay(2000); // Wait for current and voltage to stabilize under load

        double temp1_loaded, temp2_loaded, tempDiff_loaded;
        float t1_millivolts_loaded;
        float voltage_loaded, current_loaded;
        getThermistorReadings(temp1_loaded, temp2_loaded, tempDiff_loaded, t1_millivolts_loaded, voltage_loaded, current_loaded);
        printThermistorSerial(temp1_loaded, temp2_loaded, tempDiff_loaded, t1_millivolts_loaded, voltage_loaded, current_loaded);
        updateTemperatureHistory(temp1_loaded, temp2_loaded, tempDiff_loaded, voltage_loaded, current_loaded);
        prepareTemperaturePlot();
        plotTemperatureData();
        displayTemperatureLabels(temp1_loaded, temp2_loaded, tempDiff_loaded, t1_millivolts_loaded, voltage_loaded, current_loaded);

        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.print("Rint TEST ");

        voltages_loaded.push_back(voltage_loaded); // Store in Volts
        currents_loaded.push_back(current_loaded);
        duty_cycles.push_back(static_cast<float>(dc));

        Serial.printf("Duty Cycle ON (%d): Voltage: %.3f V, Current: %.3f A\n", dc, voltage_loaded, current_loaded);

        // Measure voltage with duty cycle off (unloaded) immediately after
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        delay(2000); // delay to make battery voltage return to no load state

        double temp1_unloaded, temp2_unloaded, tempDiff_unloaded;
        float t1_millivolts_unloaded;
        float voltage_unloaded, current_unloaded_temp; // Current should be close to zero
        getThermistorReadings(temp1_unloaded, temp2_unloaded, tempDiff_unloaded, t1_millivolts_unloaded, voltage_unloaded, current_unloaded_temp);
        printThermistorSerial(temp1_unloaded, temp2_unloaded, tempDiff_unloaded, t1_millivolts_unloaded, voltage_unloaded, current_unloaded_temp);

        voltages_unloaded.push_back(voltage_unloaded);
        unloaded_voltages_history.push_back(voltage_unloaded);
        unloaded_voltage_timestamps.push_back(millis());

        Serial.printf("Duty Cycle OFF: Voltage: %.3f V, Current: %.3f A\n", voltage_unloaded, current_unloaded_temp);

        // Calculate internal resistance for this step using the immediately following unloaded voltage
        if (current_loaded > 0.01f) { // Avoid division by very small numbers
            float internalResistance = (voltage_unloaded - voltage_loaded) / current_loaded;
            internal_resistances_loaded_unloaded.push_back(std::abs(internalResistance));

            if (resistanceDataCount < MAX_RESISTANCE_POINTS) {
                internalResistanceData[resistanceDataCount][0] = current_loaded; // Store current
                internalResistanceData[resistanceDataCount][1] = std::abs(internalResistance);
                resistanceDataCount++;
            }
            Serial.printf("Calculated Internal Resistance (Loaded-Unloaded, Step %zu): %.3f Ohm\n", i + 1, std::abs(internalResistance));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            tft.printf("(%.1f): %.3f ", (((float)(i + 1) / duty_cycle_order.size()) * 100.0), std::abs(internalResistance));
        } else {
            Serial.println("Warning: Current is too low to reliably calculate internal resistance for this step (Loaded-Unloaded).");
            if (resistanceDataCount < MAX_RESISTANCE_POINTS) {
                internalResistanceData[resistanceDataCount][0] = current_loaded;
                internalResistanceData[resistanceDataCount][1] = -1.0f; // Indicate invalid measurement
                resistanceDataCount++;
            }
            internal_resistances_loaded_unloaded.push_back(-1.0f); // Indicate invalid measurement
        }
        Serial.println("---------------------------\n");
    }



    resistanceDataCountPairs = 0;
    Serial.println("\n--- Starting Internal Resistance Measurement using Consecutive Steps ---");
    // Iterate through consecutive pairs of duty cycles
    for (size_t i = 0; i < duty_cycle_order.size() - 1; i=i+2) {
        int dc_low = duty_cycle_order[i];
        int dc_high = duty_cycle_order[i + 1];

        Serial.printf("--- Consecutive Duty Cycle Steps (%d/%zu): Low=%d, High=%d ---\n", static_cast<int>(i) + 1, duty_cycle_order.size() - 1, dc_low, dc_high);

        // Measure unloaded voltage before the consecutive steps
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        delay(1000); // Small delay for stability
        float voltage_unloaded_before, current_unloaded_before;
        double temp1_unloaded_before, temp2_unloaded_before, tempDiff_unloaded_before;
        float t1_millivolts_unloaded_before;
        getThermistorReadings(temp1_unloaded_before, temp2_unloaded_before, tempDiff_unloaded_before, t1_millivolts_unloaded_before, voltage_unloaded_before, current_unloaded_before);
        printThermistorSerial(temp1_unloaded_before, temp2_unloaded_before, tempDiff_unloaded_before, t1_millivolts_unloaded_before, voltage_unloaded_before, current_unloaded_before);
        updateTemperatureHistory(temp1_unloaded_before, temp2_unloaded_before, tempDiff_unloaded_before, voltage_unloaded_before, current_unloaded_before);
        prepareTemperaturePlot();
        plotTemperatureData();
        displayTemperatureLabels(temp1_unloaded_before, temp2_unloaded_before, tempDiff_unloaded_before, t1_millivolts_unloaded_before, voltage_unloaded_before, current_unloaded_before);

        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.print("Rint TEST2 ");

        
        Serial.printf("Unloaded Voltage (Before Consecutive Steps): %.3f V\n", voltage_unloaded_before);

        // Measure voltage and current at low duty cycle
        dutyCycle = dc_low;
        analogWrite(pwmPin, dc_low);
        delay(2000); // Wait for stabilization
        float voltage_low, current_low;
        double temp1_low, temp2_low, tempDiff_low;
        float t1_millivolts_low;
        getThermistorReadings(temp1_low, temp2_low, tempDiff_low, t1_millivolts_low, voltage_low, current_low);
        Serial.printf("Duty Cycle Low (%d): Voltage: %.3f V, Current: %.3f A\n", dc_low, voltage_low, current_low);

        // Measure voltage and current at high duty cycle
        dutyCycle = dc_high;
        analogWrite(pwmPin, dc_high);
        delay(2000); // Wait for stabilization
        float voltage_high, current_high;
        double temp1_high, temp2_high, tempDiff_high;
        float t1_millivolts_high;
        getThermistorReadings(temp1_high, temp2_high, tempDiff_high, t1_millivolts_high, voltage_high, current_high);
        Serial.printf("Duty Cycle High (%d): Voltage: %.3f V, Current: %.3f A\n", dc_high, voltage_high, current_high);

        // Calculate internal resistance using consecutive steps
        if (current_high > current_low + 0.01f) { // Ensure a meaningful current difference
            float internalResistanceConsecutive = (voltage_low - voltage_high) / (current_high - current_low);
            consecutive_internal_resistances.push_back(std::abs(internalResistanceConsecutive));

            // Store in internalResistanceData, marking it as from consecutive steps (optional)
            if (resistanceDataCountPairs < MAX_RESISTANCE_POINTS) {
//                internalResistanceDataPairs[resistanceDataCountPairs][0] = (current_low + current_high) / 2.0f; // Store average current
                internalResistanceDataPairs[resistanceDataCountPairs][0] =  current_high; // Store high current
                internalResistanceDataPairs[resistanceDataCountPairs][1] = std::abs(internalResistanceConsecutive);
                resistanceDataCountPairs++;
            }
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            tft.printf("(%.1f): %.3f ", (((float)(i + 1) / duty_cycle_order.size()) * 100.0), std::abs(internalResistanceConsecutive));
            Serial.printf("Calculated Internal Resistance (Consecutive Steps): %.3f Ohm\n", std::abs(internalResistanceConsecutive));
        } else {
            Serial.println("Warning: Current difference is too small to reliably calculate internal resistance (Consecutive Steps).");
            if (resistanceDataCountPairs < MAX_RESISTANCE_POINTS) {
//                internalResistanceDataPairs[resistanceDataCountPairs][0] = (current_low + current_high) / 2.0f;
                internalResistanceDataPairs[resistanceDataCountPairs][0] = current_high;
                internalResistanceDataPairs[resistanceDataCountPairs][1] = -1.0f; // Indicate invalid measurement from consecutive steps
                resistanceDataCountPairs++;
            }
            consecutive_internal_resistances.push_back(-1.0f); // Indicate invalid measurement
        }
        Serial.println("---------------------------\n");
    }

    // Measure final unloaded voltage
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    delay(2000);
    float final_voltage_unloaded, final_current_unloaded;
    double temp1_unloaded_final, temp2_unloaded_final, tempDiff_unloaded_final;
    float t1_millivolts_unloaded_final;
    getThermistorReadings(temp1_unloaded_final, temp2_unloaded_final, tempDiff_unloaded_final, t1_millivolts_unloaded_final, final_voltage_unloaded, final_current_unloaded);
    unloaded_voltages_history.push_back(final_voltage_unloaded);
    unloaded_voltage_timestamps.push_back(millis());
    Serial.printf("Final Unloaded Voltage: %.3f V\n", final_voltage_unloaded);

    // Sort the collected data points by current using Bubble Sort
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    // Calculate average internal resistance from consecutive steps
    float sum_consecutive_rint = 0;
    int valid_consecutive_count = 0;
    for (float rint : consecutive_internal_resistances) {
        if (rint > 0) { // Consider only valid (positive) resistance values
            sum_consecutive_rint += rint;
            valid_consecutive_count++;
        }
    }

    float average_consecutive_rint = -1.0f; // Default to -1.0f if no valid values
    if (valid_consecutive_count > 0) {
        average_consecutive_rint = sum_consecutive_rint / valid_consecutive_count;
        Serial.printf("\nAverage Internal Resistance (Consecutive Steps): %.3f Ohm (%d valid points)\n", average_consecutive_rint, valid_consecutive_count);
    } else {
        Serial.println("\nNo valid internal resistance measurements from consecutive steps.");
    }

    // Optional: Perform Linear Regression on the loaded voltage and current data
    if (voltages_loaded.size() >= 2) {
        Serial.println("Calculating overall internal resistance using linear regression (Loaded Data)...");

        double sum_I = std::accumulate(currents_loaded.begin(), currents_loaded.end(), 0.0);
        double sum_V = std::accumulate(voltages_loaded.begin(), voltages_loaded.end(), 0.0);
        double sum_II = std::inner_product(currents_loaded.begin(), currents_loaded.end(), currents_loaded.begin(), 0.0);
        double sum_IV = 0.0;
        for (size_t i = 0; i < currents_loaded.size(); ++i) {
            sum_IV += currents_loaded[i] * voltages_loaded[i];
        }

        int n = voltages_loaded.size();
        double denominator = (n * sum_II - sum_I * sum_I);

        if (std::abs(denominator) > 1e-9) {
            double overallInternalResistance = (n * sum_IV - sum_I * sum_V) / denominator;
            double openCircuitVoltage = (sum_V - overallInternalResistance * sum_I) / n;

            regressedInternalResistance = overallInternalResistance;
            Serial.printf("Calculated Overall Internal Resistance (Linear Regression on Loaded Data): %.3f Ohm\n", std::abs(overallInternalResistance));
            Serial.printf("Estimated Open Circuit Voltage: %.3f V\n", openCircuitVoltage);
        } else {
            Serial.println("Error: Could not calculate overall internal resistance (division by zero in regression).");
        }
    } else {
        Serial.println("Not enough data points to perform overall linear regression on loaded data.");
    }

    // You now have internal resistance calculated using both methods
    Serial.printf("Internal resistance measurement complete. %d data points collected.\n", resistanceDataCount);
    isMeasuringResistance = false; // Only measure once at startup for this example (you can change this logic)
}

void displayInternalResistanceGraph() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    // --- Process the first resistance dataset (internalResistanceData) ---
    std::vector<std::pair<float, float>> validResistanceData;
    for (int i = 0; i < resistanceDataCount; ++i) {
        if (internalResistanceData[i][1] != -1.0f) {
            validResistanceData.push_back({internalResistanceData[i][0], internalResistanceData[i][1]});
        }
    }

    // --- Process the second internal resistance dataset (internalResistanceDataPairs) ---
    std::vector<std::pair<float, float>> validInternalResistancePairs;
    if (internalResistanceDataPairs != nullptr) { // Check if the pointer is valid
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

    for (const auto& dataPoint : validResistanceData) {
        minCurrent = std::min(minCurrent, dataPoint.first);
        maxCurrent = std::max(maxCurrent, dataPoint.first);
        minResistance = std::min(minResistance, dataPoint.second);
        maxResistance = std::max(maxResistance, dataPoint.second);
    }

    for (const auto& dataPoint : validInternalResistancePairs) {
        minCurrent = std::min(minCurrent, dataPoint.first);
        maxCurrent = std::max(maxCurrent, dataPoint.first);
        minResistance = std::min(minResistance, dataPoint.second);
        maxResistance = std::max(maxResistance, dataPoint.second);
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

    // Plot the second internal resistance dataset (internalResistanceDataPairs)
    tft.setTextColor(GRAPH_COLOR_RESISTANCE_PAIR);
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
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.2f °C", temp1);
    Serial.print(", Thermistor 2 (Bottom): ");
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.2f °C", temp2);
    Serial.print(", Diff (T1-T2): ");
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.2f °C", tempDiff);
    Serial.printf(", Voltage : %.2f V", voltage);
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
        tft.printf("%.2f V", voltage);
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
