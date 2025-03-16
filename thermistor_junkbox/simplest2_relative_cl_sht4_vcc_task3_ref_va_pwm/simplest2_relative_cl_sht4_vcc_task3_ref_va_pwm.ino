#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <cmath>     // For math functions like log, exp, pow, isnan
#include <limits>    // For std::numeric_limits

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
double THERMISTOR_1_OFFSET = 11.0;  // zero offset to cancel out slight thermistor differences and adc nonlinearity

#define THERMISTOR_VCC_PIN 35 // analog pin to measure VCC for thermistors

// Define Voltage Readout Pin
#define VOLTAGE_READ_PIN 39    // analog pin to measure battery voltage
#define MAIN_VCC_RATIO 4.125 // multiplier to determine VCC from thermistor divider VCC
uint32_t voltage_last_time;
uint32_t voltage_update_interval = 250;

// Define Current Shunt Pin and Value
#define CURRENT_SHUNT_PIN 34
#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 140 // offset of current pin

// Define PWM Pin
#define PWM_PIN 19

// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const double MIN_TEMP            = 15.0;
const double MAX_TEMP            = 30.0;
const double MIN_DIFF_TEMP       = -1.0; // Difference range -3 to +3
const double MAX_DIFF_TEMP       = 1.0;

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

// Label area at the bottom
#define LABEL_Y_START       PLOT_Y_START + PLOT_HEIGHT + 3 // Start labels below graph
#define LABEL_TEXT_SIZE     1

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

// --- Function Declarations ---
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map
void controlPWM();
void setupPWM();

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
// no need repeat      //analogSetPinAttenuation(VOLTAGE_READ_PIN, ADC_11db); // Set attenuation to 11dB
//        voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - analogReadMilliVolts(VOLTAGE_READ_PIN);

        // Read current and store in global variable
        double voltageAcrossShunt = analogReadMilliVolts(CURRENT_SHUNT_PIN) - CURRENT_SHUNT_PIN_ZERO_OFFSET ;
        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE); // Convert to mA
//      current_ma = voltageAcrossShunt ; // Convert to mA 

    if ((current_time - voltage_last_time) > voltage_update_interval) {        
        analogWrite(pwmPin, 0); // set current to 0 to not interfere with voltage reading
        vTaskDelay(100);
        voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - analogReadMilliVolts(VOLTAGE_READ_PIN);
        analogWrite(pwmPin, dutyCycle);
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

    // Initialize temperature, voltage, and current arrays to a default value
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f; // Initialize voltage array
        current_values[i] = 0.0f;    // Initialize current array
    }

    sht4Sensor.begin(); // Initialize SHT4x sensor
    thermistorSensor.begin(); // Initialize thermistor sensor
    analogSetPinAttenuation(VOLTAGE_READ_PIN, ADC_11db) ; // Set attenuation to 11dB
    analogSetPinAttenuation(CURRENT_SHUNT_PIN, ADC_0db); // Set attenuation for current shunt pin

    xTaskCreate(task_readSHT4x,     "SHT4",  2048, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 2048, NULL, 1, NULL); // Create the new thermistor reading task

    setupPWM(); // Initialize PWM

    Serial.println("TFT and Temperature Sensor Example Started");
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
        current_values[i] = current_values[i + 1];    // Update current history
    }
    temp1_values[PLOT_WIDTH - 1] = temp1;
    temp2_values[PLOT_WIDTH - 1] = temp2;
    diff_values[PLOT_WIDTH - 1] = tempDiff;
    voltage_values[PLOT_WIDTH - 1] = voltage; // Add new voltage reading
    current_values[PLOT_WIDTH - 1] = current;    // Add new current reading
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

    controlPWM(); // Control the PWM on pin 19

    delay(500); // Reduced delay to make PWM more responsive, PWM timing is handled in controlPWM
}
