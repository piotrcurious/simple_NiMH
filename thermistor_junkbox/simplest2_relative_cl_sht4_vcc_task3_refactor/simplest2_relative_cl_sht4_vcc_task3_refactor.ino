#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <cmath>     // For math functions like log, exp, pow, isnan
#include <limits>    // For std::numeric_limits

#include "SHT4xSensor.h"
#include "ThermistorSensor.h"

// --- Configuration Section ---
// Define TFT display pins
#define TFT_CS      15  // Chip Select pin for TFT CS
#define TFT_DC      2   // Data Command pin for TFT DC
#define TFT_RST     4   // Reset pin for TFT RST (or -1 if not used, connect to ESP32 enable pin)

// Define Thermistor pins
#define THERMISTOR_PIN_1 36 // Analog pin for Thermistor 1
//double THERMISTOR_1_OFFSET = -960.0;  // zero offset to cancel out slight thermistor differences
double THERMISTOR_1_OFFSET = 13.0;  // zero offset to cancel out slight thermistor differences


#define THERMISTOR_PIN_2 39 // Analog pin for Thermistor 2
#define THERMISTOR_PIN_3 34 // Analog pin for Thermistor 3
//#define THERMISTOR_PIN_4 35 // Analog pin for Thermistor 4
#define THERMISTOR_VCC_PIN 35 // analog pin to measure VCC for thermistors

// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const double MIN_TEMP      = 15.0;
const double MAX_TEMP      = 30.0;
const double MIN_DIFF_TEMP = -1.0; // Difference range -3 to +3
const double MAX_DIFF_TEMP = 1.0;

// Plotting parameters - Maximized graph and bottom labels - ADJUST IF NEEDED
#define PLOT_WIDTH    320       // Maximize width (almost full screen)
#define PLOT_HEIGHT   (216 - 3)   // Maximize height (leaving space for labels at bottom)
#define PLOT_X_START  0         // Adjusted start position for wider graph
#define PLOT_Y_START  0         // Adjusted start position from top

// Plot colors - ADJUST IF DESIRED
#define PLOT_X_AXIS_COLOR TFT_WHITE
#define PLOT_Y_AXIS_COLOR TFT_WHITE
#define PLOT_ZERO_COLOR   0x62ec

#define GRAPH_COLOR_1     TFT_RED
#define GRAPH_COLOR_2     TFT_GREEN
#define GRAPH_COLOR_DIFF  TFT_BLUE

// Label area at the bottom
#define LABEL_Y_START     PLOT_Y_START + PLOT_HEIGHT + 3 // Start labels below graph
#define LABEL_TEXT_SIZE   1

// --- Global variables ---
TFT_eSPI tft = TFT_eSPI(); // TFT_eSPI instance

// --- Temperature sensor objects
SHT4xSensor sht4Sensor;
ThermistorSensor thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_PIN_2, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);

// Temperature reading arrays for plotting
float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// --- Function Declarations ---
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map

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
        vTaskDelay(10);
    }
}

void task_readThermistor(void* parameter) {
    while (true) {
        while (sht4Sensor.isLocked()) {
            vTaskDelay(1);
        }; // if lock is set, wait for sht4 task to update the data
        thermistorSensor.read(sht4Sensor.getTemperature());
        vTaskDelay(10);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize TFT
    tft.init();
    tft.setRotation(1); // Adjust rotation as needed

    // Clear screen and set background color
    tft.fillScreen(TFT_BLACK);

    // Initialize temperature arrays to a default value (e.g., 25 degrees)
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
    }

    sht4Sensor.begin(); // Initialize SHT4x sensor
    thermistorSensor.begin(); // Initialize thermistor sensor

    xTaskCreate(task_readSHT4x,      "SHT4",  2048, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 1024, NULL, 1, NULL); // Create the new thermistor reading task

    Serial.println("TFT and Temperature Sensor Example Started");
}

// Function to read and retrieve thermistor data
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts) {
    while (thermistorSensor.isLocked()) {
        yield();
    } // Wait for thermistor task to update data
    temp1 = sht4Sensor.getTemperature(); // Thermistor 1 (Top) now comes from SHT4x
    temp2 = thermistorSensor.getTemperature2();
    tempDiff = thermistorSensor.getDifference();
    t1_millivolts = thermistorSensor.getRawMillivolts1();
}

// Function to print thermistor data to the serial monitor
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts) {
    Serial.print("Thermistor 1 (Top, SHT4x): ");
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.2f °C", temp1);
    Serial.print(", Thermistor 2 (Bottom): ");
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.2f °C", temp2);
    Serial.print(", Diff (T1-T2): ");
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.2f °C", tempDiff);
    Serial.println(" °C");
}

// Function to update the temperature history arrays for plotting
void updateTemperatureHistory(double temp1, double temp2, double tempDiff) {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        temp1_values[i] = temp1_values[i + 1];
        temp2_values[i] = temp2_values[i + 1];
        diff_values[i] = diff_values[i + 1];
    }
    temp1_values[PLOT_WIDTH - 1] = temp1;
    temp2_values[PLOT_WIDTH - 1] = temp2;
    diff_values[PLOT_WIDTH - 1] = tempDiff;
}

// Function to clear the plot area and draw the zero line
void prepareTemperaturePlot() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    tft.drawFastHLine(PLOT_X_START, PLOT_Y_START + (PLOT_HEIGHT / 2), PLOT_WIDTH, PLOT_ZERO_COLOR);
}

// Function to plot the temperature data on the TFT display
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
    }
}

// Function to display current temperature values and other labels on the TFT display
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    int label_line_height = 8;

    // T1 Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START);
    tft.print("T1 (Top)    : ");
    if (!isnan(temp1)) {
        tft.printf("%.2f C", temp1);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);
    tft.print(" [R]");

    // VCC Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 160, LABEL_Y_START);
    tft.print("VCC:");
    if (!isnan(thermistorSensor.getVCC())) {
        tft.printf("%.2f mV", thermistorSensor.getVCC());
    } else {
        tft.print("Error");
    }

    // T2 Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START + label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("T2 (Bottom) : ");
    if (!isnan(temp2)) {
        tft.printf("%.2f C", temp2);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);
    tft.print(" [G]");

    // Raw Millivolts Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 160, LABEL_Y_START + label_line_height);
    tft.print("mV :");
    if (!isnan(t1_millivolts)) {
        tft.printf("%.2f mV", t1_millivolts);
    } else {
        tft.print("Error");
    }

    // Diff Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("Diff (T1-T2): ");
    if (!isnan(tempDiff)) {
        tft.printf("%.2f C", tempDiff);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);
    tft.print(" [B]");
}

void loop() {
    double temp1, temp2, tempDiff;
    float t1_millivolts;

    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts);
    printThermistorSerial(temp1, temp2, tempDiff, t1_millivolts);
    updateTemperatureHistory(temp1, temp2, tempDiff);
    prepareTemperaturePlot();
    plotTemperatureData();
    displayTemperatureLabels(temp1, temp2, tempDiff, t1_millivolts);

    delay(500);
}
