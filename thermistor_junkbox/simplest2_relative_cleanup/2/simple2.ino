#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <cmath>
#include <limits>

// --- Configuration Section ---
#define TFT_CS      15
#define TFT_DC      2
#define TFT_RST     4

#define THERMISTOR_PIN_1 34
#define THERMISTOR_PIN_2 35

const double THERMISTORNOMINAL = 10000.0;
const double TEMPERATURENOMINAL = 25.0;
const double BCOEFFICIENT = 3950.0;
const double SERIESRESISTOR = 10000.0;
const double VCC_MILLIVOLTS = 3300.0;

const double MIN_TEMP = 20.0;
const double MAX_TEMP = 30.0;
const double MIN_DIFF_TEMP = -3.0;
const double MAX_DIFF_TEMP = 3.0;

#define PLOT_WIDTH     320
#define PLOT_HEIGHT    (216-3)
#define PLOT_X_START   0
#define PLOT_Y_START   0

#define PLOT_X_AXIS_COLOR TFT_WHITE
#define PLOT_Y_AXIS_COLOR TFT_WHITE
#define PLOT_ZERO_COLOR   0x62ec

#define GRAPH_COLOR_1     TFT_RED
#define GRAPH_COLOR_2     TFT_GREEN
#define GRAPH_COLOR_DIFF  TFT_BLUE

#define LABEL_Y_START     PLOT_Y_START + PLOT_HEIGHT + 3
#define LABEL_TEXT_SIZE   1

// --- Global Variables ---
TFT_eSPI tft = TFT_eSPI();

float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// --- Function Declarations ---
double readThermistor(int pin, int numSamples);
double resistanceToTemperature(double resistance);
double temperatureToResistance(double temperatureCelsius);
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples);
float mapf(float value, float in_min, float in_max, float out_min, float out_max);

// --- Helper Functions ---

// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Helper function to calculate resistance from ADC reading
double calculateResistance(double analogValue, double referenceResistance) {
    if (VCC_MILLIVOLTS - analogValue < 1.0) {
        return std::numeric_limits<double>::quiet_NaN(); // Avoid division by zero or very small values
    }
    return (analogValue * referenceResistance) / (VCC_MILLIVOLTS - analogValue);
}

// Helper function to convert resistance to temperature (Celsius)
double resistanceToTemperature(double resistance) {
    double steinhart = log(resistance / THERMISTORNOMINAL);
    steinhart /= BCOEFFICIENT;
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
    steinhart = 1.0 / steinhart; // Kelvin
    return steinhart - 273.15; // Celsius
}

// Helper function to convert temperature (Celsius) to resistance
double temperatureToResistance(double temperatureCelsius) {
    double temperatureKelvin = temperatureCelsius + 273.15;
    return THERMISTORNOMINAL * exp(BCOEFFICIENT * (1.0 / temperatureKelvin - 1.0 / (TEMPERATURENOMINAL + 273.15)));
}

// Function to read thermistor temperature with averaging and error handling
double readThermistor(int pin, int numSamples = 16) {
    if (numSamples <= 0) numSamples = 1;

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(pin);
        if (analogValue == 0) return std::numeric_limits<double>::quiet_NaN();
        sumAnalogValues += analogValue;
    }
    double averageAnalogValue = sumAnalogValues / numSamples;
    double resistance = calculateResistance(averageAnalogValue, SERIESRESISTOR);
    return resistanceToTemperature(resistance);
}

// Function to calculate temperature difference between two thermistors
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples = 16) {
    if (numSamples <= 0) numSamples = 1;

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(bottomThermistorPin);
        if (analogValue == 0) return std::numeric_limits<double>::quiet_NaN();
        sumAnalogValues += analogValue;
    }
    double averageAnalogValue = sumAnalogValues / numSamples;

    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);
    double bottomThermistorResistance = calculateResistance(averageAnalogValue, topThermistorResistance);

    if (std::isnan(bottomThermistorResistance)) return std::numeric_limits<double>::quiet_NaN();
    
    double bottomTemperatureCelsius = resistanceToTemperature(bottomThermistorResistance);
    return topThermistorTemperatureCelsius - bottomTemperatureCelsius;
}

// --- Setup and Loop ---

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = temp2_values[i] = diff_values[i] = NAN;
    }

    Serial.println("TFT and Thermistor Example Started");
}

void loop() {
    // Read temperatures from thermistors
    double temp1 = readThermistor(THERMISTOR_PIN_2); // Top thermistor
    double tempDiff = readThermistorDifference(THERMISTOR_PIN_1, temp1); // Temperature difference

    Serial.printf("T1: %.2f °C, Diff: %.2f °C\n", temp1, tempDiff);

    // Update arrays for plotting
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        temp1_values[i] = temp1_values[i + 1];
        diff_values[i] = diff_values[i + 1];
    }
    temp1_values[PLOT_WIDTH - 1] = temp1;
    diff_values[PLOT_WIDTH - 1] = tempDiff;

    // Clear plot area and redraw graph
    tft.fillRect(PLOT_X_START , PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(temp1_values[i]) && !isnan(temp1_values[i + 1])) {
            int y_prev = PLOT_Y_START + PLOT_HEIGHT - mapf(temp1_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y_current = PLOT_Y_START + PLOT_HEIGHT - mapf(temp1_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_prev, PLOT_X_START + i + 1, y_current, GRAPH_COLOR_1);
        }
        if (!isnan(diff_values[i]) && !isnan(diff_values[i + 1])) {
            int y_diff_prev = PLOT_Y_START + PLOT_HEIGHT - mapf(diff_values[i], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            int y_diff_current = PLOT_Y_START + PLOT_HEIGHT - mapf(diff_values[i + 1], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_diff_prev, PLOT_X_START + i + 1, y_diff_current, GRAPH_COLOR_DIFF);
        }
    }

    delay(50); // Sampling rate adjustment
}
