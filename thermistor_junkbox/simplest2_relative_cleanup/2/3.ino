#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <cmath>    // For math functions like log, exp, pow, isnan
#include <limits>   // For std::numeric_limits

// --- Configuration Section ---
// Define TFT display pins
#define TFT_CS      15  // Chip Select pin for TFT CS
#define TFT_DC      2   // Data Command pin for TFT DC
#define TFT_RST     4   // Reset pin for TFT RST (or -1 if not used, connect to ESP32 enable pin)

// Define Thermistor pins
#define THERMISTOR_PIN_1 34 // Analog pin for Thermistor 1
#define THERMISTOR_PIN_2 35 // Analog pin for Thermistor 2

// Thermistor parameters - ADJUST THESE BASED ON YOUR THERMISTOR DATASHEET
const double THERMISTOR_NOMINAL    = 10000.0;  // Resistance at 25 degrees C (Ohms)
const double TEMPERATURE_NOMINAL   = 25.0;     // Temperature for nominal resistance (degrees C)
const double B_COEFFICIENT         = 3950.0;   // Beta coefficient (from datasheet)
const double SERIES_RESISTOR       = 10000.0;  // Series resistor value in voltage divider circuit (Ohms)
const double VCC_MILLIVOLTS        = 3300.0;   // Supply voltage in millivolts

// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const double MIN_TEMP      = 20.0;
const double MAX_TEMP      = 30.0;
const double MIN_DIFF_TEMP = -3.0; // Difference range -3 to +3
const double MAX_DIFF_TEMP = 3.0;

// Plotting parameters
#define PLOT_WIDTH     320       // Maximize width (almost full screen)
#define PLOT_HEIGHT    213       // Maximize height (leaving space for labels at bottom)
#define PLOT_X_START   0         // Start position for graph
#define PLOT_Y_START   0         // Start position from top

// Plot colors
#define PLOT_X_AXIS_COLOR TFT_WHITE
#define PLOT_Y_AXIS_COLOR TFT_WHITE
#define PLOT_ZERO_COLOR   0x62EC  // Dim blue-green for zero line

#define GRAPH_COLOR_1     TFT_RED
#define GRAPH_COLOR_2     TFT_GREEN
#define GRAPH_COLOR_DIFF  TFT_BLUE

// Label area at the bottom
#define LABEL_Y_START     (PLOT_Y_START + PLOT_HEIGHT + 3) // Start labels below graph
#define LABEL_TEXT_SIZE   1
#define LABEL_LINE_HEIGHT 8  // Height per line of text with textSize=1

// Sampling configuration
#define DEFAULT_NUM_SAMPLES 16  // Default number of samples for readings
#define SAMPLE_DELAY        1   // Delay between samples in milliseconds
#define LOOP_DELAY          50  // Delay between loop iterations in milliseconds

// --- Global variables ---
TFT_eSPI tft = TFT_eSPI(); // TFT_eSPI instance

// Temperature reading arrays for plotting
float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// --- Function Declarations ---
double readThermistor(int pin, int numSamples = DEFAULT_NUM_SAMPLES);
double resistanceToTemperature(double resistance);
double temperatureToResistance(double temperatureCelsius);
double readThermistorRelative(int pin, double topThermistorTemperatureCelsius, int numSamples = DEFAULT_NUM_SAMPLES);
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples = DEFAULT_NUM_SAMPLES);
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map
void drawPlot();
void displayLabels(double temp1, double temp2, double tempDiff);

// --- Function Implementations ---

// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to read thermistor temperature with enhanced precision and error handling
double readThermistor(int pin, int numSamples) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;  
    for (int i = 0; i < numSamples; ++i) {  
        uint32_t analogValue = analogReadMilliVolts(pin);  
        if (analogValue == 0) {  
            // Handle potential zero reading - indicates error  
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN to indicate error  
        }  
        sumAnalogValues += analogValue;  
        if (numSamples > 1) delay(SAMPLE_DELAY); // Short delay between samples if multiple
    }  
    double averageAnalogValue = sumAnalogValues / numSamples;  

    // Convert average analog reading to resistance  
    double resistance = (SERIES_RESISTOR * averageAnalogValue) / (VCC_MILLIVOLTS - averageAnalogValue);  
    return resistanceToTemperature(resistance);
}

// Function to read relative thermistor using top thermistor as reference
double readThermistorRelative(int pin, double topThermistorTemperatureCelsius, int numSamples) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample
    if (isnan(topThermistorTemperatureCelsius)) {
        return std::numeric_limits<double>::quiet_NaN(); // Cannot calculate if reference temp is invalid
    }

    double sumAnalogValues = 0;  
    for (int i = 0; i < numSamples; ++i) {  
        uint32_t analogValue = analogReadMilliVolts(pin);  
        if (analogValue == 0) {  
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN to indicate error  
        }  
        sumAnalogValues += analogValue;  
        if (numSamples > 1) delay(SAMPLE_DELAY); // Short delay between samples if multiple
    }  
    double averageAnalogValue = sumAnalogValues / numSamples;  
    
    // Calculate resistance of the top thermistor from its temperature  
    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);  
    
    // Calculate bottom thermistor resistance using top thermistor as reference
    if (VCC_MILLIVOLTS - averageAnalogValue < 1.0) {
        return std::numeric_limits<double>::quiet_NaN(); // Avoid division by near-zero
    }
    
    double bottomThermistorResistance = (averageAnalogValue * topThermistorResistance) / 
                                        (VCC_MILLIVOLTS - averageAnalogValue);  
    
    return resistanceToTemperature(bottomThermistorResistance);
}

// Helper function to convert resistance to temperature (Celsius) using Beta equation
double resistanceToTemperature(double resistance) {
    if (resistance <= 0) {
        return std::numeric_limits<double>::quiet_NaN(); // Invalid resistance
    }
    
    double steinhart;
    steinhart = log(resistance / THERMISTOR_NOMINAL);      // ln(R/Ro)
    steinhart /= B_COEFFICIENT;                           // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);    // + (1/To)  (To in Kelvin)
    steinhart = 1.0 / steinhart;                          // Invert (Kelvin)
    return steinhart - 273.15;                            // Convert to Celsius
}

// Helper function to convert temperature (Celsius) to resistance
double temperatureToResistance(double temperatureCelsius) {
    double temperatureKelvin = temperatureCelsius + 273.15;
    return THERMISTOR_NOMINAL * exp(B_COEFFICIENT * (1.0/temperatureKelvin - 1.0/(TEMPERATURE_NOMINAL + 273.15)));
}

// Function to read temperature difference between two thermistors
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples) {
    if (isnan(topThermistorTemperatureCelsius)) {
        return std::numeric_limits<double>::quiet_NaN(); // Cannot calculate if reference temp is invalid
    }
    
    // Read bottom thermistor temperature using the relative method
    double bottomThermistorTemperatureCelsius = readThermistorRelative(
        bottomThermistorPin, 
        topThermistorTemperatureCelsius, 
        numSamples
    );
    
    if (isnan(bottomThermistorTemperatureCelsius)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Return temperature difference (top - bottom)
    return topThermistorTemperatureCelsius - bottomThermistorTemperatureCelsius;
}

// Function to draw the plot with temperature data
void drawPlot() {
    // Clear the plot area for redrawing the graph  
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    // Draw zero reference line for temperature difference
    tft.drawFastHLine(PLOT_X_START, PLOT_Y_START + (PLOT_HEIGHT/2), PLOT_WIDTH, PLOT_ZERO_COLOR);

    // Plot temperatures with fixed scaling  
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {  
        // Plot temperature 1 (Top thermistor)
        if (!isnan(temp1_values[i]) && !isnan(temp1_values[i + 1])) {
            int y1_prev = PLOT_Y_START + PLOT_HEIGHT - 
                          (int)mapf(temp1_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            int y1_current = PLOT_Y_START + PLOT_HEIGHT - 
                             (int)mapf(temp1_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            tft.drawLine(PLOT_X_START + i, y1_prev, PLOT_X_START + i + 1, y1_current, GRAPH_COLOR_1);  
        }
        
        // Plot temperature 2 (Bottom thermistor)
        if (!isnan(temp2_values[i]) && !isnan(temp2_values[i + 1])) {
            int y2_prev = PLOT_Y_START + PLOT_HEIGHT - 
                          (int)mapf(temp2_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            int y2_current = PLOT_Y_START + PLOT_HEIGHT - 
                             (int)mapf(temp2_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            tft.drawLine(PLOT_X_START + i, y2_prev, PLOT_X_START + i + 1, y2_current, GRAPH_COLOR_2);  
        }
        
        // Plot temperature difference
        if (!isnan(diff_values[i]) && !isnan(diff_values[i + 1])) {
            int y_diff_prev = PLOT_Y_START + PLOT_HEIGHT - 
                             (int)mapf(diff_values[i], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            int y_diff_current = PLOT_Y_START + PLOT_HEIGHT - 
                                (int)mapf(diff_values[i + 1], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);  
            tft.drawLine(PLOT_X_START + i, y_diff_prev, PLOT_X_START + i + 1, y_diff_current, GRAPH_COLOR_DIFF);  
        }  
    }
}

// Function to display temperature labels
void displayLabels(double temp1, double temp2, double tempDiff) {
    // Clear the label area
    tft.fillRect(PLOT_X_START, LABEL_Y_START, PLOT_WIDTH, 3 * LABEL_LINE_HEIGHT, TFT_BLACK);
    
    // Display current temperature values numerically as labels at the bottom  
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Text color, background color  
    tft.setTextSize(LABEL_TEXT_SIZE);  

    // Display Temperature 1 (Top)
    tft.setCursor(PLOT_X_START, LABEL_Y_START);
    tft.print("T1 (Top): ");  
    if (!isnan(temp1)) {  
        tft.printf("%.2f C", temp1);  
    } else {  
        tft.print("Error");  
    }  
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);  
    tft.print(" [Red]");  

    // Display Temperature 2 (Bottom)
    tft.setCursor(PLOT_X_START, LABEL_Y_START + LABEL_LINE_HEIGHT);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  
    tft.print("T2 (Bottom): ");  
    if (!isnan(temp2)) {  
        tft.printf("%.2f C", temp2);  
    } else {  
        tft.print("Error");  
    }  
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);  
    tft.print(" [Green]");  

    // Display Temperature Difference
    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * LABEL_LINE_HEIGHT);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  
    tft.print("Diff (T1-T2): ");  
    if (!isnan(tempDiff)) {  
        tft.printf("%.2f C", tempDiff);  
    } else {  
        tft.print("Error");  
    }  
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);  
    tft.print(" [Blue]");
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Set ESP32 ADC resolution to 12 bits

    // Initialize TFT  
    tft.init();  
    tft.setRotation(1); // Landscape mode  
    tft.fillScreen(TFT_BLACK);  

    // Initialize temperature arrays to default values
    for (int i = 0; i < PLOT_WIDTH; i++) {  
        temp1_values[i] = 25.0f;  
        temp2_values[i] = 25.0f;  
        diff_values[i] = 0.0f;  
    }  

    Serial.println("TFT Thermistor Monitor Started");
}

void loop() {
    // Read temperatures from thermistors
    double temp1 = readThermistor(THERMISTOR_PIN_2); // Top thermistor
    double temp2 = readThermistorRelative(THERMISTOR_PIN_1, temp1); // Bottom thermistor using top as reference
    double tempDiff = temp1 - temp2; // Calculate difference directly

    // Log temperature readings to serial monitor
    Serial.print("Thermistor 1 (Top): ");  
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.2f °C", temp1);  
    Serial.print(", Thermistor 2 (Bottom): ");  
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.2f °C", temp2);  
    Serial.print(", Diff (T1-T2): ");  
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.2f °C", tempDiff);  
    Serial.println();  

    // Update temperature arrays - shift values left to create scrolling effect  
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {  
        temp1_values[i] = temp1_values[i + 1];  
        temp2_values[i] = temp2_values[i + 1];  
        diff_values[i] = diff_values[i + 1];  
    }  
    
    // Add new values at the right edge
    temp1_values[PLOT_WIDTH - 1] = temp1;  
    temp2_values[PLOT_WIDTH - 1] = temp2;  
    diff_values[PLOT_WIDTH - 1] = tempDiff;  

    // Draw the temperature plot
    drawPlot();
    
    // Display temperature labels
    displayLabels(temp1, temp2, tempDiff);

    // Wait before next reading
    delay(LOOP_DELAY);
}
