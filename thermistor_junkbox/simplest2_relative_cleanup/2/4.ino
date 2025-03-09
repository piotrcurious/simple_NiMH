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
const double THERMISTORNOMINAL     = 10000.0;  // Resistance at 25 degrees C (Ohms)
const double TEMPERATURENOMINAL    = 25.0;     // Temperature for nominal resistance (degrees C)
const double BCOEFFICIENT         = 3950.0;   // Beta coefficient (from datasheet)
const double SERIESRESISTOR        = 10000.0;  // Series resistor value in voltage divider circuit (Ohms)
const double VCC_MILLIVOLTS       = 3300.0;   // Supply voltage in millivolts

// ADC Calibration Factor - ESP32 ADC may need calibration for better accuracy
// You can adjust this value based on your measurements
const double ADC_CALIBRATION_FACTOR = 1.0;

// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const double MIN_TEMP      = 20.0;
const double MAX_TEMP      = 30.0;
const double MIN_DIFF_TEMP = -3.0; // Difference range -3 to +3
const double MAX_DIFF_TEMP = 3.0;

// Plotting parameters - Maximized graph and bottom labels - ADJUST IF NEEDED
#define PLOT_WIDTH     320       // Maximize width (almost full screen)
#define PLOT_HEIGHT    (216-3)     // Maximize height (leaving space for labels at bottom)
#define PLOT_X_START   0         // Adjusted start position for wider graph
#define PLOT_Y_START   0         // Adjusted start position from top

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

// Precision improvement settings
#define DEFAULT_SAMPLES   32      // Increased from 16 to 32 for better averaging
#define ADC_MAX_VALUE     4095.0  // Maximum ADC value (12-bit = 4095)
#define FILTER_ALPHA      0.3     // Low-pass filter coefficient (0.0-1.0)

// --- Global variables ---
TFT_eSPI tft = TFT_eSPI(); // TFT_eSPI instance

// Temperature reading arrays for plotting
float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// IIR Filter previous values
double prev_temp1 = 0.0;
double prev_temp2 = 0.0;
double prev_diff = 0.0;
bool first_reading = true;

// --- Function Declarations ---
double readThermistor(int pin, int numSamples);
double resistanceToTemperature(double resistance);
double temperatureToResistance(double temperatureCelsius);
double readThermistorRelative(int pin, double topThermistorTemperatureCelsius, int numSamples);
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples);
double applyLowPassFilter(double newValue, double prevValue, double alpha);
double readCalibratedAnalogMilliVolts(int pin, int numSamples);
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map

// --- Function Implementations ---

// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Apply low-pass filter to smooth readings
double applyLowPassFilter(double newValue, double prevValue, double alpha) {
    // Skip filtering for the first reading or if previous value is NaN
    if (isnan(prevValue) || first_reading) {
        return newValue;
    }
    // Apply IIR filter: output = alpha * new_value + (1 - alpha) * prev_value
    return alpha * newValue + (1.0 - alpha) * prevValue;
}

// Read calibrated analog value with oversampling for noise reduction
double readCalibratedAnalogMilliVolts(int pin, int numSamples) {
    if (numSamples <= 0) numSamples = 1;
    
    // Use 64-bit accumulator to prevent overflow with large number of samples
    uint64_t sumAnalogValues = 0;
    int validSamples = 0;
    
    // Take samples in rapid succession to reduce temporal noise
    for (int i = 0; i < numSamples; ++i) {
        uint32_t sample = analogReadMilliVolts(pin);
        
        // Filter out spurious zero readings
        if (sample > 0) {
            sumAnalogValues += sample;
            validSamples++;
        }
    }
    
    // Check if we got any valid samples
    if (validSamples == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Calculate average and apply calibration factor
    double averageValue = (double)sumAnalogValues / validSamples;
    return averageValue * ADC_CALIBRATION_FACTOR;
}

// Improved function to read thermistor temperature with enhanced precision
double readThermistor(int pin, int numSamples = DEFAULT_SAMPLES) {
    // Read analog value with enhanced precision
    double milliVolts = readCalibratedAnalogMilliVolts(pin, numSamples);
    
    // Check for valid reading
    if (isnan(milliVolts) || milliVolts <= 0.0 || milliVolts >= VCC_MILLIVOLTS) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Calculate resistance with higher precision
    // R_thermistor = R_series * (Vmeasured / (Vcc - Vmeasured))
    double resistance = (milliVolts * SERIESRESISTOR) / (VCC_MILLIVOLTS - milliVolts);
    
    // Convert resistance to temperature
    double temperature = resistanceToTemperature(resistance);
    
    // Apply low-pass filter for smoother readings
    double filteredTemp = applyLowPassFilter(temperature, prev_temp1, FILTER_ALPHA);
    prev_temp1 = filteredTemp;
    
    return filteredTemp;
}

// Improved function to read relative thermistor temperature with enhanced precision
double readThermistorRelative(int pin, double topThermistorTemperatureCelsius, int numSamples = DEFAULT_SAMPLES) {
    // Calculate top thermistor resistance from its temperature
    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);
    
    // If top thermistor temperature is invalid, we can't calculate relative temperature
    if (isnan(topThermistorTemperatureCelsius) || isnan(topThermistorResistance)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Read analog value with enhanced precision
    double milliVolts = readCalibratedAnalogMilliVolts(pin, numSamples);
    
    // Check for valid reading
    if (isnan(milliVolts) || milliVolts <= 0.0 || milliVolts >= VCC_MILLIVOLTS) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Calculate resistance of bottom thermistor using top thermistor as reference resistor
    // R_bottom = R_top * (Vmeasured / (Vcc - Vmeasured))
    double bottomThermistorResistance = (milliVolts * topThermistorResistance) / (VCC_MILLIVOLTS - milliVolts);
    
    // Convert resistance to temperature
    double temperature = resistanceToTemperature(bottomThermistorResistance);
    
    // Apply low-pass filter for smoother readings
    double filteredTemp = applyLowPassFilter(temperature, prev_temp2, FILTER_ALPHA);
    prev_temp2 = filteredTemp;
    
    return filteredTemp;
}

// Helper function to convert resistance to temperature (Celsius) using Beta equation
double resistanceToTemperature(double resistance) {
    // Check for invalid resistance values
    if (resistance <= 0.0 || resistance > 10000000.0) { // Upper bound to catch disconnected thermistor
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Use Steinhart-Hart equation for better precision
    double steinhart = log(resistance / THERMISTORNOMINAL);  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                               // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);        // + (1/To) (To in Kelvin)
    steinhart = 1.0 / steinhart;                             // Invert (Kelvin)
    double temperatureCelsius = steinhart - 273.15;          // Convert to Celsius
    
    // Check for physically unreasonable temperatures
    if (temperatureCelsius < -50.0 || temperatureCelsius > 150.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    return temperatureCelsius;
}

// Helper function to convert temperature (Celsius) to resistance
double temperatureToResistance(double temperatureCelsius) {
    // Check for valid temperature range
    if (temperatureCelsius < -50.0 || temperatureCelsius > 150.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    double temperatureKelvin = temperatureCelsius + 273.15;
    double resistance = THERMISTORNOMINAL * exp(BCOEFFICIENT * (1.0/temperatureKelvin - 1.0/(TEMPERATURENOMINAL + 273.15)));
    
    return resistance;
}

// Improved function to read temperature difference with enhanced precision
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples = DEFAULT_SAMPLES) {
    // Calculate top thermistor resistance from its temperature
    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);
    
    // If top thermistor temperature is invalid, we can't calculate difference
    if (isnan(topThermistorTemperatureCelsius) || isnan(topThermistorResistance)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Read analog value with enhanced precision
    double milliVolts = readCalibratedAnalogMilliVolts(bottomThermistorPin, numSamples);
    
    // Check for valid reading
    if (isnan(milliVolts) || milliVolts <= 0.0 || milliVolts >= VCC_MILLIVOLTS) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Calculate resistance of bottom thermistor using top thermistor as reference resistor
    // R_bottom = R_top * (Vmeasured / (Vcc - Vmeasured))
    double bottomThermistorResistance = (milliVolts * topThermistorResistance) / (VCC_MILLIVOLTS - milliVolts);
    
    // Convert resistance to temperature
    double bottomThermistorTemperatureCelsius = resistanceToTemperature(bottomThermistorResistance);
    
    // Calculate temperature difference directly
    double tempDifference = topThermistorTemperatureCelsius - bottomThermistorTemperatureCelsius;
    
    // Apply low-pass filter for smoother readings
    double filteredDiff = applyLowPassFilter(tempDifference, prev_diff, FILTER_ALPHA);
    prev_diff = filteredDiff;
    
    return filteredDiff;
}

void setup() {
    Serial.begin(115200);
    
    // Set ESP32 ADC configuration for maximum precision
    analogReadResolution(12);     // Set ESP32 ADC resolution to 12 bits
    analogSetAttenuation(ADC_11db); // Set attenuation to increase input range
    
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
    
    Serial.println("TFT and Thermistor Example Started");
    
    // Mark that we haven't done any readings yet for filter initialization
    first_reading = true;
}

void loop() {
    // Read temperatures from thermistors with enhanced precision
    double temp1 = readThermistor(THERMISTOR_PIN_2); // Thermistor 1 (Top) connected to THERMISTOR_PIN_2
    double temp2 = readThermistorRelative(THERMISTOR_PIN_1, temp1); // Thermistor 2 (Bottom) connected to THERMISTOR_PIN_1, series resistor replaced with "top" thermistor
    double tempDiff = readThermistorDifference(THERMISTOR_PIN_1, temp1); // Calculate difference using T1 as top temperature and reading T2
    
    // After first reading, disable first_reading flag
    if (first_reading) {
        first_reading = false;
    }
    
    // Output readings to serial with higher precision (4 decimal places)
    Serial.print("Thermistor 1 (Top): ");  
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.4f °C", temp1);  
    Serial.print(", Thermistor 2 (Bottom): ");  
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.4f °C", temp2);  
    Serial.print(", Diff (T1-T2): ");  
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.4f °C", tempDiff);  
    Serial.println();  
    
    // Update temperature arrays, shifting old values to the left to create scrolling effect  
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {  
        temp1_values[i] = temp1_values[i + 1];  
        temp2_values[i] = temp2_values[i + 1];  
        diff_values[i] = diff_values[i + 1];  
    }  
    temp1_values[PLOT_WIDTH - 1] = temp1;  
    temp2_values[PLOT_WIDTH - 1] = temp2;  
    diff_values[PLOT_WIDTH - 1] = tempDiff;  
    
    // Clear the plot area for redrawing the graph  
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    
    // Draw zero line
    tft.drawFastHLine(PLOT_X_START, PLOT_Y_START+(PLOT_HEIGHT/2), PLOT_WIDTH, PLOT_ZERO_COLOR);
    
    // Plot temperatures with fixed scaling  
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {  
        if (!isnan(temp1_values[i]) && !isnan(temp1_values[i + 1])) { // Check for valid temperature values (not NaN)  
            int y1_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp1_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            int y1_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp1_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            tft.drawLine(PLOT_X_START + i, y1_prev, PLOT_X_START + i + 1, y1_current, GRAPH_COLOR_1);  
        }  
        if (!isnan(temp2_values[i]) && !isnan(temp2_values[i + 1])) { // Check for valid temperature values (not NaN)  
            int y2_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp2_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            int y2_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp2_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);  
            tft.drawLine(PLOT_X_START + i, y2_prev, PLOT_X_START + i + 1, y2_current, GRAPH_COLOR_2);  
        }  
        if (!isnan(diff_values[i]) && !isnan(diff_values[i + 1])) { // Check for valid temperature values (not NaN)  
            int y_diff_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(diff_values[i], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT); // Scale diff with fixed range  
            int y_diff_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(diff_values[i + 1], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);  
            tft.drawLine(PLOT_X_START + i, y_diff_prev, PLOT_X_START + i + 1, y_diff_current, GRAPH_COLOR_DIFF);  
        }  
    }  
    
    // Display current temperature values numerically as labels at the bottom with higher precision  
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Text color, background color  
    tft.setTextSize(LABEL_TEXT_SIZE);  
    
    int label_line_height = 8; // Approximate height per line of text with textSize=1  
    
    tft.setCursor(PLOT_X_START, LABEL_Y_START); // First label line  
    tft.print("T1 (Top): ");  
    if (!isnan(temp1)) {  
        tft.printf("%.3f C", temp1); // Increased precision to 3 decimal places  
    } else {  
        tft.print("Error");  
    }  
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);  
    tft.print(" [Red]");  
    
    tft.setCursor(PLOT_X_START, LABEL_Y_START + label_line_height); // Second label line  
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  
    tft.print("T2 (Bottom): ");  
    if (!isnan(temp2)) {  
        tft.printf("%.3f C", temp2); // Increased precision to 3 decimal places  
    } else {  
        tft.print("Error");  
    }  
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);  
    tft.print(" [Green]");  
    
    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * label_line_height); // Third label line  
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  
    tft.print("Diff (T1-T2): ");  
    if (!isnan(tempDiff)) {  
        tft.printf("%.4f C", tempDiff); // Increased precision to 4 decimal places for difference  
    } else {  
        tft.print("Error");  
    }  
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);  
    tft.print(" [Blue]");  
    
    delay(50); // Adjust delay for sampling rate
}
