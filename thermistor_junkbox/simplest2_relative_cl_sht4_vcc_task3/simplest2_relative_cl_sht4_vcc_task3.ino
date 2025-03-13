// Review, clean up, and improve code quality

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
#define THERMISTOR_PIN_1 36 // Analog pin for Thermistor 1
double THERMISTOR_1_OFFSET = -955.0;  // zero offset to cancel out slight thermistor differences
//double THERMISTOR_1_OFFSET = 0.0;  // zero offset to cancel out slight thermistor differences


#define THERMISTOR_PIN_2 39 // Analog pin for Thermistor 2
#define THERMISTOR_PIN_3 34 // Analog pin for Thermistor 3
//#define THERMISTOR_PIN_4 35 // Analog pin for Thermistor 4
#define THERMISTOR_VCC_PIN 35 // analog pin to measure VCC for thermistors

// Thermistor parameters - ADJUST THESE BASED ON YOUR THERMISTOR DATASHEET
const double THERMISTORNOMINAL     = 10000.0;  // Resistance at 25 degrees C (Ohms)
const double TEMPERATURENOMINAL    = 25.0;     // Temperature for nominal resistance (degrees C)
const double BCOEFFICIENT         = 3950.0;   // Beta coefficient (from datasheet)
const double SERIESRESISTOR        = 10000.0;  // Series resistor value in voltage divider circuit (Ohms)
//const double VCC_MILLIVOLTS       = 3300.0;   // Supply voltage in millivolts
//const double VCC_MILLIVOLTS       = 800.0;   // Supply voltage in millivolts
double VCC_MILLIVOLTS       = 1000.0;   // Thermistor supply voltage in millivolts


// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const double MIN_TEMP      = 15.0;
const double MAX_TEMP      = 30.0;
const double MIN_DIFF_TEMP = -1.0; // Difference range -3 to +3
const double MAX_DIFF_TEMP = 1.0;
//const double MIN_DIFF_TEMP = -0.4; // Difference range -3 to +3
//const double MAX_DIFF_TEMP = 0.4;


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

// --- Global variables ---
TFT_eSPI tft = TFT_eSPI(); // TFT_eSPI instance

// ---  SHT4x ambient temperature sensor
#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

uint32_t sht4_last_time = 0;
uint32_t sht4_update_interval = 500 ; // update once per 100ms
bool sht4_lock = false; // lock to prevent data being read while data is updated to variables

// Temperature reading arrays for plotting
float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// Global variables for thermistor readouts
double thermistor1Value = 25.0;
double thermistor2Value = 25.0;
double thermistorDiffValue = 0.0;
double thermistorVccValue = 600.0;
float thermistor1RawMillivolts = 300.0;
bool thermistor_lock = false;
uint32_t thermistor_last_time = 0;
const uint32_t thermistor_update_interval = 500; // Adjust as needed

// --- Function Declarations ---
double readThermistor(int pin, int numSamples);
double resistanceToTemperature(double resistance);
double temperatureToResistance(double temperatureCelsius);
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples, double VCC_offset);
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map
void  initSHT4x();

// --- Function Implementations ---

// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Improved Function to read thermistor temperature with enhanced precision and error handling
double readVCC(int pin, int numSamples = 128) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(pin);
        if (analogValue == 0) {
            // Handle potential zero reading - indicates error
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN (Not a Number) to indicate error
        }
        sumAnalogValues += analogValue;
        //delay(1); // Short delay between samples - can be adjusted if needed
    }
    double averageAnalogValue = sumAnalogValues / numSamples;

    return averageAnalogValue;
}

// Improved Function to read thermistor temperature with enhanced precision and error handling
double readThermistor(int pin, int numSamples = 1) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(pin);
        if (analogValue == 0) {
            // Handle potential zero reading - indicates error
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN (Not a Number) to indicate error
        }
        sumAnalogValues += analogValue;
        //delay(1); // Short delay between samples - can be adjusted if needed
    }
    double averageAnalogValue = sumAnalogValues / numSamples;

    // Convert average analog reading to resistance
    double resistance = (averageAnalogValue * SERIESRESISTOR) / (VCC_MILLIVOLTS - averageAnalogValue);
    return resistanceToTemperature(resistance);
}


// Improved Function to read thermistor temperature with enhanced precision and error handling
double readThermistorRelative(int pin, double topThermistorTemperatureCelsius, int numSamples = 1, double VCC_offset = 0) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(pin);
        if (analogValue == 0) {
            // Handle potential zero reading - indicates error
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN (Not a Number) to indicate error
        }
        sumAnalogValues += analogValue;
        //delay(1); // Short delay between samples - can be adjusted if needed
    }
    double averageAnalogValue = sumAnalogValues / numSamples;
    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);
    // Convert average analog reading to resistance
    double resistance = VCC_offset+(averageAnalogValue * topThermistorResistance) / (VCC_MILLIVOLTS - averageAnalogValue);
    return resistanceToTemperature(resistance);
}

// Helper function to convert resistance to temperature (Celsius) using Beta equation
double resistanceToTemperature(double resistance) {
    double steinhart;
    steinhart = log(resistance / THERMISTORNOMINAL);               // (R/Ro)
    steinhart /= BCOEFFICIENT;                                      // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);              // + (1/To)  (To in Kelvin)
    steinhart = 1.0 / steinhart;                                      // Invert (Kelvin)
    return steinhart - 273.15;                                       // Convert to Celsius
}

// Helper function to convert temperature (Celsius) to resistance
double temperatureToResistance(double temperatureCelsius) {
    double temperatureKelvin = temperatureCelsius + 273.15;
    return THERMISTORNOMINAL * exp(BCOEFFICIENT * (1.0/temperatureKelvin - 1.0/(TEMPERATURENOMINAL + 273.15)));
}


// Function to read temperature difference between two thermistors
// Assumes THERMISTOR_PIN_2 (topThermistorPin) temperature is already known and passed as argument
// Reads THERMISTOR_PIN_1 (bottomThermistorPin) analog value to calculate temperature difference.
double readThermistorDifference(int bottomThermistorPin, double topThermistorTemperatureCelsius, int numSamples = 64, double VCC_offset = 0 ) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(bottomThermistorPin); // Read analog value from bottom thermistor pin
        if (analogValue == 0) {
            return std::numeric_limits<double>::quiet_NaN(); // Indicate error
        }
        sumAnalogValues += analogValue;
        //delay(1); // Short delay between samples - uncomment if needed
    }
    double averageAnalogValue = sumAnalogValues / numSamples;

    // Calculate resistance of the top thermistor (T1) from its temperature
    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);

    // Calculate resistance of the bottom thermistor (T2) using the voltage divider equation
    // Vpin = VCC * (Rt2 / (Rt1 + Rt2))  where Rt1 is top thermistor resistance, Rt2 is bottom thermistor resistance
    // Rearranging for Rt2: Rt2 = (Vpin * Rt1) / (VCC - Vpin)
    double bottomThermistorResistance;

    if (VCC_MILLIVOLTS - averageAnalogValue < 0.1) { // Avoid division by zero or very small number. Adjust threshold if needed.
        return std::numeric_limits<double>::quiet_NaN(); // Indicate potential error if Vpin is too close to VCC
    }

    bottomThermistorResistance = VCC_offset + (averageAnalogValue * topThermistorResistance) / (VCC_MILLIVOLTS - averageAnalogValue);

    // Calculate temperature of the bottom thermistor (T2) from its resistance
    double bottomThermistorTemperatureCelsius = resistanceToTemperature(bottomThermistorResistance);

    // Return the temperature difference: topThermistorTemperature - bottomThermistorTemperature (T1 - T2)
    //return topThermistorTemperatureCelsius - bottomThermistorTemperatureCelsius;
    return -topThermistorTemperatureCelsius + bottomThermistorTemperatureCelsius; // or reverse
}

void initSHT4x () {
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
  }


// You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }


// You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);  // no heater
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }
  
}

// global variables for sht4x readouts
float aTemperature = 25.0;
float aHumidity = 50.0;

void readSHT4x(){
    sensors_event_t humidity, temp;
    sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    aTemperature = temp.temperature;
    aHumidity    = humidity.relative_humidity;
}

void task_readSHT4x(void* parameter) {
    while(true) {
    uint32_t sht4_current_time = millis();
    if ((sht4_current_time - sht4_last_time) > sht4_update_interval) {
      sensors_event_t humidity, temp;
      sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
      sht4_lock = true; // set lock to prevent reads
      aTemperature = (11.0*aTemperature + temp.temperature)/12.0;//average
      aHumidity    = humidity.relative_humidity;  
      sht4_lock = false; // release lock 
      sht4_last_time = millis();
    }    
    vTaskDelay(10);
    }
}

void task_readThermistor(void* parameter) {
    while (true) {
        uint32_t thermistor_current_time = millis();
        if ((thermistor_current_time - thermistor_last_time) > thermistor_update_interval) {
            thermistor_lock = true; // Set lock to prevent reads in loop

            // Read raw values
            while (sht4_lock) {vTaskDelay(1);}; // if lock is set, wait for sht4 task to update the data 
            double temp1 = aTemperature; // Use averaged SHT4x temperature as reference for the top thermistor

//            double rawTemp1 = readThermistor(THERMISTOR_PIN_2); // Assuming THERMISTOR_PIN_2 is for the top thermistor
//            double rawTemp2 = readThermistor(THERMISTOR_PIN_1);
            double currentVCC = readVCC(THERMISTOR_VCC_PIN, 128);
//            VCC_MILLIVOLTS = currentVCC; // update global variable TODO: should be passed as parameter...
            const float vcc_alpha = 0.1;
            thermistorVccValue = (1.0 - vcc_alpha) * thermistorVccValue + vcc_alpha * currentVCC; 
            VCC_MILLIVOLTS = thermistorVccValue; // Read averaged VCC value

            float rawMillivolts1 = analogReadMilliVolts(THERMISTOR_PIN_1);

            // Calculate derived values
            double temp2 = readThermistorRelative(THERMISTOR_PIN_1, temp1, 64, THERMISTOR_1_OFFSET); // Thermistor 2 (Bottom)
            double tempDiff = readThermistorDifference(THERMISTOR_PIN_1, temp1, 64, THERMISTOR_1_OFFSET);

            // Average the readings
            const float alpha = 0.1; // Smoothing factor for the moving average
            thermistor1Value = (1.0 - alpha) * thermistor1Value + alpha * temp1;
            thermistor2Value = (1.0 - alpha) * thermistor2Value + alpha * temp2;
            thermistorDiffValue = (1.0 - alpha) * thermistorDiffValue + alpha * tempDiff;
//            thermistorVccValue = (1.0 - alpha) * thermistorVccValue + alpha * currentVCC;
            thermistor1RawMillivolts = (1.0 - alpha) * thermistor1RawMillivolts + alpha * rawMillivolts1;

            thermistor_lock = false; // Release lock
            thermistor_last_time = millis();
        }
        vTaskDelay(10);
    }
}


void setup() {
    Serial.begin(115200);
    //analogReadResolution(12); // Set ESP32 ADC resolution to 12 bits

    // Initialize TFT
    tft.init();
    tft.setRotation(1); // Adjust rotation as needed

    // Clear screen and set background color
    tft.fillScreen(TFT_BLACK);

    // Draw plot frame and axes
//    tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_WHITE);
//    tft.drawLine(PLOT_X_START, PLOT_Y_START + PLOT_HEIGHT, PLOT_X_START + PLOT_WIDTH, PLOT_Y_START + PLOT_HEIGHT, PLOT_X_AXIS_COLOR); // X-axis
//    tft.drawLine(PLOT_X_START, PLOT_Y_START, PLOT_X_START, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_AXIS_COLOR);      // Y-axis
//    tft.drawLine(PLOT_X_START, PLOT_Y_START+PLOT_HEIGHT/2, PLOT_X_START, PLOT_Y_START + PLOT_HEIGHT/2, PLOT_Y_AXIS_COLOR);      // zero

    // Initialize temperature arrays to a default value (e.g., 25 degrees)
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
    }

    initSHT4x(); // initalize sht4x
//    xTaskCreatePinnedToCore(task_readSHT4x, "SHT4",4096,NULL,1,NULL,0);
    xTaskCreate(task_readSHT4x,      "SHT4",  4096, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 4096, NULL, 1, NULL); // Create the new thermistor reading task

    analogSetPinAttenuation(THERMISTOR_PIN_1,ADC_0db);
    analogSetPinAttenuation(THERMISTOR_PIN_2,ADC_0db);
    analogSetPinAttenuation(THERMISTOR_PIN_3,ADC_0db);
//    analogSetPinAttenuation(THERMISTOR_VCC_PIN,ADC_2_5db);
    analogSetPinAttenuation(THERMISTOR_VCC_PIN,ADC_0db);

    Serial.println("TFT and Thermistor Example Started");
}


// Function to read and retrieve thermistor data
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts) {
  while (thermistor_lock) {
    yield();
  } // Wait for thermistor task to update data
  temp1 = thermistor1Value;
  temp2 = thermistor2Value;
  tempDiff = thermistorDiffValue;
  t1_millivolts = thermistor1RawMillivolts;
}



// Function to print thermistor data to the serial monitor
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts) {
  Serial.print("Thermistor 1 (Top): ");
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
  tft.print("T1 (Top)   : ");
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
  if (!isnan(VCC_MILLIVOLTS)) {
    tft.printf("%.2f mV", VCC_MILLIVOLTS);
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
