#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <IRremote.h>
#include <cmath>     // For math functions like log, exp, pow, isnan
#include <limits>    // For std::numeric_limits
#include <vector>    // For dynamic arrays
#include <numeric>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <ArduinoEigenDense.h>
#include <cstdlib>
#include <map>
#include <ctime>

#include <Arduino.h>
#include "analog.h"


// Define additional constants for charging (adjust as needed)
//#define STABILIZATION_DELAY_MS 100    // Delay after setting a duty cycle to allow voltage stabilization
#define CHARGING_UPDATE_INTERVAL_MS 2000  // Interval between re-evaluation during charging
#define PWM_MAX 255                   // Maximum PWM value (for analogWrite)
#define TEST_DUTY_CYCLE 128           // Initial test duty cycle value
//#define VOLTAGE_TOLERANCE 0.05        // Voltage tolerance (in volts) for convergence
#define TOTAL_TIMEOUT (20UL * 60 * 60 * 1000) // 20h total timeout after charge stops for safety reasons

 // Constants for battery charging
const float MAX_TEMP_DIFF_THRESHOLD = 0.9f;         // Maximum temperature difference in Celsius before stopping charge
const float MH_ELECTRODE_RATIO = 0.6f;              // Target ratio for MH electrode voltage 
const int CHARGE_EVALUATION_INTERVAL_MS = 120000;   // Re-evaluate charging parameters every 2 minutes
// 600 points for 20 hours, so 19200 bytes data log . should fit into continous allocation of esp32 .. or not ...
const int CHARGE_CURRENT_STEP = 1;                  // Step size for PWM duty cycle adjustment
const int MAX_CHARGE_DUTY_CYCLE = 255;              // Maximum duty cycle for charging
const int MIN_CHARGE_DUTY_CYCLE = 5;                // Minimum duty cycle for charging
#define ISOLATION_THRESHOLD 0.02f // for oportunistic rint calculation - error distribution and averaging of data clusters

//end charge

// --- Configuration --- of R int measurement (and for current estimation)
//const int MAX_RESISTANCE_POINTS = 50; // Define as a constant
const float MEASURABLE_CURRENT_THRESHOLD = 0.005f; // Adjust as needed (40mA) - also used for estimation of current when duty cycle is below measurable current. 
const int MIN_DUTY_CYCLE_START = 8;
const int MAX_DUTY_CYCLE = 255;
const int DUTY_CYCLE_INCREMENT_FIND_MIN = 5;
const int STABILIZATION_DELAY_MS = 2000;
const int STABILIZATION_PAIRS_FIND_DELAY_MS = 1000; // two task current measurement cycles minimum
const int UNLOADED_VOLTAGE_DELAY_MS = 4000;
const int MIN_DUTY_CYCLE_ADJUSTMENT_STEP = 5;
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.02f;
const float MIN_VALID_RESISTANCE = 0.0f; // Threshold for considering a resistance value valid

// end of Rint configuration


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


struct Label {
    std::string text;
    int x;
    int y;
    uint16_t color;
    int textWidth;
    int textHeight;
    int lineStartX;
    int lineEndX;
    int y_initial;
    int lineY;
};

// Define the structure to hold the current model (polynomial coefficients)
struct CurrentModel {
    Eigen::VectorXd coefficients;
    bool isModelBuilt = false;
};

// Structure to hold supplementary tick information
struct TickLabel {
    int y;
    float value;
    uint16_t color;
    Label label;
};

// Global instance of the current model
CurrentModel currentModel;

// Structure to hold charge log data
struct ChargeLogData {
    unsigned long timestamp;
    float current;
    float voltage;
    float ambientTemperature; // Placeholder - you might need to get this from a sensor
    float batteryTemperature; // Assuming this is temp1 or temp2
    int dutyCycle;
    float internalResistanceLoadedUnloaded;
    float internalResistancePairs;
};

// Global vector to store charge log data
std::vector<ChargeLogData> chargeLog;

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
const double MIN_DIFF_TEMP        = -0.5; // Difference range 
const double MAX_DIFF_TEMP        = 1.5;

// Fixed voltage range for scaling the voltage plot
const float MIN_VOLTAGE = 1.0f;
const float MAX_VOLTAGE = 2.3f;

// Fixed current range for scaling the current plot - ADJUST IF NEEDED
const float MIN_CURRENT = 0.0f;
const float MAX_CURRENT = 0.40f; // Adjust this based on your expected current range

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
    KEY_GUIDE       = 0x4F,
    KEY_SOURCE      = 0x01

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
const int MAX_RESISTANCE_POINTS = 100; // Maximum number of data points for internal resistance
float internalResistanceData[MAX_RESISTANCE_POINTS][2]; // [current, internal_resistance]
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2]; // [current, internal_resistance] // from consecutive pairs
int resistanceDataCountPairs = 0;
float regressedInternalResistance = 0 ; 
float regressedInternalResistancePairs = 0 ; // from consecutive pairs
// Global variables to store the results of linear regression
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;

float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;

bool isMeasuringResistance = false;

// --- Function Declarations ---
float mapf(float value, float in_min, float in_max, float out_min, float out_max); // Float version of map
void controlPWM();
void setupPWM();
void measureInternalResistance();
void displayInternalResistanceGraph();

//   function declarations 

void processThermistorData(const MeasurementData& data, const String& measurementType );


// --- Function Implementations ---


// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



#define BUILD_CURRENT_MODEL_DELAY 200 // 200ms is enough (current measurement takes about 100ms)

// duty cycle to current modelling function 
// Function to build the current model
void buildCurrentModel(bool warmStart = false) {
    Serial.println("Building current model...");
    std::vector<float> dutyCycles;
    std::vector<float> currents;

    // Warm start: Load previous data if available (you'd need to implement storage)
    if (warmStart && currentModel.isModelBuilt) {
        Serial.println("Warm start: Accumulating data with previous model.");
        // For simplicity, we'll just continue adding new data in this example.
        // In a real application, you might load previously saved dutyCycles and currents.
    } else {
        Serial.println("Starting fresh model building.");
        dutyCycles.clear();
        currents.clear();
        // Ensure the first point is (0, 0)
        dutyCycles.push_back(0.0f);
        currents.push_back(0.0f);
    }

    // Sweep through duty cycles and measure current
    for (int dutyCycle = 1; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle += 1) { // Adjust step for faster sweep
        MeasurementData data = takeMeasurement(dutyCycle, BUILD_CURRENT_MODEL_DELAY);
        processThermistorData(data, "Estimating Min Current");
        if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
            dutyCycles.push_back(static_cast<float>(dutyCycle));
            currents.push_back(data.current);
        } else {
            Serial.printf("Current below threshold (%.3f A) at duty cycle %d. Skipping.\n", data.current, dutyCycle);
        }
    }

    if (dutyCycles.size() < 2) {
        Serial.println("Not enough data points to build a reliable model.");
        currentModel.isModelBuilt = false;
        dutyCycle = 0 ;
        analogWrite(pwmPin, dutyCycle);
        return;
    }

    // Polynomial regression using Eigen
    int degree = 3; // Choose the degree of the polynomial
    int numPoints = dutyCycles.size();
    Eigen::MatrixXd A(numPoints, degree + 1);
    Eigen::VectorXd b(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = std::pow(dutyCycles[i], j);
        }
        b(i) = currents[i];
    }

    // Solve for the coefficients using least squares
    Eigen::VectorXd coefficients = A.householderQr().solve(b);

    // Ensure the model passes through (0, 0) by forcing the constant term to zero
    if (degree >= 0) {
        coefficients(0) = 0.0f;
    }

    currentModel.coefficients = coefficients;
    currentModel.isModelBuilt = true;

    Serial.println("Current model built successfully with coefficients:");
    for (int i = 0; i <= degree; ++i) {
        Serial.printf("Coefficient for x^%d: %.4f\n", i, coefficients(i));
    }
    dutyCycle = 0 ;
    analogWrite(pwmPin, dutyCycle);

}

// Helper function to estimate current based on duty cycle
float estimateCurrent(int dutyCycle) {
    if (!currentModel.isModelBuilt) {
        Serial.println("Warning: Current model has not been built yet. Returning 0.");
        return 0.0f;
    }

    float estimatedCurrent = 0.0f;
    float dutyCycleFloat = static_cast<float>(dutyCycle);
    for (int i = 0; i < currentModel.coefficients.size(); ++i) {
        estimatedCurrent += currentModel.coefficients(i) * std::pow(dutyCycleFloat, i);
    }

    if (estimatedCurrent < MEASURABLE_CURRENT_THRESHOLD && dutyCycle > 0 ) {
        Serial.printf("Estimated current (%.3f A) below threshold at duty cycle %d. Inferring.\n", estimatedCurrent, dutyCycle);
    }

    return std::max(0.0f, estimatedCurrent); // Ensure current is not negative
}



// end duty cycle to current modeling function


// global variables for sht4x readouts (now managed by the class)
// float aTemperature = 25.0;
// float aHumidity = 50.0;

void task_readSHT4x(void* parameter) {
    while (true) {
        sht4Sensor.read();
        vTaskDelay(100);
    }
}

// Modified task_readThermistor function
void task_readThermistor(void* parameter) {
    mAh_last_time = millis(); // Initialize the last update time for mAh

    while (true) {
        uint32_t current_time = millis();

        // Wait for SHT4 sensor data if locked
        while (sht4Sensor.isLocked()) {
            vTaskDelay(10 / portTICK_PERIOD_MS); // Use portTICK_PERIOD_MS for FreeRTOS delays
        };

        // Read thermistor data (assuming this also updates VCC)
        thermistorSensor.read(sht4Sensor.getTemperature());

        // Read current and store in global variable (for model building)
        int task_current_numSamples = 256;
        double sumAnalogValuesCurrent = 0;
        for (int i = 0; i < task_current_numSamples; ++i) {
            //uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN); // Assuming your analogReadMillivolts function handles attenuation and oversampling
            uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN, CURRENT_SHUNT_ATTENUATION, CURRENT_SHUNT_OVERSAMPLING);
            sumAnalogValuesCurrent += analogValue;
        }
        double voltageAcrossShunt = (sumAnalogValuesCurrent / task_current_numSamples) - CURRENT_SHUNT_PIN_ZERO_OFFSET;
        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE); // in mA

        // Read voltage periodically
        if ((current_time - voltage_last_time) > voltage_update_interval) {
            int task_voltage_numSamples = 256;
            double sumAnalogValuesVoltage = 0;
            for (int i = 0; i < task_voltage_numSamples; ++i) {
                //uint32_t analogValue = analogReadMillivolts(VOLTAGE_READ_PIN); // Assuming your analogReadMillivolts function handles attenuation and oversampling
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
        double time_elapsed_hours = (double)time_elapsed / (1000.0 * 3600.0);
        double current_for_mah_calculation_ma;

        if (currentModel.isModelBuilt&& (current_ma/1000.0)<MEASURABLE_CURRENT_THRESHOLD) {
            // Use the estimated current for mAh calculation
            current_for_mah_calculation_ma = static_cast<double>(estimateCurrent(dutyCycle)*1000.0); // Convert Amps to mA
        } else {
            // Use the measured current for mAh calculation
            current_for_mah_calculation_ma = current_ma;
        }

        // Calculate mAh charged during this interval
        mAh_charged += (current_for_mah_calculation_ma) * time_elapsed_hours; // Convert mA to A, then to mAh

        mAh_last_time = current_time;

        // Check and reset mAh if the flag is set
        if (resetAh) {
            mAh_charged = 0.0f;
            resetAh = false; // Clear the flag after resetting
            Serial.println("mAh counter reset.");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Use portTICK_PERIOD_MS for FreeRTOS delays
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
    dutyCycle = 0;
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
        dutyCycle = 0;
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
void processThermistorData(const MeasurementData& data, const String& measurementType ) {
    printThermistorSerial(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    updateTemperatureHistory(data.temp1, data.temp2, data.tempDiff, data.voltage, data.current);
    prepareTemperaturePlot();
    plotVoltageData();
    plotTemperatureData();
    displayTemperatureLabels(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
    tft.print(measurementType);
}

void bigUglyMessage(const String& measurementType = "") {

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
            float currentDifference = std::fabs(midData.current - targetHighCurrent);

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
            storeResistanceData(loadedData.current, std::fabs(internalResistance), internalResistanceData, resistanceDataCount);
            Serial.printf("Calculated Internal Resistance (Loaded-Unloaded): %.3f Ohm\n", std::fabs(internalResistance));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            tft.printf("(L/UL): %.3f ", std::fabs(internalResistance));
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
            consecutiveInternalResistances.push_back(std::fabs(internalResistanceConsecutive));
            storeResistanceData(highData.current, std::fabs(internalResistanceConsecutive), internalResistanceDataPairs, resistanceDataCountPairs);
            Serial.printf("Calculated Internal Resistance (Pair): %.3f Ohm\n", std::fabs(internalResistanceConsecutive));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 50);
            tft.printf("(Pair): %.3f ", std::fabs(internalResistanceConsecutive));
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

        if (std::fabs(denominator) > 1e-9) {
            double overallInternalResistance = (n * sumIV - sumI * sumV) / denominator;
            double openCircuitVoltage = (sumV - overallInternalResistance * sumI) / n;

            regressedInternalResistance = overallInternalResistance;
            Serial.printf("Calculated Overall Internal Resistance (Linear Regression on Loaded Data): %.3f Ohm\n", std::fabs(overallInternalResistance));
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
//    calculateAverageInternalResistance(consecutiveInternalResistances);

    // Optional: Perform Linear Regression on the loaded voltage and current data
//    performLinearRegression(voltagesLoaded, currentsLoaded);

    // Perform linear regression after finding the optimal duty cycle
    if (resistanceDataCount >= 2) {
        if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
            Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistanceSlope, regressedInternalResistanceIntercept);     
        }
    } else {
        Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
    }

    if (resistanceDataCountPairs >= 2) {
        if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
            Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of paired resistance.");
    }


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


/*
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

    if (dataFound) {
        if (regressedInternalResistancePairs >= minResistance && regressedInternalResistancePairs <= maxResistance) {
            int overallResistanceY = mapf(regressedInternalResistancePairs, minResistance, maxResistance, graphYEnd, graphYStart);
            // Draw a horizontal line
            tft.drawLine(graphXStart, overallResistanceY, graphXEnd, overallResistanceY, TFT_WHITE);
            // Add a label for the line
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "Rint: %.2f", regressedInternalResistancePairs);
            tft.setCursor(graphXEnd - 50, overallResistanceY - 20); // Adjust position as needed
            tft.print(buffer);
            // tft.print((char)937); //ohm
        } else if (regressedInternalResistancePairs < minResistance) {
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(graphXEnd - 50, graphYEnd - 10); // Adjust position as needed
            tft.printf("R_int < %.2f", minResistance);
        } else if (regressedInternalResistancePairs > maxResistance) {
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(graphXEnd - 50, graphYStart + 10); // Adjust position as needed
            tft.printf("R_int > %.2f", maxResistance);
        }
    }
*/

   // --- Add the overall internal resistance line and label (Loaded/Unloaded Regression) ---
    if (dataFound) {
        if (resistanceDataCount >= 2) {
            float resistanceAtMinCurrent = regressedInternalResistanceSlope * minCurrent + regressedInternalResistanceIntercept;
            float resistanceAtMaxCurrent = regressedInternalResistanceSlope * maxCurrent + regressedInternalResistanceIntercept;

            if ((resistanceAtMinCurrent >= minResistance && resistanceAtMinCurrent <= maxResistance) ||
                (resistanceAtMaxCurrent >= minResistance && resistanceAtMaxCurrent <= maxResistance)) {
                int y1 = mapf(resistanceAtMinCurrent, minResistance, maxResistance, graphYEnd, graphYStart);
                int y2 = mapf(resistanceAtMaxCurrent, minResistance, maxResistance, graphYEnd, graphYStart);

                // Draw the regression line
                tft.drawLine(graphXStart, y1, graphXEnd, y2, TFT_WHITE);

                // Add a label for the line
                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE);
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "Rint(LU): %.2f + %.2f*I", regressedInternalResistanceIntercept, regressedInternalResistanceSlope);
                tft.setCursor(graphXEnd - 150, y2 - 10); // Adjust position as needed
                tft.print(buffer);
                // tft.print((char)937); //ohm
            } else {
                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE);
                tft.setCursor(graphXStart + 10, graphYStart + 10); // Adjust position as needed
                tft.print("Rint(LU) out of range");
            }
        } else if (resistanceDataCount == 1) {
            // If only one point, might want to show it as a single point or handle differently
            int overallResistanceY = mapf(internalResistanceData[0][1], minResistance, maxResistance, graphYEnd, graphYStart);
            tft.drawCircle(mapf(internalResistanceData[0][0], minCurrent, maxCurrent, graphXStart, graphXEnd), overallResistanceY, 2, TFT_WHITE);
        } else {
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(graphXStart + 10, graphYStart + 20); // Adjust position as needed
            tft.print("No Rint(LU) regression");
        }
    }

    // --- Add the overall internal resistance line and label (Pairs Regression) ---
    if (dataFound) {
        if (resistanceDataCountPairs >= 2) {
            float resistanceAtMinCurrentPairs = regressedInternalResistancePairsSlope * minCurrent + regressedInternalResistancePairsIntercept;
            float resistanceAtMaxCurrentPairs = regressedInternalResistancePairsSlope * maxCurrent + regressedInternalResistancePairsIntercept;

            if ((resistanceAtMinCurrentPairs >= minResistance && resistanceAtMinCurrentPairs <= maxResistance) ||
                (resistanceAtMaxCurrentPairs >= minResistance && resistanceAtMaxCurrentPairs <= maxResistance)) {
                int y1 = mapf(resistanceAtMinCurrentPairs, minResistance, maxResistance, graphYEnd, graphYStart);
                int y2 = mapf(resistanceAtMaxCurrentPairs, minResistance, maxResistance, graphYEnd, graphYStart);

                // Draw the regression line (using a different color if desired)
                tft.drawLine(graphXStart, y1, graphXEnd, y2, TFT_GREEN);

                // Add a label for the line
                tft.setTextSize(1);
                tft.setTextColor(TFT_GREEN);
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "Rint(Pair): %.2f + %.2f*I", regressedInternalResistancePairsIntercept, regressedInternalResistancePairsSlope);
                tft.setCursor(graphXEnd - 150, y2 + 10); // Adjust position as needed
                tft.print(buffer);
                // tft.print((char)937); //ohm
            } else {
                tft.setTextSize(1);
                tft.setTextColor(TFT_GREEN);
                tft.setCursor(graphXStart + 10, graphYStart + 30); // Adjust position as needed
                tft.print("Rint(Pair) out of range");
            }
        } else if (resistanceDataCountPairs == 1) {
            // If only one point, might want to show it as a single point or handle differently
            int overallResistanceY = mapf(internalResistanceDataPairs[0][1], minResistance, maxResistance, graphYEnd, graphYStart);
            tft.drawCircle(mapf(internalResistanceDataPairs[0][0], minCurrent, maxCurrent, graphXStart, graphXEnd), overallResistanceY, 2, TFT_GREEN);
        } else {
            tft.setTextSize(1);
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(graphXStart + 10, graphYStart + 40); // Adjust position as needed
            tft.print("No Rint(Pair) regression");
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

// Improved function to get a darker and more gray shade of a color
uint16_t darkerColor(uint16_t color, float darkeningFactor) {
    // Ensure factor is within valid range
    darkeningFactor = max(0.0f, min(1.0f, darkeningFactor));
    float grayingFactor = 0.6f; // Adjust for desired level of grayness

    // Extract RGB components (5-6-5 format)
    uint8_t r = (color >> 11) & 0x1F;
    uint8_t g = (color >> 5) & 0x3F;
    uint8_t b = color & 0x1F;

    // Convert to floating point (0.0 to 1.0 range)
    float fr = r / 31.0f;
    float fg = g / 63.0f;
    float fb = b / 31.0f;

    // Calculate luminance
    float luminance = 0.2126 * fr + 0.7152 * fg + 0.0722 * fb;

    // Reduce saturation
    fr = fr * (1.0f - grayingFactor) + luminance * grayingFactor;
    fg = fg * (1.0f - grayingFactor) + luminance * grayingFactor;
    fb = fb * (1.0f - grayingFactor) + luminance * grayingFactor;

    // Apply Darkening
    fr *= (1.0f - darkeningFactor);
    fg *= (1.0f - darkeningFactor);
    fb *= (1.0f - darkeningFactor);

    // Clamp values
    fr = max(0.0f, min(1.0f, fr));
    fg = max(0.0f, min(1.0f, fg));
    fb = max(0.0f, min(1.0f, fb));

    // Convert back to 5-6-5 format
    uint16_t new_r = static_cast<uint16_t>(fr * 31.0f + 0.5f);
    uint16_t new_g = static_cast<uint16_t>(fg * 63.0f + 0.5f);
    uint16_t new_b = static_cast<uint16_t>(fb * 31.0f + 0.5f);

    return (new_r << 11) | (new_g << 5) | new_b;
}

// Function to clear the plot area and draw the zero line
void prepareTemperaturePlot() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    // Calculate the Y-coordinate for the zero line of the temperature difference graph
    float zero_diff_mapped = mapf(0, MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
    int zero_diff_y = PLOT_Y_START + PLOT_HEIGHT - (int)zero_diff_mapped;

    // Draw the zero line for the temperature difference graph
    tft.drawFastHLine(PLOT_X_START, zero_diff_y, PLOT_WIDTH, PLOT_ZERO_COLOR);
}


void plotVoltageData() {
    if (PLOT_WIDTH <= 1) return; // Avoid division by zero

    // 1. Auto-scale voltage
    float min_voltage = 1000.0; // Initialize to a very high value
    float max_voltage = -1000.0; // Initialize to a very low value
    bool first_valid_voltage = true;

    for (int i = 0; i < PLOT_WIDTH; i++) {
        if (!isnan(voltage_values[i])) {
            if (first_valid_voltage) {
                min_voltage = voltage_values[i];
                max_voltage = voltage_values[i];
                first_valid_voltage = false;
            } else {
                min_voltage = fmin(min_voltage, voltage_values[i]);
                max_voltage = fmax(max_voltage, voltage_values[i]);
            }
        }
    }

    // Constrain auto-scale
    min_voltage = fmax(min_voltage, 1.15f);
    max_voltage = fmin(max_voltage, 3.0f);

    Serial.print("Calculated min_voltage: ");
    Serial.println(min_voltage);
    Serial.print("Calculated max_voltage: ");
    Serial.println(max_voltage);

    if (min_voltage == max_voltage) {
        // Handle the case where all voltage values are the same or NaN
        if (first_valid_voltage) {
            min_voltage = 0.5f;
            max_voltage = 1.0f; // Default range if no valid data
        } else {
            // Add a small range to make the line visible
            float offset = 0.1f;
            min_voltage -= offset;
            max_voltage += offset;
            min_voltage = fmax(min_voltage, 0.5f);
            max_voltage = fmin(max_voltage, 3.0f);
        }
    }

    uint16_t grid_color = darkerColor(GRAPH_COLOR_VOLTAGE, 0.25f); // Use the improved function

    // 2. Draw guidance grid (1/8 of graph width)
    int grid_x = PLOT_X_START + PLOT_WIDTH / 8;
//    tft.drawFastVLine(grid_x, PLOT_Y_START, PLOT_HEIGHT, grid_color);

    // 3. Draw horizontal grid lines and labels
    float target_voltages[] = {min_voltage, 1.25f, 1.38f, 1.55f, max_voltage};
    int num_targets = sizeof(target_voltages) / sizeof(target_voltages[0]);

    tft.setTextColor(grid_color);
    tft.setTextSize(1); // Adjust as needed

    for (int i = 0; i < num_targets; i++) {
        float voltage = target_voltages[i];
        float mapped_y = mapf(voltage, min_voltage, max_voltage, 0, PLOT_HEIGHT);
        int grid_y = PLOT_Y_START + PLOT_HEIGHT - (int)mapped_y;

        // Draw horizontal grid line
        tft.drawFastHLine(grid_x, grid_y, PLOT_WIDTH, grid_color);

        // Draw label using tft.drawFloat
        if (voltage == min_voltage) {
            //tft.drawFloat(voltage, 2, grid_x - 5, grid_y - 5, 1); // Adjust position as needed
        } else if (voltage == max_voltage) {
            //tft.drawFloat(voltage, 2, grid_x - 5, grid_y - 15, 1); // Adjust position as needed
        } else {
            tft.drawFloat(voltage, 2, grid_x - 5, grid_y + 8, 1); // Adjust position as needed
        }
    }

    // 4. Plot voltage data with auto-scaled range
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(voltage_values[i]) && !isnan(voltage_values[i + 1])) {
            int y_voltage_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(voltage_values[i], min_voltage, max_voltage, 0, PLOT_HEIGHT);
            int y_voltage_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(voltage_values[i + 1], min_voltage, max_voltage, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_voltage_prev, PLOT_X_START + i + 1, y_voltage_current, GRAPH_COLOR_VOLTAGE);
        }
    }

    // 5. Draw legend (min and max voltage) using tft.drawFloat
    tft.setTextColor(GRAPH_COLOR_VOLTAGE);
    tft.setTextSize(1); // Adjust as needed
    tft.drawFloat(min_voltage, 2, PLOT_X_START + PLOT_WIDTH - 40, PLOT_Y_START + PLOT_HEIGHT - 15, 1); // Adjust position
    //tft.drawString("Min V", PLOT_X_START + PLOT_WIDTH + 5, PLOT_Y_START + PLOT_HEIGHT - 5, 1); // Label
    tft.drawFloat(max_voltage, 2, PLOT_X_START + PLOT_WIDTH - 40, PLOT_Y_START, 1); // Adjust position
    //tft.drawString("Max V", PLOT_X_START + PLOT_WIDTH + 5, PLOT_Y_START + 10, 1); // Label
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
        // Removed the old voltage plotting here, call the dedicated function
        if (!isnan(current_values[i]) && !isnan(current_values[i + 1])) {
            int y_current_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(current_values[i], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            int y_current_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(current_values[i + 1], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_current_prev, PLOT_X_START + i + 1, y_current_current, GRAPH_COLOR_CURRENT);
        }
    }
    // Call the dedicated function to plot voltage with auto-scaling and grid
   // plotVoltageData();
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

// Helper function to insert a data point into the sorted array
void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count < MAX_RESISTANCE_POINTS) {
        for (int i = count; i > index; --i) {
            data[i][0] = data[i - 1][0];
            data[i][1] = data[i - 1][1];
        }
        data[index][0] = current;
        data[index][1] = resistance;
        count++;
    }
}

// Helper function to average two data points
void averageDataPoints(float data[][2], int index1, int index2) {
    data[index1][0] = (data[index1][0] + data[index2][0]) / 2.0f;
    data[index1][1] = (data[index1][1] + data[index2][1]) / 2.0f;
}

// Helper function to remove a data point at a given index
void removeDataPoint(float data[][2], int& count, int index) {
    if (index >= 0 && index < count) {
        for (int i = index; i < count - 1; ++i) {
            data[i][0] = data[i + 1][0];
            data[i][1] = data[i + 1][1];
        }
        count--;
    }
}

// Helper function to calculate the standard deviation of a vector of floats
float standardDeviation(const std::vector<float>& data) {
    if (data.empty()) return 0.0f;
    float sum = std::accumulate(data.begin(), data.end(), 0.0f);
    float mean = sum / data.size();
    std::vector<float> diffSq(data.size());
    std::transform(data.begin(), data.end(), diffSq.begin(),
                   [mean](float x){ return std::pow(x - mean, 2); });
    float sqSum = std::accumulate(diffSq.begin(), diffSq.end(), 0.0f);
    return std::sqrt(sqSum / data.size());
}

// Improved helper function to store or average resistance data with overflow handling
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (resistance <= 0) return; // Ignore invalid or negative resistance

    if (count < MAX_RESISTANCE_POINTS) {
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) {
            insertIndex++;
        }
        insertDataPoint(data, count, current, resistance, insertIndex);
    } else {
        int closestIndex = findClosestIndex(data, count, current);
        if (closestIndex >= 0 && closestIndex < MAX_RESISTANCE_POINTS) {

            float isolationThreshold = 0.0f; // Initialize

            if (count >= 2) {
                std::vector<float> spacings;
                for (int i = 1; i < count; ++i) {
                    spacings.push_back(data[i][0] - data[i - 1][0]);
                }

                if (!spacings.empty()) {
                    float sum = std::accumulate(spacings.begin(), spacings.end(), 0.0f);
                    float meanSpacing = sum / spacings.size();
                    float stdDevSpacing = standardDeviation(spacings);

                    // Define isolation threshold based on mean and standard deviation
                    isolationThreshold = meanSpacing + 1.5f * stdDevSpacing; // Adjust the multiplier as needed
                    if (isolationThreshold <= 0) isolationThreshold = 0.02f; // Ensure a minimum threshold
                } else {
                    isolationThreshold = 0.02f; // Default if only one spacing
                }
            } else {
                isolationThreshold = 0.02f; // Default if less than 2 points
            }

            bool isIsolated = true;
            if (count > 1) {
                float distanceToPrev = (closestIndex > 0) ? std::fabs(data[closestIndex][0] - data[closestIndex - 1][0]) : isolationThreshold * 2.0f;
                float distanceToNext = (closestIndex < count - 1) ? std::fabs(data[closestIndex][0] - data[closestIndex + 1][0]) : isolationThreshold * 2.0f;

                if (distanceToPrev < isolationThreshold || distanceToNext < isolationThreshold) {
                    isIsolated = false;
                }
            } else {
                isIsolated = false; // Not isolated if only one point exists
            }

            if (isIsolated && count >= 2) {
                int index1 = -1, index2 = -1;

                if (closestIndex > 0) {
                    index1 = closestIndex - 1;
                }
                if (closestIndex < count - 1) {
                    index2 = closestIndex + 1;
                }

                if (index1 != -1 && index2 != -1) {
                    // Average the two neighbors
                    averageDataPoints(data, index1, index2);
                    // Remove the second averaged point
                    removeDataPoint(data, count, index2);
                    // Insert the new data point at the correct sorted position
                    int insertIndex = 0;
                    while (insertIndex < count && data[insertIndex][0] < current) {
                        insertIndex++;
                    }
                    insertDataPoint(data, count, current, resistance, insertIndex);
                } else {
                    // Fallback to averaging with the closest if no two other close points are available
                    data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                    data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
                }
            } else {
                // Average the new data with the closest point if not isolated or less than 2 points exist
                data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
            }
        }
    }
}

// --- New function to distribute error ---
void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < 4) return; // Need at least 4 points for a cluster

    for (int i = 0; i <= count - 4; ++i) {
        // Check for a potential cluster of at least 4 points
        for (int j = i + 3; j < count; ++j) {
            if (data[j][0] - data[i][0] <= spacing_threshold) {
                // Found a cluster from index i to j (inclusive)
                std::vector<float> cluster_resistances;
                for (int k = i; k <= j; ++k) {
                    cluster_resistances.push_back(data[k][1]);
                }

                if (cluster_resistances.size() >= 4) {
                    float sum_resistance = std::accumulate(cluster_resistances.begin(), cluster_resistances.end(), 0.0f);
                    float average_resistance = sum_resistance / cluster_resistances.size();
                    float std_dev_resistance = standardDeviation(cluster_resistances);

                    std::vector<int> high_error_indices;
                    for (int k = i; k <= j; ++k) {
                        if (std::fabs(data[k][1] - average_resistance) > error_threshold_multiplier * std_dev_resistance) {
                            high_error_indices.push_back(k);
                        }
                    }

                    if (!high_error_indices.empty()) {
                        // Distribute the error by setting high error points to the average
                        for (int index : high_error_indices) {
                            data[index][1] = average_resistance;
                        }
                        // Optionally, you could implement a more sophisticated error distribution here
                        // e.g., shifting the difference proportionally to other points in the cluster.
                    }
                    // Move the outer loop index 'i' to the end of the current cluster
                    // to avoid re-processing overlapping clusters immediately.
                    i = j;
                    break; // Break the inner loop 'j' as we've processed this cluster
                }
            } else {
                break; // Points are no longer closely spaced, move to the next potential starting point 'i'
            }
        }
    }
}



// --- Linear Regression Function ---
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < 2) {
        Serial.println("Insufficient data points for linear regression.");
        return false;
    }

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumXY = 0.0f;
    float sumX2 = 0.0f;

    for (int i = 0; i < count; ++i) {
        sumX += data[i][0];
        sumY += data[i][1];
        sumXY += data[i][0] * data[i][1];
        sumX2 += data[i][0] * data[i][0];
    }

    float n = static_cast<float>(count);
    float denominator = n * sumX2 - sumX * sumX;

    if (std::fabs(denominator) < 1e-6) { // Avoid division by zero
        Serial.println("Denominator is too small for linear regression.");
        return false;
    }

    slope = (n * sumXY - sumX * sumY) / denominator;
    intercept = (sumY - slope * sumX) / n;

    return true;
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
// --- Modified findOptimalChargingDutyCycle function with caching ---
int findOptimalChargingDutyCycle(int maxChargeDutyCycle, int suggestedStartDutyCycle) {

//    Serial.println("building duty cycle to current model...");
//    bigUglyMessage("duty cycle model");
//    buildCurrentModel(true); // Accumulate more data and refine the model

    Serial.println("Finding optimal charging duty cycle...");
    bigUglyMessage("finding duty cycle");

    if (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) {
        Serial.println("Error: maxChargeDutyCycle is invalid, using MAX_CHARGE_DUTY_CYCLE instead.");
        maxChargeDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    }

    int optimalDutyCycle = MIN_CHARGE_DUTY_CYCLE;
    float closestVoltageDifference = 1000.0f; // Initialize with a large value
    float targetVoltage = 0.0f; // Will be updated in the loop

    // Start with a binary search approach to narrow down the range
    int lowDC = MIN_CHARGE_DUTY_CYCLE;
    int highDC = maxChargeDutyCycle;

    // First, measure the unloaded voltage once
    MeasurementData initialUnloadedData = getUnloadedVoltageMeasurement();
    processThermistorData(initialUnloadedData, "MH idle (initial)");
    float initialUnloadedVoltage = initialUnloadedData.voltage;

    // Measure loaded voltage at maximum duty cycle
    MHElectrodeData dataHigh = measureMHElectrodeVoltage(highDC);

    // Calculate and store internal resistance during initial measurement at max duty cycle (Loaded/Unloaded)
    if (dataHigh.current > 0.01f) {
        float internalResistanceLUInitial = (initialUnloadedVoltage - dataHigh.loadedVoltage) / dataHigh.current;
        storeOrAverageResistanceData(dataHigh.current, std::fabs(internalResistanceLUInitial), internalResistanceData, resistanceDataCount);
        bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
    }

    // Recalculate target voltage based on the initial unloaded voltage and max duty cycle possible
    targetVoltage = initialUnloadedVoltage + (dataHigh.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

    MHElectrodeData previousData = dataHigh; // Store the result of the max duty cycle measurement
    bool firstIteration = true;

    Serial.printf("Initial Measurement (Max DC %d): Unloaded: %.3fV, Loaded: %.3fV, Current: %.3fA\n",
                  highDC, initialUnloadedVoltage, dataHigh.loadedVoltage, dataHigh.current);

    // --- Introduce a cache for MHElectrodeData ---
    std::vector<MHElectrodeData> dataCache;
    const int MAX_CACHE_SIZE = MAX_RESISTANCE_POINTS; // Adjust the cache size as needed

    auto addToCache = [&](const MHElectrodeData& data) {
        if (dataCache.size() >= MAX_CACHE_SIZE) {
            dataCache.erase(dataCache.begin()); // Remove the oldest element
        }
        dataCache.push_back(data);
    };

    // Add the initial high duty cycle measurement to the cache
    addToCache(dataHigh);

    // Set the starting point for the binary search
    lowDC = max(MIN_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
    
    if (lowDC > highDC) {
        lowDC = MIN_CHARGE_DUTY_CYCLE; // Ensure lowDC is not greater than highDC
    }

    while (highDC - lowDC > CHARGE_CURRENT_STEP * 2) {
        int midDC = (lowDC + highDC) / 2;

        MHElectrodeData currentData = measureMHElectrodeVoltage(midDC);
        if (currentData.current < MEASURABLE_CURRENT_THRESHOLD) {currentData.current=static_cast<double>(estimateCurrent(midDC));}; // replace with inference if below threshold

        // Add current data to the cache
        addToCache(currentData);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            storeOrAverageResistanceData(currentData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
            bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
        }

        // --- Check cache for more opportunities to calculate internal resistance (Pair) ---
        for (const auto& cachedData : dataCache) {
            if (std::fabs(currentData.current - cachedData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                float internalResistancePair = (cachedData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - cachedData.current);
                float higherCurrent = std::max(currentData.current, cachedData.current);
                storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
                bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs); // Sort after adding
            }
        }

        float currentVoltageDifference = fabs(currentData.loadedVoltage - targetVoltage);

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

/*
    // Fine tune with small steps around the approximate value
    for (int dc = max(MIN_CHARGE_DUTY_CYCLE, optimalDutyCycle - CHARGE_CURRENT_STEP * 3);
         dc <= min(MAX_CHARGE_DUTY_CYCLE, optimalDutyCycle + CHARGE_CURRENT_STEP * 3);
         dc += CHARGE_CURRENT_STEP) {

        MHElectrodeData currentData = measureMHElectrodeVoltage(dc);

        // Add current data to the cache
        addToCache(currentData);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            storeOrAverageResistanceData(currentData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
            bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
        }

        // --- Check cache for more opportunities to calculate internal resistance (Pair) ---
        for (const auto& cachedData : dataCache) {
            if (std::abs(currentData.current - cachedData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                float internalResistancePair = (cachedData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - cachedData.current);
                float higherCurrent = std::max(currentData.current, cachedData.current);
                storeOrAverageResistanceData(higherCurrent, std::abs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
                bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs); // Sort after adding
            }
        }

        float currentVoltageDifference = fabs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Fine-tuning - Duty Cycle: %d, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      dc, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = dc;
        }
        previousData = currentData;
    }
*/

    // Perform error distribution on the collected Loaded/Unloaded resistance data
    distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f); // Example parameters, adjust as needed

    // Perform error distribution on the collected paired resistance data
    distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f); // Example parameters, adjust as needed

    // Final measurement at the optimal duty cycle
    MHElectrodeData finalData = measureMHElectrodeVoltage(optimalDutyCycle);
    targetVoltage = initialUnloadedVoltage + (finalData.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

    Serial.printf("Optimal charging duty cycle found: %d (loaded: %.3fV, target: %.3fV, diff: %.3fV)\n",
                  optimalDutyCycle, finalData.loadedVoltage, targetVoltage, fabs(finalData.loadedVoltage - targetVoltage));

    // Perform linear regression after finding the optimal duty cycle
    if (resistanceDataCount >= 2) {
        if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
            Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
    }

    if (resistanceDataCountPairs >= 2) {
        if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
            Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of paired resistance.");
    }


    return optimalDutyCycle;
}


// Function to estimate temperature difference (in °C) due to heating by current
float estimateTempDiff(float voltageUnderLoad, float voltageNoLoad, 
                         float current, float internalResistance, 
                         float ambientTempC) {
  // Calculate electrical power dissipated (in Watts)
  // (This assumes that the voltage drop due to the internal resistance is given by I*R)
  float power = current * current * internalResistance;
  
  // Assumed battery parameters for a AAA NiMH cell:
  // Estimated surface area (m^2) of a typical AAA cell (~44mm x 10mm cylinder)
  //float A = 0.0015; // m^2
  float A = 0.001477 ; // m^2 for 10mm and 42mm
  
  // Convective heat transfer coefficient (W/m^2·K)
  float h = 8.0;
  
  // Radiation parameters
  float epsilon = 0.9;           // Emissivity (dimensionless)
  float sigma = 5.67e-8;         // Stefan-Boltzmann constant (W/m^2·K^4)
  
  // Convert ambient temperature from Celsius to Kelvin
  float T_ambient = ambientTempC + 273.15;
  
  // Effective thermal conductance (W/K)
  // For small ΔT, the radiation term can be linearized: 
  //   Radiation loss ≈ 4 * ε * σ * A * T_ambient^3 * ΔT
  float G = h * A + 4 * epsilon * sigma * A * pow(T_ambient, 3);
  
  // Estimate temperature rise (ΔT in Kelvin, equivalent to °C difference)
  float deltaT = power / G;
  
  return deltaT;
}



bool checkCollision(const Label& label1, const Label& label2) {
    return label1.x < label2.x + label2.textWidth &&
           label1.x + label1.textWidth > label2.x &&
           label1.y < label2.y + label2.textHeight &&
           label1.y + label1.textHeight > label2.y;
}

bool checkCollision(const Label& label1, const TickLabel& tickLabel) {
    return checkCollision(label1, tickLabel.label);
}

bool checkCollision(const TickLabel& tickLabel1, const Label& label2) {
    return checkCollision(tickLabel1.label, label2);
}

bool checkCollision(const TickLabel& tickLabel1, const TickLabel& tickLabel2) {
    return checkCollision(tickLabel1.label, tickLabel2.label);
}

// Enum for label line placement
enum class LinePlacement {
    BEGINNING,
    END,
    CENTER,
    LEFT,
    RIGHT,
    FRACTION_LEFT_1,
    FRACTION_LEFT_2,
    FRACTION_RIGHT_1,
    FRACTION_RIGHT_2
};

// Enum for label vertical placement
enum class LabelVerticalPlacement {
    CENTER,
    ABOVE,
    BELOW
};

void drawChargePlot(bool autoscaleX, bool autoscaleY) {
    if (chargeLog.empty()) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(6); // MC_DATUM
        tft.drawString("No charge data to plot", tft.width() / 2, tft.height() / 2, 2);
        return;
    }

    tft.fillScreen(TFT_BLACK);

    int plotAreaWidth = tft.width() - 40;
    int plotAreaHeight = tft.height() - 60; // Adjusted height to accommodate legend
    int marginX = 20;
    int marginYTop = 20; // Adjusted top margin

    // --- Determine X-axis scaling ---
    unsigned long startTime = chargeLog.front().timestamp;
    unsigned long endTime = chargeLog.back().timestamp;
    float timeScaleX = 1.0;
    if (autoscaleX && endTime > startTime) {
        timeScaleX = (float)plotAreaWidth / (endTime - startTime);
    } else if (!autoscaleX) {
        unsigned long window = 10 * 60 * 1000; // 10 minutes
        startTime = std::max((long long)startTime, (long long)endTime - window);
        if (endTime > startTime) {
            timeScaleX = (float)plotAreaWidth / (endTime - startTime);
        } else {
            timeScaleX = 1.0;
        }
    }

    // --- Determine Y-axis scaling for all parameters ---
    float currentMin = 1000.0, currentMax = -1000.0;
    float voltageMin = 1000.0, voltageMax = -1000.0;
    float dutyCycleMin = 1000.0, dutyCycleMax = -1000.0;
    float tempDiffMin = 1000.0, tempDiffMax = -1000.0;
    float estTempDiffThresholdMin = 1000.0, estTempDiffThresholdMax = -1000.0;
    float irLU_Min = 1000.0, irLU_Max = -1000.0;
    float irPairs_Min = 1000.0, irPairs_Max = -1000.0;

    if (autoscaleY) {
        for (const auto& logEntry : chargeLog) {
            currentMin = std::fmin(currentMin, logEntry.current);
            currentMax = std::fmax(currentMax, logEntry.current);
            voltageMin = std::fmin(voltageMin, logEntry.voltage);
            voltageMax = std::fmax(voltageMax, logEntry.voltage);
            dutyCycleMin = std::fmin(dutyCycleMin, (float)logEntry.dutyCycle);
            dutyCycleMax = std::fmax(dutyCycleMax, (float)logEntry.dutyCycle);
            float currentTempDiff = logEntry.batteryTemperature - logEntry.ambientTemperature;
            tempDiffMin = std::fmin(tempDiffMin, currentTempDiff);
            tempDiffMax = std::fmax(tempDiffMax, currentTempDiff);
            float estimatedDiff = estimateTempDiff(logEntry.voltage, logEntry.voltage, logEntry.current, regressedInternalResistanceIntercept, logEntry.ambientTemperature);
            float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
            estTempDiffThresholdMin = std::fmin(estTempDiffThresholdMin, thresholdValue);
            estTempDiffThresholdMax = std::fmax(estTempDiffThresholdMax, thresholdValue);
            irLU_Min = std::fmin(irLU_Min, logEntry.internalResistanceLoadedUnloaded);
            irLU_Max = std::fmax(irLU_Max, logEntry.internalResistanceLoadedUnloaded);
            irPairs_Min = std::fmin(irPairs_Min, logEntry.internalResistancePairs);
            irPairs_Max = std::fmax(irPairs_Max, logEntry.internalResistancePairs);
        }
        // Add some padding if needed
        if (currentMin == currentMax) { currentMin -= 0.1; currentMax += 0.1; }
        if (voltageMin == voltageMax) { voltageMin -= 0.1; voltageMax += 0.1; }
        if (dutyCycleMin == dutyCycleMax) { dutyCycleMin -= 1; dutyCycleMax += 1; }
        if (tempDiffMin == tempDiffMax) { tempDiffMin -= 1; tempDiffMax += 1; }
        if (estTempDiffThresholdMin == estTempDiffThresholdMax) { estTempDiffThresholdMin -= 0.1; estTempDiffThresholdMax += 0.1; }
        if (irLU_Min == irLU_Max) { irLU_Min -= 0.01; irLU_Max += 0.01; }
        if (irPairs_Min == irPairs_Max) { irPairs_Min -= 0.01; irPairs_Max += 0.01; }
    } else {
        currentMin = 0.0; currentMax = 0.4;
        voltageMin = 1.0; voltageMax = 2.0;
        dutyCycleMin = 0.0; dutyCycleMax = 255.0;
        tempDiffMin = -0.5; tempDiffMax = 1.5;
        estTempDiffThresholdMin = -0.5; estTempDiffThresholdMax = 1.5;
        irLU_Min = 0.0; irLU_Max = 1.5;
        irPairs_Min = 0.0; irPairs_Max = 1.5;
    }

    float scaleY = plotAreaHeight;

    auto scaleValue = [&](float val, float minVal, float maxVal) {
        if (maxVal <= minVal) return (float)(plotAreaHeight / 2.0); // Avoid division by zero
        float scaleValue_result = (plotAreaHeight - (val - minVal) / (maxVal - minVal) * scaleY);
        return marginYTop + scaleValue_result;
    };

    auto inverseScaleValue = [&](float yPixel, float minVal, float maxVal) {
        if (maxVal <= minVal) return (minVal + maxVal) / 2.0f;
        float normalizedY = (yPixel - marginYTop) / scaleY;
        return minVal + (1.0f - normalizedY) * (maxVal - minVal);
    };

    // Draw plot axes
    tft.drawLine(marginX, marginYTop, marginX + plotAreaWidth, marginYTop, TFT_DARKGREY); // X-axis
    tft.drawLine(marginX, marginYTop, marginX, marginYTop + plotAreaHeight, TFT_DARKGREY); // Y-axis

    // Plot data with different colors
    for (size_t i = 1; i < chargeLog.size(); ++i) {
        int x1 = marginX + (chargeLog[i - 1].timestamp - startTime) * timeScaleX;
        int x2 = marginX + (chargeLog[i].timestamp - startTime) * timeScaleX;

        // Current
        int y1 = scaleValue(chargeLog[i - 1].current, currentMin, currentMax);
        int y2 = scaleValue(chargeLog[i].current, currentMin, currentMax);
        tft.drawLine(x1, y1, x2, y2, TFT_MAGENTA);

        // Voltage
        y1 = scaleValue(chargeLog[i - 1].voltage, voltageMin, voltageMax);
        y2 = scaleValue(chargeLog[i].voltage, voltageMin, voltageMax);
        tft.drawLine(x1, y1, x2, y2, TFT_YELLOW);

        // Duty Cycle
        y1 = scaleValue(chargeLog[i - 1].dutyCycle, dutyCycleMin, dutyCycleMax);
        y2 = scaleValue(chargeLog[i].dutyCycle, dutyCycleMin, dutyCycleMax);
        tft.drawLine(x1, y1, x2, y2, TFT_DARKGREY);

        // Temperature Difference
        float currentTempDiffPrev = chargeLog[i - 1].batteryTemperature - chargeLog[i - 1].ambientTemperature;
        float currentTempDiffCurr = chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature;
        y1 = scaleValue(currentTempDiffPrev, tempDiffMin, tempDiffMax);
        y2 = scaleValue(currentTempDiffCurr, tempDiffMin, tempDiffMax);
        tft.drawLine(x1, y1, x2, y2, TFT_BLUE);

        // Estimated Temperature Difference Threshold
        float estimatedDiffPrev = estimateTempDiff(chargeLog[i - 1].voltage, chargeLog[i - 1].voltage, chargeLog[i - 1].current, regressedInternalResistanceIntercept, chargeLog[i - 1].ambientTemperature);
        float thresholdValuePrev = MAX_TEMP_DIFF_THRESHOLD + estimatedDiffPrev;
        float estimatedDiffCurr = estimateTempDiff(chargeLog[i].voltage, chargeLog[i].voltage, chargeLog[i].current, regressedInternalResistanceIntercept, chargeLog[i].ambientTemperature);
        float thresholdValueCurr = MAX_TEMP_DIFF_THRESHOLD + estimatedDiffCurr;
        y1 = scaleValue(thresholdValuePrev, estTempDiffThresholdMin, estTempDiffThresholdMax);
        y2 = scaleValue(thresholdValueCurr, estTempDiffThresholdMin, estTempDiffThresholdMax);
        tft.drawLine(x1, y1, x2, y2, TFT_SKYBLUE);

        // Internal Resistance Loaded/Unloaded
        y1 = scaleValue(chargeLog[i - 1].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
        y2 = scaleValue(chargeLog[i].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
        tft.drawLine(x1, y1, x2, y2, TFT_ORANGE);

        // Internal Resistance Pairs
        y1 = scaleValue(chargeLog[i - 1].internalResistancePairs, irPairs_Min, irPairs_Max);
        y2 = scaleValue(chargeLog[i].internalResistancePairs, irPairs_Min, irPairs_Max);
        tft.drawLine(x1, y1, x2, y2, TFT_CYAN);
    }

    // --- Draw dynamic value labels ---
    std::vector<Label> labels;
    float labelLineLengthFactor = 0.20f; // Make the initial lines shorter, like ticks
    int maxLabelLineLength = static_cast<int>(plotAreaWidth * 0.1f); // Max length for magnitude representation
    float darkeningFactor = 0.1f;
    int textSpacing = 2;
    int tickLength = 20; // Length of supplementary ticks
    int supplementaryTickLength = 5;

    // Hardcoded value ranges for magnitude representation (example values, adjust as needed)
    std::map<std::string, std::pair<float, float>> hardcodedRanges = {
        {"Current", {0.0f, 0.4f}},
        {"Voltage", {0.0f, 2.5f}},
        {"Duty", {0.0f, 255.0f}},
        {"TempDiff", {-0.5f, 2.0f}},
        {"TempDiffThresh", {-0.5f, 2.5f}},
        {"IR_LU", {0.0f, 2.0f}},
        {"IR_Pairs", {0.0f, 2.0f}}
    };

    auto calculateDynamicLineLength = [&](float value, float minHardcoded, float maxHardcoded) {
        if (maxHardcoded <= minHardcoded) return tickLength;
        float normalizedValue = std::abs(value - minHardcoded) / (maxHardcoded - minHardcoded);
        return static_cast<int>(normalizedValue * maxLabelLineLength);
    };

    auto createLabel = [&](const std::string& name, float value, float minValue, float maxValue, uint16_t color) {
        float yValue = scaleValue(value, minValue, maxValue);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << value;
        std::string text = ss.str();
        uint16_t darkColor = darkerColor(color, darkeningFactor);

        int tw = tft.textWidth(text.c_str(), 1);
        int th = tft.fontHeight(1);

        Label label;
        label.text = text;
        label.y = static_cast<int>(yValue); // Initial Y
        label.color = darkColor;
        label.textWidth = tw;
        label.textHeight = th;
        label.lineY = static_cast<int>(yValue);
        label.lineStartX = 0; // Will be set later
        label.lineEndX = 0;     // Will be set later
        label.y_initial = static_cast<int>(yValue);
        return label;
    };

    // Add min/max labels for each parameter
    labels.push_back(createLabel("Current", currentMax, currentMin, currentMax, TFT_MAGENTA));
    labels.push_back(createLabel("Current", currentMin, currentMin, currentMax, TFT_MAGENTA));
    labels.push_back(createLabel("Voltage", voltageMax, voltageMin, voltageMax, TFT_YELLOW));
    labels.push_back(createLabel("Voltage", voltageMin, voltageMin, voltageMax, TFT_YELLOW));
    labels.push_back(createLabel("DutyCycle", dutyCycleMax, dutyCycleMin, dutyCycleMax, TFT_DARKGREY));
    labels.push_back(createLabel("DutyCycle", dutyCycleMin, dutyCycleMin, dutyCycleMax, TFT_DARKGREY));
    labels.push_back(createLabel("TempDiff", tempDiffMax, tempDiffMin, tempDiffMax, TFT_BLUE));
    labels.push_back(createLabel("TempDiff", tempDiffMin, tempDiffMin, tempDiffMax, TFT_BLUE));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMax, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_SKYBLUE));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMin, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_SKYBLUE));
    labels.push_back(createLabel("IrLU", irLU_Max, irLU_Min, irLU_Max, TFT_ORANGE));
    labels.push_back(createLabel("IrLU", irLU_Min, irLU_Min, irLU_Max, TFT_ORANGE));
    labels.push_back(createLabel("IrPairs", irPairs_Max, irPairs_Min, irPairs_Max, TFT_CYAN));
    labels.push_back(createLabel("IrPairs", irPairs_Min, irPairs_Min, irPairs_Max, TFT_CYAN));

    std::vector<Label> positionedLabels;
    std::vector<TickLabel> tickLabels;
    std::srand(std::time(nullptr)); // Seed random number generator
    int labelShiftAmount = 12; // Amount to shift label up or down

    auto placeLabelAndLine = [&](Label& label, int startX, int endX, int textX, int textY, int textDatum) {
        label.lineStartX = startX;
        label.lineEndX = endX;
        label.x = textX;
        label.y = textY;

        auto checkAllCollisions = [&](const Label& currentLabel) {
            for (const auto& placedLabel : positionedLabels) {
                if (checkCollision(currentLabel, placedLabel)) return true;
            }
            for (const auto& tickLabel : tickLabels) {
                if (checkCollision(currentLabel, tickLabel)) return true;
            }
            return false;
        };

        if (!checkAllCollisions(label)) {
            tft.drawLine(label.lineStartX, label.lineY, label.lineEndX, label.lineY, label.color);
            tft.setTextColor(label.color);
            tft.setTextDatum(textDatum);
            tft.drawString(label.text.c_str(), label.x, label.y, 1);
            positionedLabels.push_back(label);
            return true;
        } else {
            // Try shifting up
            Label shiftedLabelUp = label;
            shiftedLabelUp.y -= labelShiftAmount;
            if (shiftedLabelUp.y >= marginYTop && !checkAllCollisions(shiftedLabelUp)) {
                tft.drawLine(label.lineStartX, label.lineY, label.lineEndX, label.lineY, label.color);
                tft.drawLine(marginX - supplementaryTickLength, label.y_initial, marginX, label.y_initial, label.color); // Supplementary tick
                tft.setTextColor(shiftedLabelUp.color);
                tft.setTextDatum(textDatum);
                tft.drawString(shiftedLabelUp.text.c_str(), shiftedLabelUp.x, shiftedLabelUp.y, 1);
                positionedLabels.push_back(shiftedLabelUp);
                return true;
            }

            // Try shifting down
            Label shiftedLabelDown = label;
            shiftedLabelDown.y += labelShiftAmount;
            if (shiftedLabelDown.y + shiftedLabelDown.textHeight <= marginYTop + plotAreaHeight && !checkAllCollisions(shiftedLabelDown)) {
                tft.drawLine(label.lineStartX, label.lineY, label.lineEndX, label.lineY, label.color);
                tft.drawLine(marginX - supplementaryTickLength, label.y_initial, marginX, label.y_initial, label.color); // Supplementary tick
                tft.setTextColor(shiftedLabelDown.color);
                tft.setTextDatum(textDatum);
                tft.drawString(shiftedLabelDown.text.c_str(), shiftedLabelDown.x, shiftedLabelDown.y, 1);
                positionedLabels.push_back(shiftedLabelDown);
                return true;
            }
        }
        return false;
    };

    std::vector<LinePlacement> placements = {
        LinePlacement::RIGHT,
        LinePlacement::LEFT,
        LinePlacement::BEGINNING,
        LinePlacement::END,
        LinePlacement::CENTER,
        LinePlacement::FRACTION_LEFT_1,
        LinePlacement::FRACTION_LEFT_2,
        LinePlacement::FRACTION_RIGHT_1,
        LinePlacement::FRACTION_RIGHT_2
    };
    std::vector<LabelVerticalPlacement> verticalPlacements = {LabelVerticalPlacement::CENTER, LabelVerticalPlacement::ABOVE, LabelVerticalPlacement::BELOW};

    for (auto& label : labels) {
        bool placed = false;
        std::string paramName;
        if (label.text.find("Current") != std::string::npos) paramName = "Current";
        else if (label.text.find("Voltage") != std::string::npos) paramName = "Voltage";
        else if (label.text.find("DutyCycle") != std::string::npos) paramName = "Duty";
        else if (label.text.find("TempDiff") != std::string::npos) paramName = "TempDiff";
        else if (label.text.find("EstTempDiffThreshold") != std::string::npos) paramName = "TempDiffThresh";
        else if (label.text.find("IrLU") != std::string::npos) paramName = "IR_LU";
        else if (label.text.find("IrPairs") != std::string::npos) paramName = "IR_Pairs";

        int dynamicLineLength = tickLength;
        if (!paramName.empty() && hardcodedRanges.count(paramName)) {
            float value;
            if (sscanf(label.text.c_str(), "%f", &value) == 1) {
                dynamicLineLength = calculateDynamicLineLength(value, hardcodedRanges[paramName].first, hardcodedRanges[paramName].second);
            }
        }

        for (const auto& placement : placements) {
            for (const auto& vPlacement : verticalPlacements) {
                if (placed) break;
                int startX = 0;
                int endX = 0;
                int textX = 0;
                int textY = 0;
                int textDatum = 0; // Placeholder

                switch (placement) {
                    case LinePlacement::RIGHT:
                        startX = marginX + plotAreaWidth - dynamicLineLength;
                        endX = marginX + plotAreaWidth;
                        textX = startX - textSpacing;
                        textDatum = 4; // TR_DATUM
                        break;
                    case LinePlacement::LEFT:
                        startX = marginX;
                        endX = marginX + dynamicLineLength;
                        textX = endX + textSpacing;
                        textDatum = 2; // TL_DATUM
                        break;
                    case LinePlacement::BEGINNING: // Similar to LEFT
                        startX = marginX;
                        endX = marginX + dynamicLineLength;
                        textX = endX + textSpacing;
                        textDatum = 2; // TL_DATUM
                        break;
                    case LinePlacement::END: // Similar to RIGHT
                        startX = marginX + plotAreaWidth - dynamicLineLength;
                        endX = marginX + plotAreaWidth;
                        textX = startX - textSpacing;
                        textDatum = 4; // TR_DATUM
                        break;
                    case LinePlacement::CENTER:
                        startX = marginX + plotAreaWidth / 2 - dynamicLineLength / 2;
                        endX = marginX + plotAreaWidth / 2 + dynamicLineLength / 2;
                        if (std::rand() % 2 == 0) { // Try left or right side randomly
                            textX = endX + textSpacing;
                            textDatum = 2; // TL_DATUM
                        } else {
                            textX = startX - textSpacing;
                            textDatum = 4; // TR_DATUM
                        }
                        break;
                    case LinePlacement::FRACTION_LEFT_1:
                        startX = marginX + static_cast<int>(plotAreaWidth * 0.15f);
                        endX = startX + dynamicLineLength;
                        textX = endX + textSpacing;
                        textDatum = 2; // TL_DATUM
                        break;
                    case LinePlacement::FRACTION_LEFT_2:
                        startX = marginX + static_cast<int>(plotAreaWidth * 0.3f);
                        endX = startX + dynamicLineLength;
                        textX = endX + textSpacing;
                        textDatum = 2; // TL_DATUM
                        break;
                    case LinePlacement::FRACTION_RIGHT_1:
                        startX = marginX + static_cast<int>(plotAreaWidth * 0.7f) - dynamicLineLength;
                        endX = startX + dynamicLineLength;
                        textX = startX - textSpacing;
                        textDatum = 4; // TR_DATUM
                        break;
                    case LinePlacement::FRACTION_RIGHT_2:
                        startX = marginX + static_cast<int>(plotAreaWidth * 0.85f) - dynamicLineLength;
                        endX = startX + dynamicLineLength;
                        textX = startX - textSpacing;
                        textDatum = 4; // TR_DATUM
                        break;
                }

                switch (vPlacement) {
                    case LabelVerticalPlacement::CENTER:
                        textY = label.y_initial - label.textHeight / 2;
                        break;
                    case LabelVerticalPlacement::ABOVE:
                        textY = label.y_initial - label.textHeight - textSpacing;
                        break;
                    case LabelVerticalPlacement::BELOW:
                        textY = label.y_initial + textSpacing;
                        break;
                }

                if (placeLabelAndLine(label, startX, endX, textX, textY, textDatum)) {
                    placed = true;
                }
            }
            if (placed) break;
        }
    }

    // Draw time axis labels
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(2); // TL_DATUM
    if (autoscaleX && endTime > startTime) {
        for (int i = 0; i <= 5; ++i) {
            unsigned long timePoint = startTime + (endTime - startTime) * i / 5;
            time_t t = timePoint / 1000;
            struct tm* tm_info = localtime(&t);
            char buffer[6];
            strftime(buffer, sizeof(buffer), "%H:%M", tm_info);
            int x = marginX + plotAreaWidth * i / 5;
            tft.drawLine(x, marginYTop, x, marginYTop - 5, TFT_DARKGREY);
            tft.drawString(buffer, x, marginYTop - 20, 1);
        }
    }

    // Draw Legend
    int legendY = marginYTop + plotAreaHeight + 10;
    int legendX = marginX;
    int colorSize = 9;
    int textOffset = 15;
    tft.setTextDatum(2); // TL_DATUM
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_MAGENTA);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("I", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_YELLOW);
    tft.drawString("V", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_DARKGREY);
    tft.drawString("%", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_BLUE);
    tft.drawString("dT", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_SKYBLUE);
    tft.drawString("dT/", legendX + textOffset, legendY, 1);

    legendX = marginX;
    legendY += 12;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_ORANGE);
    tft.drawString("iR(L/UL)", legendX + textOffset, legendY, 1);
    legendX += 90;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_CYAN);
    tft.drawString("iR(Pairs)", legendX + textOffset, legendY, 1);
}

/**
 * Main charging function - starts, monitors, and stops battery charging
 * Returns true if charging is in progress, false if charging has stopped
 */
uint32_t chargingStartTime = 0 ;
bool chargeBattery() {
//    static bool isCharging = false; // Make isCharging static to persist between calls
//    static unsigned long lastChargeEvaluationTime = 0;
    static int lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    static int chargingDutyCycle = MAX_CHARGE_DUTY_CYCLE; // Initialize with max for the first run
    tft.setTextColor(TFT_RED,TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("CHARGING");

    // If not currently charging, start the charging process
    if (!isCharging) {
        Serial.println("Starting battery charging process...");
        isCharging = true;
        chargingStartTime = millis(); // Record the start time
        lastChargeEvaluationTime = millis();
        lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE; // Reset to max on new charging start

        // Find the initial optimal charging duty cycle
        chargingDutyCycle = findOptimalChargingDutyCycle(MAX_CHARGE_DUTY_CYCLE, MIN_CHARGE_DUTY_CYCLE);
        lastOptimalDutyCycle = chargingDutyCycle; // Store the initial optimal duty cycle

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

        // Log the initial state
        unsigned long currentTimestamp = millis();
        ChargeLogData logEntry;
        logEntry.timestamp = currentTimestamp;
        logEntry.current = initialData.current;
        logEntry.voltage = initialData.voltage;
        logEntry.ambientTemperature = initialData.temp1; // ambient temperature
        logEntry.batteryTemperature = initialData.temp2; // battery temperature
        logEntry.dutyCycle = chargingDutyCycle;
        logEntry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
        logEntry.internalResistancePairs = regressedInternalResistancePairsIntercept;
        chargeLog.push_back(logEntry);

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
//    float temp1, temp2, tempDiff, t1_millivolts, voltage, current;
    double temp1, temp2, tempDiff;
    float t1_millivolts;
    float voltage;
    float current;

    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    
    float tempRise = estimateTempDiff(voltage, voltage, current, (regressedInternalResistanceIntercept), temp1); // if 0 then no power
  // resistance of MH electrode sums with internal resistance. 
    Serial.print("Estimated temperature rise due to Rint heating: ");
    Serial.print(tempRise);
    Serial.println(" °C");
    
    // Check if temperature difference exceeds threshold - stop charging if it does
    if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD+tempRise)) {
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
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        delay(UNLOADED_VOLTAGE_DELAY_MS);

        // Calculate the starting duty cycle for the next evaluation
        int suggestedStartDutyCycle = min(lastOptimalDutyCycle + (int)(1.0 * lastOptimalDutyCycle), MAX_CHARGE_DUTY_CYCLE);

        // Find the optimal charging duty cycle again, starting from the suggested value
//        chargingDutyCycle = findOptimalChargingDutyCycle(MAX_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
        chargingDutyCycle = findOptimalChargingDutyCycle(suggestedStartDutyCycle,MIN_CHARGE_DUTY_CYCLE);

        lastOptimalDutyCycle = chargingDutyCycle; // Update the last optimal duty cycle

        // Apply the updated charging current
        dutyCycle = chargingDutyCycle;
        analogWrite(pwmPin, chargingDutyCycle);

        lastChargeEvaluationTime = currentTime;

        Serial.printf("Charging parameters updated - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                      chargingDutyCycle, current, temp1, temp2, tempDiff);

        // Log the charging parameters after re-evaluation
        unsigned long currentTimestamp = millis();
        ChargeLogData logEntry;
        logEntry.timestamp = currentTimestamp;
        logEntry.current = current;
        logEntry.voltage = voltage;
        // Assuming temp2 is battery temperature and we need a way to get ambient temperature
        logEntry.ambientTemperature = temp1; // Placeholder for ambient temperature - you need to replace this
        logEntry.batteryTemperature = temp2;
        logEntry.dutyCycle = chargingDutyCycle;
        logEntry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
        logEntry.internalResistancePairs = regressedInternalResistancePairsIntercept;
        chargeLog.push_back(logEntry);
    }

    // Display charging status on TFT
//    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
//    tft.printf("DC: %d, I: %.2fA", chargingDutyCycle, current);
//    tft.setCursor(14*10, PLOT_Y_START + PLOT_HEIGHT + 40);
//    tft.printf("T1: %.2fC, T2: %.2fC, dT: %.2fC", temp1, temp2, tempDiff);

    return true; // Charging is still in progress
}



/**
 * Start charging with a single call
 */
void startCharging() {
  if (!isCharging) {
    Serial.println("Initiating battery charging...");
    tft.setTextColor(TFT_RED,TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("CHARGING");
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
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("STOPPED");
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
      tft.setTextSize(1);
      tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
      tft.printf("COMPLETE");
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
    plotVoltageData();
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

        case RemoteKeys::KEY_SOURCE:{        
          drawChargePlot(true,true); // Display the charging process log graph
          delay(10000); // wait 10 seconds
          tft.fillScreen(TFT_BLACK); // clear junk afterwards

        break;
        }    

        case RemoteKeys::KEY_POWER:{        
        resetAh = true; // reset mAh counter
        buildCurrentModel(false); // Build the model from scratch
        startCharging(); // slow charge battery
        break;
        }    
  
               
        }// switch
            }//address 7
      }//samsung
}// handle IR
