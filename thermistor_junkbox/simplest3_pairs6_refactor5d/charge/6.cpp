#include <vector>
#include <algorithm>
#include <cmath>

// Assuming these are defined elsewhere in your code
extern int pwmPin;
extern int dutyCycle;
extern void analogWrite(int pin, int value);
extern void delay(unsigned long ms);
extern unsigned long millis();

// Assuming these structures and functions are defined elsewhere
struct MeasurementData {
    float temp1;
    float temp2;
    float tempDiff;
    float t1_millivolts;
    float voltage;
    float current;
    int dutyCycle;
    unsigned long timestamp;
};
extern void getThermistorReadings(float& temp1, float& temp2, float& tempDiff, float& t1_millivolts, float& voltage, float& current);
extern void processThermistorData(const MeasurementData& data, const char* prefix);
extern void bubbleSort(std::vector<DataPoint>& data, int& count);
extern void calculateAverageInternalResistance(const std::vector<float>& resistances);
extern void performLinearRegression(const std::vector<float>& x, const std::vector<float>& y);
extern void storeResistanceData(float current, float resistance, std::vector<DataPoint>& data, int& count);
extern int findMinimalDutyCycle();
extern std::vector<std::pair<int, int>> generateDutyCyclePairs(int minDutyCycle);
extern bool isMeasuringResistance;
extern std::vector<DataPoint> internalResistanceData;
extern int resistanceDataCount;
extern std::vector<DataPoint> internalResistanceDataPairs;
extern int resistanceDataCountPairs;
extern float voltage_mv;
extern unsigned long voltage_last_time;
extern unsigned long voltage_update_interval;
extern float current_ma;
extern float MAIN_VCC_RATIO;
extern float CURRENT_SHUNT_RESISTANCE;
extern float CURRENT_SHUNT_PIN_ZERO_OFFSET;
extern int VOLTAGE_READ_PIN;
extern int CURRENT_SHUNT_PIN;
extern float UNLOADED_VOLTAGE_DELAY_MS;
extern float STABILIZATION_DELAY_MS;
extern float MIN_CURRENT_DIFFERENCE_FOR_PAIR;
extern void task_readThermistor(void* parameter);
extern int VOLTAGE_ATTENUATION;
extern int VOLTAGE_OVERSAMPLING;
extern int CURRENT_SHUNT_ATTENUATION;
extern int CURRENT_SHUNT_OVERSAMPLING;
extern std::vector<float> unloadedVoltagesHistory; // Assuming this is a global variable updated by measureInternalResistance
extern float averageInternalResistance; // Assuming this is a global variable calculated by calculateAverageInternalResistance

// Assuming tft is a display object if used
// extern Adafruit_GFX tft;
// extern int PLOT_X_START;
// extern int PLOT_Y_START;
// extern int PLOT_HEIGHT;

namespace {
// Charging parameters
const int MIN_CHARGE_DUTY_CYCLE = 50; // Adjust as needed
const int MAX_CHARGE_DUTY_CYCLE = 255; // Adjust as needed
const int NUM_CHARGE_STEPS = 10; // Number of steps to try for finding optimal current
const unsigned long CHARGE_EVALUATION_INTERVAL_MS = 5000; // Re-evaluate voltage every 5 seconds
const unsigned long CHARGE_STABILIZATION_DELAY_MS = 200; // Delay for voltage to stabilize during charging
const int INITIAL_CHARGE_DUTY_CYCLE = 100; // Initial duty cycle to get a loaded voltage reading

bool isCharging = false;
}

void chargeBattery() {
    if (isCharging) {
        Serial.println("Charging is already in progress.");
        return;
    }
    isCharging = true;
    Serial.println("Starting battery charging using internal resistance data...");

    if (unloadedVoltagesHistory.empty()) {
        Serial.println("Error: Unloaded voltage history is empty. Run internal resistance measurement first.");
        isCharging = false;
        return;
    }
    float initialUnloadedVoltage = unloadedVoltagesHistory.back();
    Serial.printf("Initial Unloaded Voltage (from history): %.3f V\n", initialUnloadedVoltage);

    if (averageInternalResistance < 0) {
        Serial.println("Warning: Average internal resistance is not valid. Charging might not be optimal.");
    } else {
        Serial.printf("Average Internal Resistance (from measurement): %.3f Ohm\n", averageInternalResistance);
    }

    float targetVoltage;
    int bestChargeDutyCycle = 0;
    float currentTemperatureDifference;
    unsigned long lastEvaluationTime = 0;

    // Apply an initial charging current to get a loaded voltage reading
    Serial.printf("Applying initial charging current (Duty Cycle: %d) to estimate loaded voltage...\n", INITIAL_CHARGE_DUTY_CYCLE);
    analogWrite(pwmPin, INITIAL_CHARGE_DUTY_CYCLE);
    delay(CHARGE_STABILIZATION_DELAY_MS * 5);
    MeasurementData initialLoadData;
    getThermistorReadings(initialLoadData.temp1, initialLoadData.temp2, initialLoadData.tempDiff, initialLoadData.t1_millivolts, initialLoadData.voltage, initialLoadData.current);
    float initialLoadedVoltage = initialLoadData.voltage;
    float initialChargingCurrent = initialLoadData.current;
    Serial.printf("Voltage under initial charging current: %.3f V, Current: %.3f A\n", initialLoadedVoltage, initialChargingCurrent);

    // Calculate the initial target "half voltage"
    targetVoltage = (initialUnloadedVoltage + initialLoadedVoltage) / 2.0f;
    Serial.printf("Initial Target Voltage (midpoint): %.3f V\n", targetVoltage);

    // Main charging loop
    while (true) {
        // Periodically re-evaluate MH electrode voltage and adjust charging current
        if (millis() - lastEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
            // a. Measure current unloaded voltage
            MeasurementData currentUnloaded = getUnloadedVoltageMeasurement();
            float currentUnloadedVoltage = currentUnloaded.voltage;
            Serial.printf("Current Unloaded Voltage: %.3f V\n", currentUnloadedVoltage);

            // b. Apply a moderate charging current to get a loaded voltage reading for midpoint calculation
            Serial.printf("Applying moderate charging current (Duty Cycle: %d) for updated reference voltage...\n", INITIAL_CHARGE_DUTY_CYCLE);
            analogWrite(pwmPin, INITIAL_CHARGE_DUTY_CYCLE);
            delay(CHARGE_STABILIZATION_DELAY_MS * 5);
            MeasurementData currentLoadData;
            getThermistorReadings(currentLoadData.temp1, currentLoadData.temp2, currentLoadData.tempDiff, currentLoadData.t1_millivolts, currentLoadData.voltage, currentLoadData.current);
            float currentLoadedVoltage = currentLoadData.voltage;
            Serial.printf("Voltage under moderate charging current: %.3f V, Current: %.3f A\n", currentLoadedVoltage, currentLoadData.current);

            // c. Recalculate the target "half voltage"
            targetVoltage = (currentUnloadedVoltage + currentLoadedVoltage) / 2.0f;
            Serial.printf("Updated Target Voltage (midpoint): %.3f V\n", targetVoltage);

            // d. Find the charging current (duty cycle) to achieve the target voltage
            float bestVoltageDifference = 1000.0f; // Initialize with a large value

            for (int dutyCycle = MIN_CHARGE_DUTY_CYCLE; dutyCycle <= MAX_CHARGE_DUTY_CYCLE; dutyCycle += (MAX_CHARGE_DUTY_CYCLE - MIN_CHARGE_DUTY_CYCLE) / (NUM_CHARGE_STEPS - 1 > 0 ? (NUM_CHARGE_STEPS - 1) : 1)) {
                dutyCycle = constrain(dutyCycle, MIN_CHARGE_DUTY_CYCLE, MAX_CHARGE_DUTY_CYCLE);
                analogWrite(pwmPin, dutyCycle); // Apply charging current
                delay(CHARGE_STABILIZATION_DELAY_MS);
                MeasurementData loadedData;
                getThermistorReadings(loadedData.temp1, loadedData.temp2, loadedData.tempDiff, loadedData.t1_millivolts, loadedData.voltage, loadedData.current);
                float voltageDifference = std::abs(loadedData.voltage - targetVoltage);

                Serial.printf("Trying Duty Cycle: %d, Loaded Voltage: %.3f V, Target: %.3f V, Diff: %.3f V\n", dutyCycle, loadedData.voltage, targetVoltage, voltageDifference);

                if (voltageDifference < bestVoltageDifference) {
                    bestVoltageDifference = voltageDifference;
                    bestChargeDutyCycle = dutyCycle;
                }
            }
            Serial.printf("Setting charging duty cycle to: %d to maintain target voltage.\n", bestChargeDutyCycle);
            analogWrite(pwmPin, bestChargeDutyCycle); // Set the best charging current
            lastEvaluationTime = millis();
        }

        // Check temperature difference
        MeasurementData tempData;
        getThermistorReadings(tempData.temp1, tempData.temp2, tempData.tempDiff, tempData.t1_millivolts, tempData.voltage, tempData.current);
        currentTemperatureDifference = tempData.tempDiff;
        Serial.printf("Current Temperature Difference: %.3f C\n", currentTemperatureDifference);

        if (currentTemperatureDifference > 0.8f) {
            Serial.println("Temperature difference exceeded 0.8C. Stopping charging.");
            break;
        }

        delay(1000); // Small delay to avoid busy-waiting
    }

    // Stop charging
    analogWrite(pwmPin, 0);
    isCharging = false;
    Serial.println("Battery charging complete.");
}

// In your main loop, you should call measureInternalResistance() before potentially calling chargeBattery()
// void loop() {
//     measureInternalResistance(); // Measure internal resistance before charging
//     if (/* some condition to start charging */) {
//         chargeBattery();
//     }
//     // ... your other code ...
// }
