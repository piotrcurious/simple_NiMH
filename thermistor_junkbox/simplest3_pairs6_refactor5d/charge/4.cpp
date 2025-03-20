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

bool isCharging = false;
}

void chargeBattery() {
    if (isCharging) {
        Serial.println("Charging is already in progress.");
        return;
    }
    isCharging = true;
    Serial.println("Starting battery charging...");

    float initialUnloadedVoltage;
    float currentUnloadedVoltage;
    float targetVoltage;
    int bestChargeDutyCycle = 0;
    float currentTemperatureDifference;
    unsigned long lastEvaluationTime = 0;

    // Measure initial unloaded voltage
    MeasurementData unloadedData = getUnloadedVoltageMeasurement();
    initialUnloadedVoltage = unloadedData.voltage;
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloadedVoltage);

    // Main charging loop
    while (true) {
        // Periodically re-evaluate MH electrode voltage
        if (millis() - lastEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
            MeasurementData currentUnloaded = getUnloadedVoltageMeasurement();
            currentUnloadedVoltage = currentUnloaded.voltage;
            Serial.printf("Current Unloaded Voltage: %.3f V\n", currentUnloadedVoltage);
            targetVoltage = currentUnloadedVoltage / 2.0f;
            Serial.printf("Target Voltage (50%% of unloaded): %.3f V\n", targetVoltage);

            // Find the charging current that keeps the MH electrode voltage at half value
            float bestVoltageDifference = 1000.0f; // Initialize with a large value

            for (int dutyCycle = MIN_CHARGE_DUTY_CYCLE; dutyCycle <= MAX_CHARGE_DUTY_CYCLE; dutyCycle += (MAX_CHARGE_DUTY_CYCLE - MIN_CHARGE_DUTY_CYCLE) / (NUM_CHARGE_STEPS -1 > 0 ? (NUM_CHARGE_STEPS - 1) : 1)) {
                dutyCycle = constrain(dutyCycle, MIN_CHARGE_DUTY_CYCLE, MAX_CHARGE_DUTY_CYCLE);
                analogWrite(pwmPin, dutyCycle); // Apply charging current (assuming pwmPin controls charging)
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
            Serial.printf("Setting charging duty cycle to: %d\n", bestChargeDutyCycle);
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

// You would call this function from your main loop or another appropriate place
// void loop() {
//     // ... your other code ...
//     if (/* some condition to start charging */) {
//         chargeBattery();
//     }
//     // ... your other code ...
// }
