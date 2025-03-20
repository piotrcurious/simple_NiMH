#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric> // For accumulate

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

// Modified DataPoint structure to include dutyCycle
struct DataPoint {
    float current;
    float resistance;
    int dutyCycle;
};

// Helper function to take a measurement and return the duty cycle
MeasurementData takeMeasurement(int dc, uint32_t stabilization_delay) {
    dutyCycle = dc;
    analogWrite(pwmPin, dc);
    delay(stabilization_delay);
    MeasurementData data;
    getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    data.dutyCycle = dc;
    data.timestamp = millis();
    return data;
}

// Helper function to store resistance data with duty cycle
void storeResistanceData(float current, float resistance, int dc, std::vector<DataPoint>& data, int& count) {
    if (count < data.size()) {
        data[count] = {current, resistance, dc};
        count++;
    } else {
        data.push_back({current, resistance, dc});
        count++;
    }
}

// Function to estimate saturation duty cycle
int estimateSaturationDutyCycle() {
    if (internalResistanceDataPairs.size() < 2) {
        Serial.println("Not enough data to estimate saturation duty cycle (pairs).");
        return MAX_CHARGE_DUTY_CYCLE; // Default to max if not enough data
    }
    if (internalResistanceData.size() < 2) {
        Serial.println("Not enough data to estimate saturation duty cycle (loaded/unloaded).");
        return MAX_CHARGE_DUTY_CYCLE; // Default to max if not enough data
    }

    float resistanceThresholdMultiplier = 1.5f; // Threshold for considering a significant increase
    int potentialSaturationDutyCycle = MAX_CHARGE_DUTY_CYCLE;

    for (size_t i = 1; i < internalResistanceDataPairs.size(); ++i) {
        float currentPair = internalResistanceDataPairs[i].current;
        float resistancePair = internalResistanceDataPairs[i].resistance;
        int dutyCyclePair = internalResistanceDataPairs[i].dutyCycle;

        // Find a corresponding loaded/unloaded data point (approximate current match)
        for (const auto& luData : internalResistanceData) {
            if (std::abs(luData.current - currentPair) < 0.02f) { // Tolerance for current matching
                if (resistancePair > luData.resistance * resistanceThresholdMultiplier && resistancePair > 0.01f && luData.resistance > 0.005f) {
                    potentialSaturationDutyCycle = dutyCyclePair;
                    Serial.printf("Potential saturation detected at Duty Cycle: %d (Pair Resistance: %.3f, LU Resistance: %.3f)\n", dutyCyclePair, resistancePair, luData.resistance);
                    return potentialSaturationDutyCycle; // Return the first potential saturation point
                }
                break; // Move to the next pair data point
            }
        }
    }

    Serial.printf("Saturation duty cycle not explicitly detected, using MAX_CHARGE_DUTY_CYCLE as fallback (%d).\n", MAX_CHARGE_DUTY_CYCLE);
    return MAX_CHARGE_DUTY_CYCLE; // Fallback if no clear saturation point is found
}

bool isCharging = false;
}

// Modified measureInternalResistanceLoadedUnloaded to use the updated takeMeasurement and storeResistanceData
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
            storeResistanceData(loadedData.current, std::abs(internalResistance), dc, internalResistanceData, resistanceDataCount);
            Serial.printf("Calculated Internal Resistance (Loaded-Unloaded): %.3f Ohm\n", std::abs(internalResistance));
            // if (tft) {
            //     tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            //     tft.printf("(L/UL): %.3f ", std::abs(internalResistance));
            // }
        } else {
            Serial.println("Warning: Current is too low to reliably calculate internal resistance (Loaded-Unloaded).");
            storeResistanceData(loadedData.current, -1.0f, dc, internalResistanceData, resistanceDataCount); // Indicate invalid
        }
        Serial.println("---------------------------\n");
    }
}

// Modified measureInternalResistancePairs to use the updated takeMeasurement and storeResistanceData
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
            consecutiveInternalResistances.push_back(std::abs(internalResistanceConsecutive));
            storeResistanceData(highData.current, std::abs(internalResistanceConsecutive), dcHigh, internalResistanceDataPairs, resistanceDataCountPairs);
            Serial.printf("Calculated Internal Resistance (Pair): %.3f Ohm\n", std::abs(internalResistanceConsecutive));
            // if (tft) {
            //     tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 50);
            //     tft.printf("(Pair): %.3f ", std::abs(internalResistanceConsecutive));
            // }
        } else {
            Serial.println("Warning: Current difference is too small to reliably calculate internal resistance (Pair).");
            consecutiveInternalResistances.push_back(-1.0f); // Indicate invalid
            storeResistanceData(highData.current, -1.0f, dcHigh, internalResistanceDataPairs, resistanceDataCountPairs);
        }
        Serial.println("---------------------------\n");
    }
}

void chargeBattery() {
    if (isCharging) {
        Serial.println("Charging is already in progress.");
        return;
    }
    isCharging = true;
    Serial.println("Starting battery charging with estimated saturation duty cycle...");

    if (unloadedVoltagesHistory.empty()) {
        Serial.println("Error: Unloaded voltage history is empty. Run internal resistance measurement first.");
        isCharging = false;
        return;
    }
    float initialUnloadedVoltage = unloadedVoltagesHistory.back();
    Serial.printf("Initial Unloaded Voltage (from history): %.3f V\n", initialUnloadedVoltage);

    int saturationDutyCycle = estimateSaturationDutyCycle();
    int HIGH_CHARGE_DUTY_CYCLE = (MAX_CHARGE_DUTY_CYCLE + saturationDutyCycle) / 2;
    Serial.printf("Estimated Saturation Duty Cycle: %d, High Charging Duty Cycle: %d\n", saturationDutyCycle, HIGH_CHARGE_DUTY_CYCLE);

    float targetVoltage;
    int bestChargeDutyCycle = 0;
    float currentTemperatureDifference;
    unsigned long lastEvaluationTime = 0;

    // Apply the determined high charging current and measure loaded voltage
    Serial.printf("Applying high charging current (Duty Cycle: %d) for reference voltage...\n", HIGH_CHARGE_DUTY_CYCLE);
    analogWrite(pwmPin, HIGH_CHARGE_DUTY_CYCLE);
    delay(CHARGE_STABILIZATION_DELAY_MS * 5); // Allow more time for stabilization
    MeasurementData highLoadData;
    getThermistorReadings(highLoadData.temp1, highLoadData.temp2, highLoadData.tempDiff, highLoadData.t1_millivolts, highLoadData.voltage, highLoadData.current);
    float highCurrentLoadedVoltage = highLoadData.voltage;
    Serial.printf("Voltage under high charging current: %.3f V\n", highCurrentLoadedVoltage);

    // Calculate the initial target "half voltage"
    targetVoltage = (initialUnloadedVoltage + highCurrentLoadedVoltage) / 2.0f;
    Serial.printf("Initial Target Voltage (midpoint): %.3f V\n", targetVoltage);

    // Main charging loop
    while (true) {
        // Periodically re-evaluate MH electrode voltage and adjust charging current
        if (millis() - lastEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
            // a. Measure current unloaded voltage
            MeasurementData currentUnloaded = getUnloadedVoltageMeasurement();
            float currentUnloadedVoltage = currentUnloaded.voltage;
            Serial.printf("Current Unloaded Voltage: %.3f V\n", currentUnloadedVoltage);

            // b. Apply high charging current and measure loaded voltage again
            Serial.printf("Applying high charging current (Duty Cycle: %d) for updated reference voltage...\n", HIGH_CHARGE_DUTY_CYCLE);
            analogWrite(pwmPin, HIGH_CHARGE_DUTY_CYCLE);
            delay(CHARGE_STABILIZATION_DELAY_MS * 5);
            MeasurementData currentHighLoadData;
            getThermistorReadings(currentHighLoadData.temp1, currentHighLoadData.temp2, currentHighLoadData.tempDiff, currentHighLoadData.t1_millivolts, currentHighLoadData.voltage, currentHighLoadData.current);
            float currentHighCurrentLoadedVoltage = currentHighLoadData.voltage;
            Serial.printf("Voltage under high charging current: %.3f V\n", currentHighCurrentLoadedVoltage);

            // c. Recalculate the target "half voltage"
            targetVoltage = (currentUnloadedVoltage + currentHighCurrentLoadedVoltage) / 2.0f;
            Serial.printf("Updated Target Voltage (midpoint): %.3f V\n", targetVoltage);

            // d. Find the charging current (duty cycle) to achieve the target voltage
            float bestVoltageDifference = 1000.0f; // Initialize with a large value

            for (int dc = MIN_CHARGE_DUTY_CYCLE; dc <= MAX_CHARGE_DUTY_CYCLE; dc += (MAX_CHARGE_DUTY_CYCLE - MIN_CHARGE_DUTY_CYCLE) / (NUM_CHARGE_STEPS - 1 > 0 ? (NUM_CHARGE_STEPS - 1) : 1)) {
                dc = constrain(dc, MIN_CHARGE_DUTY_CYCLE, MAX_CHARGE_DUTY_CYCLE);
                analogWrite(pwmPin, dc); // Apply charging current
                delay(CHARGE_STABILIZATION_DELAY_MS);
                MeasurementData loadedData;
                getThermistorReadings(loadedData.temp1, loadedData.temp2, loadedData.tempDiff, loadedData.t1_millivolts, loadedData.voltage, loadedData.current);
                float voltageDifference = std::abs(loadedData.voltage - targetVoltage);

                Serial.printf("Trying Duty Cycle: %d, Loaded Voltage: %.3f V, Target: %.3f V, Diff: %.3f V\n", dc, loadedData.voltage, targetVoltage, voltageDifference);

                if (voltageDifference < bestVoltageDifference) {
                    bestVoltageDifference = voltageDifference;
                    bestChargeDutyCycle = dc;
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
