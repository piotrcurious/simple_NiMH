#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric> // For accumulate

// ... (rest of your includes and external variable declarations)

namespace {
// Charging parameters
const int MIN_CHARGE_DUTY_CYCLE = 50; // Adjust as needed
const int MAX_CHARGE_DUTY_CYCLE = 255; // Adjust as needed
const unsigned long CHARGE_EVALUATION_INTERVAL_MS = 5000; // Re-evaluate voltage every 5 seconds
const unsigned long CHARGE_STABILIZATION_DELAY_MS = 200; // Delay for voltage to stabilize during charging
const int SATURATION_DUTY_CYCLE = 200; // Duty cycle at which MH electrode is assumed to be saturated (adjust based on your battery)
int HIGH_CHARGE_DUTY_CYCLE = 0;
const int BINARY_SEARCH_ITERATIONS = 10; // Number of iterations for binary search

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

// Function to estimate saturation duty cycle (same as before)
int estimateSaturationDutyCycle() {
    // ... (same implementation as in the previous response)
}

bool isCharging = false;
}

// ... (measureInternalResistanceLoadedUnloaded and measureInternalResistancePairs functions - same as before)

void chargeBattery() {
    if (isCharging) {
        Serial.println("Charging is already in progress.");
        return;
    }
    isCharging = true;
    Serial.println("Starting battery charging with estimated saturation and binary search...");

    if (unloadedVoltagesHistory.empty()) {
        Serial.println("Error: Unloaded voltage history is empty. Run internal resistance measurement first.");
        isCharging = false;
        return;
    }
    float initialUnloadedVoltage = unloadedVoltagesHistory.back();
    Serial.printf("Initial Unloaded Voltage (from history): %.3f V\n", initialUnloadedVoltage);

    int saturationDutyCycle = estimateSaturationDutyCycle();
    HIGH_CHARGE_DUTY_CYCLE = (MAX_CHARGE_DUTY_CYCLE + saturationDutyCycle) / 2;
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

            // d. Find the charging current (duty cycle) using binary search
            int lowDutyCycle = MIN_CHARGE_DUTY_CYCLE;
            int highDutyCycle = MAX_CHARGE_DUTY_CYCLE;
            bestChargeDutyCycle = -1;
            float bestVoltageDifference = 1000.0f;

            for (int i = 0; i < BINARY_SEARCH_ITERATIONS; ++i) {
                int midDutyCycle = lowDutyCycle + (highDutyCycle - lowDutyCycle) / 2;
                analogWrite(pwmPin, midDutyCycle);
                delay(CHARGE_STABILIZATION_DELAY_MS);
                MeasurementData loadedData;
                getThermistorReadings(loadedData.temp1, loadedData.temp2, loadedData.tempDiff, loadedData.t1_millivolts, loadedData.voltage, loadedData.current);
                float voltageDifference = loadedData.voltage - targetVoltage;

                Serial.printf("Binary Search Iteration %d: Duty Cycle: %d, Loaded Voltage: %.3f V, Target: %.3f V, Diff: %.3f V\n", i, midDutyCycle, loadedData.voltage, targetVoltage, voltageDifference);

                if (std::abs(voltageDifference) < bestVoltageDifference) {
                    bestVoltageDifference = std::abs(voltageDifference);
                    bestChargeDutyCycle = midDutyCycle;
                }

                if (voltageDifference > 0) {
                    // Loaded voltage is higher than target, need more current (higher duty cycle)
                    lowDutyCycle = midDutyCycle + 1;
                } else {
                    // Loaded voltage is lower than target, need less current (lower duty cycle)
                    highDutyCycle = midDutyCycle - 1;
                }
            }

            if (bestChargeDutyCycle != -1) {
                Serial.printf("Setting charging duty cycle to: %d (best from binary search).\n", bestChargeDutyCycle);
                analogWrite(pwmPin, bestChargeDutyCycle); // Set the best charging current
            } else {
                Serial.println("Binary search failed to find a suitable duty cycle.");
            }

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
