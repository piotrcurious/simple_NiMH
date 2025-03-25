#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream> // For Serial (assuming Arduino context)

// --- Configuration ---
const float ELECTROLYSIS_THRESHOLD_VOLTAGE = 2.3f; // Example value, adjust based on battery chemistry
const float MIN_CHARGE_DUTY_CYCLE = 0;
const float MAX_CHARGE_DUTY_CYCLE = 255;
const int CHARGE_CURRENT_STEP = 5;
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.05f;
const float MH_ELECTRODE_RATIO = 1.0f; // Adjust if needed

// --- Data Structures ---
struct MHElectrodeData {
    int dutyCycle;
    float loadedVoltage;
    float unloadedVoltage;
    float current;
};

// --- Forward Declarations (Assuming these functions exist elsewhere) ---
MHElectrodeData measureMHElectrodeVoltage(int dutyCycle);
void addToCache(const MHElectrodeData& data);
void storeOrAverageResistanceData(float current, float resistance, float* dataArray, int& count);
void bubbleSort(float* dataArray, int count);
void distribute_error(float* dataArray, int count, float minFactor, float maxFactor);
bool performLinearRegression(const float* dataX, int count, float& slope, float& intercept);
float getChargingVoltage(int dutyCycle); // Assume this function exists to get the charging voltage for a given duty cycle

// --- Global Variables (Consider making these local if possible) ---
std::vector<MHElectrodeData> dataCache;
float internalResistanceData[10]; // Example size
int resistanceDataCount = 0;
float internalResistanceDataPairs[10]; // Example size
int resistanceDataCountPairs = 0;
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;

int findOptimalChargeDutyCycle(float initialUnloadedVoltage, float targetVoltage) {
    int optimalDutyCycle = MIN_CHARGE_DUTY_CYCLE;
    float closestVoltageDifference = 1000.0f;
    MHElectrodeData previousData = {0, 0.0f, 0.0f, 0.0f};
    bool firstIteration = true;

    // Initial scan with larger steps
    for (int dc = MIN_CHARGE_DUTY_CYCLE; dc <= MAX_CHARGE_DUTY_CYCLE; dc += CHARGE_CURRENT_STEP * 2) {
        MHElectrodeData currentData = measureMHElectrodeVoltage(dc);

        // Add current data to the cache
        addToCache(currentData);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f && previousData.current >= 0.0f) { // Ensure previousData is valid for the first calculation
            float internalResistanceLU = (previousData.unloadedVoltage - currentData.loadedVoltage) / (currentData.current - previousData.current);
            if (!firstIteration && std::isfinite(internalResistanceLU) && internalResistanceLU >= 0) {
                storeOrAverageResistanceData(std::max(currentData.current, previousData.current), std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
            }
        }

        // --- Electrolysis Detection ---
        float chargingVoltage = getChargingVoltage(dc);
        float estimatedInternalResistance = 0.0f;

        // Use regressed intercept as an approximation if available, otherwise a simple calculation
        if (resistanceDataCount >= 2) {
            estimatedInternalResistance = regressedInternalResistanceIntercept;
        } else if (currentData.current > 0.01f && previousData.current >= 0.0f && !firstIteration) {
            estimatedInternalResistance = std::fabs((previousData.unloadedVoltage - currentData.loadedVoltage) / (currentData.current - previousData.current));
        } else if (currentData.current > 0.01f) {
            estimatedInternalResistance = std::fabs((currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current);
        }

        float voltageDrop = currentData.current * estimatedInternalResistance;
        float cellTerminalVoltage = chargingVoltage - voltageDrop;

        if (cellTerminalVoltage > ELECTROLYSIS_THRESHOLD_VOLTAGE) {
            Serial.printf("WARNING: Potential electrolysis detected at Duty Cycle: %d, Charging Voltage: %.3fV, Estimated Cell Voltage: %.3fV, Current: %.3fA\n",
                          dc, chargingVoltage, cellTerminalVoltage, currentData.current);
            // Consider taking action here: reduce duty cycle, stop charging, etc.
        }
        // --- End Electrolysis Detection ---

        float currentVoltageDifference = fabs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Initial Scan - Duty Cycle: %d, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      dc, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = dc;
        }

        previousData = currentData;
        firstIteration = false;
    }

    // Fine tune with small steps around the approximate value
    for (int dc = std::max((int)MIN_CHARGE_DUTY_CYCLE, optimalDutyCycle - CHARGE_CURRENT_STEP * 3);
         dc <= std::min((int)MAX_CHARGE_DUTY_CYCLE, optimalDutyCycle + CHARGE_CURRENT_STEP * 3);
         dc += CHARGE_CURRENT_STEP) {

        MHElectrodeData currentData = measureMHElectrodeVoltage(dc);

        // Add current data to the cache
        addToCache(currentData);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            if (std::isfinite(internalResistanceLU) && internalResistanceLU >= 0) {
                storeOrAverageResistanceData(currentData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
            }
        }

        // --- Check cache for more opportunities to calculate internal resistance (Pair) ---
        for (const auto& cachedData : dataCache) {
            if (std::abs(currentData.current - cachedData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                float internalResistancePair = (cachedData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - cachedData.current);
                if (std::isfinite(internalResistancePair) && internalResistancePair >= 0) {
                    float higherCurrent = std::max(currentData.current, cachedData.current);
                    storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
                    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs); // Sort after adding
                }
            }
        }

        // --- Electrolysis Detection ---
        float chargingVoltage = getChargingVoltage(dc);
        float estimatedInternalResistance = 0.0f;

        // Use regressed intercept as an approximation if available, otherwise a simple calculation
        if (resistanceDataCount >= 2) {
            estimatedInternalResistance = regressedInternalResistanceIntercept;
        } else if (currentData.current > 0.01f) {
            estimatedInternalResistance = std::fabs((currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current);
        }

        float voltageDrop = currentData.current * estimatedInternalResistance;
        float cellTerminalVoltage = chargingVoltage - voltageDrop;

        if (cellTerminalVoltage > ELECTROLYSIS_THRESHOLD_VOLTAGE) {
            Serial.printf("WARNING: Potential electrolysis detected during fine-tuning at Duty Cycle: %d, Charging Voltage: %.3fV, Estimated Cell Voltage: %.3fV, Current: %.3fA\n",
                          dc, chargingVoltage, cellTerminalVoltage, currentData.current);
            // Consider taking action here: reduce duty cycle, stop charging, etc.
        }
        // --- End Electrolysis Detection ---

        float currentVoltageDifference = fabs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Fine-tuning - Duty Cycle: %d, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      dc, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = dc;
        }
        previousData = currentData;
    }

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
