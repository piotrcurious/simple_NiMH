// Define a threshold for electrolysis (adjust this value as needed)
#define ELECTROLYSIS_THRESHOLD 1.23f  // Example: 1.23V for water splitting
#define DEFAULT_INTERNAL_RESISTANCE 0.05f  // Default internal resistance if regression data is insufficient

// ... (other code)

previousData = currentData;
firstIteration = false;

// ... (data collection and error distribution code)

distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);  // Example parameters
distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);  // Example parameters

// Final measurement at the optimal duty cycle
MHElectrodeData finalData = measureMHElectrodeVoltage(optimalDutyCycle);
targetVoltage = initialUnloadedVoltage + (finalData.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

Serial.printf("Optimal charging duty cycle found: %d (loaded: %.3fV, target: %.3fV, diff: %.3fV)\n",
              optimalDutyCycle, finalData.loadedVoltage, targetVoltage, fabs(finalData.loadedVoltage - targetVoltage));

// --- Electrolysis Detection ---
// Assume that currentData.current holds the current (in amperes) during charging.
// Also, we use the regressed slope from the loaded/unloaded regression as an estimate of internal resistance.
// If there are not enough data points for regression, fall back to a default internal resistance.
float chargingCurrent = currentData.current; // Assumes currentData has a 'current' field
float estimatedInternalResistance = (resistanceDataCount >= 2) ? regressedInternalResistanceSlope : DEFAULT_INTERNAL_RESISTANCE;
float voltageDrop = chargingCurrent * estimatedInternalResistance;
float effectiveCellVoltage = finalData.loadedVoltage - voltageDrop;

if (effectiveCellVoltage > ELECTROLYSIS_THRESHOLD) {
    Serial.printf("Warning: Electrolysis detected! Effective cell voltage (%.3fV) exceeds threshold (%.3fV).\n",
                  effectiveCellVoltage, ELECTROLYSIS_THRESHOLD);
    // Optionally, take further action here (e.g., adjust duty cycle, shut off charging, etc.)
}

// --- Linear Regression for Loaded/Unloaded Resistance ---
if (resistanceDataCount >= 2) {
    if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
        Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                      regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
    }
} else {
    Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
}

// --- Linear Regression for Paired Resistance Data ---
if (resistanceDataCountPairs >= 2) {
    if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
        Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                      regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
    }
} else {
    Serial.println("Not enough data points for linear regression of paired resistance.");
}

return optimalDutyCycle;
