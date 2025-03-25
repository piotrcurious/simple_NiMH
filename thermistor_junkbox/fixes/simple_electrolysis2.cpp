// Assuming ELECTROLYSIS_THRESHOLD is defined elsewhere based on the battery chemistry
// Example: #define ELECTROLYSIS_THRESHOLD 2.5f // Volts

// Assuming MHElectrodeData struct is defined as:
/*
struct MHElectrodeData {
    float loadedVoltage;
    float unloadedVoltage;
    float current; // Add this to your struct if it's not there
};
*/

// Assuming measureMHElectrodeVoltage function is modified to return the current as well.
// For example, it could update the MHElectrodeData struct with the current value.

int optimizeChargingDutyCycle() {
    int optimalDutyCycle = -1;
    float minVoltageDifference = FLT_MAX;
    int minDutyCycle = 0;   // Example
    int maxDutyCycle = 100; // Example
    int dutyCycleStep = 5;  // Example
    float MH_ELECTRODE_RATIO = 0.5f; // Example
    float initialUnloadedVoltage = 3.0f; // Example
    float targetVoltage = 0.0f;

    // Data collection for internal resistance calculation
    const int MAX_RESISTANCE_DATA = 50; // Example
    float internalResistanceData[MAX_RESISTANCE_DATA];
    int resistanceDataCount = 0;
    float internalResistanceDataPairs[MAX_RESISTANCE_DATA];
    int resistanceDataCountPairs = 0;

    float regressedInternalResistanceSlope = 0.0f;
    float regressedInternalResistanceIntercept = 0.0f;
    float regressedInternalResistancePairsSlope = 0.0f;
    float regressedInternalResistancePairsIntercept = 0.0f;

    MHElectrodeData currentData;
    MHElectrodeData previousData;
    bool firstIteration = true;

    for (int dutyCycle = minDutyCycle; dutyCycle <= maxDutyCycle; dutyCycle += dutyCycleStep) {
        currentData = measureMHElectrodeVoltage(dutyCycle);

        // Electrolysis Detection Logic
        // 1. Get the charging voltage (this might be the unloaded voltage just before applying the load)
        float chargingVoltage = currentData.unloadedVoltage; // Assuming unloaded voltage is a good proxy for charging voltage

        // 2. Get the measured current
        float current = currentData.current;

        // 3. Get the estimated internal resistance (using the slope from previous measurements)
        float internalResistance = regressedInternalResistanceSlope; // Using the slope as an estimate

        // 4. Calculate the voltage drop across the internal resistance
        float voltageDrop = fabs(current) * internalResistance; // Use absolute value of current

        // 5. Calculate the actual voltage at the cell terminals
        float cellVoltage = chargingVoltage - voltageDrop;

        // 6. Check if the cell voltage exceeds the electrolysis threshold
        if (cellVoltage > ELECTROLYSIS_THRESHOLD) {
            Serial.printf("WARNING: Potential electrolysis detected at duty cycle %d!\n", dutyCycle);
            Serial.printf("  Charging Voltage: %.3fV, Current: %.3fA, Estimated Internal Resistance: %.4f Ohm\n",
                          chargingVoltage, current, internalResistance);
            Serial.printf("  Voltage Drop: %.3fV, Actual Cell Voltage: %.3fV, Electrolysis Threshold: %.3fV\n",
                          voltageDrop, cellVoltage, ELECTROLYSIS_THRESHOLD);
            // You might want to take action here, like reducing the duty cycle or stopping charging
        }

        // --- Existing code for duty cycle optimization and resistance measurement ---
        if (!firstIteration) {
            float voltageDifference = fabs(targetVoltage - currentData.loadedVoltage);
            if (voltageDifference < minVoltageDifference) {
                minVoltageDifference = voltageDifference;
                optimalDutyCycle = dutyCycle;
            }

            // Collect data for internal resistance calculation (Loaded/Unloaded)
            if (resistanceDataCount < MAX_RESISTANCE_DATA && currentData.unloadedVoltage > 0 && currentData.loadedVoltage > 0) {
                internalResistanceData[resistanceDataCount++] = (currentData.unloadedVoltage - currentData.loadedVoltage) / fabs(currentData.current);
            }

            // Collect paired resistance data (if you have a way to measure resistance directly or infer it from paired measurements)
            // This part depends on how 'internalResistanceDataPairs' is populated in your original context.
            // Assuming 'measureInternalResistancePair' is a function that provides such data.
            // float resistancePair = measureInternalResistancePair(dutyCycle);
            // if (resistanceDataCountPairs < MAX_RESISTANCE_DATA && resistancePair > 0) {
            //     internalResistanceDataPairs[resistanceDataCountPairs++] = resistancePair;
            // }
        }

        previousData = currentData;
        firstIteration = false;
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

    // Electrolysis Detection for the final measurement
    float finalChargingVoltage = finalData.unloadedVoltage; // Assuming unloaded voltage is a good proxy
    float finalCurrent = finalData.current;
    float finalInternalResistance = regressedInternalResistanceSlope;
    float finalVoltageDrop = fabs(finalCurrent) * finalInternalResistance;
    float finalCellVoltage = finalChargingVoltage - finalVoltageDrop;

    if (finalCellVoltage > ELECTROLYSIS_THRESHOLD) {
        Serial.printf("WARNING: Potential electrolysis detected at optimal duty cycle!\n");
        Serial.printf("  Charging Voltage: %.3fV, Current: %.3fA, Estimated Internal Resistance: %.4f Ohm\n",
                      finalChargingVoltage, finalCurrent, finalInternalResistance);
        Serial.printf("  Voltage Drop: %.3fV, Actual Cell Voltage: %.3fV, Electrolysis Threshold: %.3fV\n",
                      finalVoltageDrop, finalCellVoltage, ELECTROLYSIS_THRESHOLD);
        // Consider taking action here
    }

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
