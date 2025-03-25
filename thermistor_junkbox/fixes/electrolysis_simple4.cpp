// Compute estimated internal resistance using regression
float estimatedInternalResistance = regressedInternalResistanceIntercept; // Default to intercept
if (resistanceDataCount >= 2) {
    float optimalCurrent = (initialUnloadedVoltage - finalData.loadedVoltage) / regressedInternalResistanceSlope;
    estimatedInternalResistance += regressedInternalResistanceSlope * optimalCurrent;
}

// Compute charging current
float estimatedCurrent = (initialUnloadedVoltage - finalData.loadedVoltage) / estimatedInternalResistance;

// Compute estimated cell voltage
float estimatedCellVoltage = finalData.loadedVoltage - (estimatedCurrent * estimatedInternalResistance);

// Check for electrolysis condition
const float ELECTROLYSIS_THRESHOLD = 1.23f; // Example threshold, adjust for your application
bool electrolysisDetected = (estimatedCellVoltage > ELECTROLYSIS_THRESHOLD);

Serial.printf("Estimated Cell Voltage: %.3fV (Electrolysis %s)\n", 
              estimatedCellVoltage, electrolysisDetected ? "DETECTED" : "NOT DETECTED");

if (electrolysisDetected) {
    Serial.println("Warning: Electrolysis detected! Adjust charging parameters.");
}

return optimalDutyCycle;
