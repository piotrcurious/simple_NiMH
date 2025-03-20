void chargeBattery() {
    Serial.println("Starting battery charging process...");

    // Measure initial unloaded voltage
    MeasurementData initialUnloaded = getUnloadedVoltageMeasurement();
    float initialVoltage = initialUnloaded.voltage;
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialVoltage);

    // Find optimal charging current
    float optimalCurrent = findOptimalChargingCurrent(initialVoltage);
    Serial.printf("Optimal Charging Current: %.3f mA\n", optimalCurrent);

    if (optimalCurrent <= 0) {
        Serial.println("Unable to determine optimal charging current. Aborting.");
        return;
    }

    // Start charging
    Serial.println("Charging started...");
    while (true) {
        // Apply optimal charging current
        applyChargingCurrent(optimalCurrent);

        // Re-evaluate MH electrode voltage
        MeasurementData currentMeasurement = getUnloadedVoltageMeasurement();
        float currentVoltage = currentMeasurement.voltage;

        // Recalculate optimal current every cycle
        float newOptimalCurrent = findOptimalChargingCurrent(currentVoltage);
        if (fabs(newOptimalCurrent - optimalCurrent) > 10) { // If deviation is significant, update current
            optimalCurrent = newOptimalCurrent;
            Serial.printf("Updated Charging Current: %.3f mA\n", optimalCurrent);
        }

        // Check temperature threshold
        if (checkTemperatureThreshold()) {
            Serial.println("Temperature threshold exceeded. Stopping charge.");
            break;
        }

        vTaskDelay(5000); // Wait 5 seconds before next evaluation
    }

    stopCharging();
    Serial.println("Charging process complete.");
}

float findOptimalChargingCurrent(float unloadedVoltage) {
    Serial.println("Finding optimal charging current...");
    float targetVoltage = unloadedVoltage / 2; // MH electrode voltage target

    float testCurrent = 10.0; // Start with a small test current (mA)
    float stepCurrent = 5.0;  // Step increase (mA)
    float maxCurrent = 500.0; // Max allowable charge current (mA)

    while (testCurrent <= maxCurrent) {
        applyChargingCurrent(testCurrent);
        vTaskDelay(200); // Allow stabilization

        MeasurementData loadedMeasurement = getUnloadedVoltageMeasurement();
        float loadedVoltage = loadedMeasurement.voltage;

        if (fabs(loadedVoltage - targetVoltage) < 0.05) { // Small tolerance
            Serial.printf("Found optimal current: %.3f mA\n", testCurrent);
            return testCurrent;
        }

        testCurrent += stepCurrent;
    }

    return -1; // Failed to find optimal current
}

void applyChargingCurrent(float current) {
    int dutyCycle = map(current, 0, 500, 0, 255); // Scale current to duty cycle
    analogWrite(pwmPin, dutyCycle);
}

bool checkTemperatureThreshold() {
    float temp1, temp2, tempDiff;
    getThermistorReadings(temp1, temp2, tempDiff, nullptr, nullptr, nullptr);
    Serial.printf("Temperature difference: %.3f°C\n", tempDiff);

    return tempDiff > 0.8;
}

void stopCharging() {
    analogWrite(pwmPin, 0);
    Serial.println("Charging stopped.");
}
