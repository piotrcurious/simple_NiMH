// Define additional constants for charging (adjust as needed)
#define STABILIZATION_DELAY_MS 100    // Delay after setting a duty cycle to allow voltage stabilization
#define CHARGING_UPDATE_INTERVAL_MS 2000  // Interval between re-evaluation during charging
#define PWM_MAX 255                   // Maximum PWM value (for analogWrite)
#define TEST_DUTY_CYCLE 128           // Initial test duty cycle value
#define VOLTAGE_TOLERANCE 0.05        // Voltage tolerance (in volts) for convergence

// Assume necessary includes and global variables are already declared
// e.g., pwmPin, dutyCycle, getThermistorReadings(), analogWrite(), millis(), etc.

// Existing functions: measureInternalResistance(), takeMeasurement(), stopLoad(), getUnloadedVoltageMeasurement(), etc.

// --- New Battery Charging Function ---
void chargeBattery() {
    Serial.println("Starting battery charging procedure...");

    // Step 1: Measure initial unloaded voltage.
    MeasurementData initialUnloaded = getUnloadedVoltageMeasurement();
    float V_unloaded = initialUnloaded.voltage;
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", V_unloaded);

    // Step 2: Apply a test current to obtain a loaded voltage measurement.
    MeasurementData testMeasurement = takeMeasurement(TEST_DUTY_CYCLE, STABILIZATION_DELAY_MS);
    float V_loaded_test = testMeasurement.voltage;
    Serial.printf("Test Loaded Voltage at duty %d: %.3f V\n", TEST_DUTY_CYCLE, V_loaded_test);

    // Calculate target voltage as the midpoint between unloaded and test loaded voltage.
    float V_target = V_unloaded - (V_unloaded - V_loaded_test) / 2.0;
    Serial.printf("Calculated Target Voltage: %.3f V\n", V_target);

    // Step 3: Use binary search to find the charging duty cycle that yields the target voltage.
    int lowerDuty = 0;
    int upperDuty = PWM_MAX;
    int chargingDuty = TEST_DUTY_CYCLE;
    bool converged = false;
    for (int i = 0; i < 10; i++) {  // maximum of 10 iterations
        chargingDuty = (lowerDuty + upperDuty) / 2;
        MeasurementData m = takeMeasurement(chargingDuty, STABILIZATION_DELAY_MS);
        float measuredVoltage = m.voltage;
        Serial.printf("Iteration %d: Duty %d, Measured Voltage: %.3f V\n", i, chargingDuty, measuredVoltage);
        
        if (fabs(measuredVoltage - V_target) < VOLTAGE_TOLERANCE) {
            converged = true;
            break;
        }
        if (measuredVoltage > V_target) {
            // Measured voltage too high means charging current is too low, so increase current (increase duty)
            lowerDuty = chargingDuty + 1;
        } else {
            // Measured voltage too low means charging current is too high, so decrease current (decrease duty)
            upperDuty = chargingDuty - 1;
        }
        delay(100); // Small delay for stabilization
    }
    if (!converged) {
        Serial.println("Binary search did not converge exactly; using last computed duty cycle.");
    }
    Serial.printf("Charging Duty Cycle determined: %d\n", chargingDuty);

    // Step 4: Begin charging loop. The charging current is applied while periodically checking the MH electrode voltage.
    while (true) {
        // Apply the determined charging current.
        analogWrite(pwmPin, chargingDuty);
        delay(CHARGING_UPDATE_INTERVAL_MS);

        // Re-evaluate the loaded voltage at the charging duty.
        MeasurementData currentMeasurement = takeMeasurement(chargingDuty, STABILIZATION_DELAY_MS);
        float currentVoltage = currentMeasurement.voltage;
        Serial.printf("Charging... Duty %d, Loaded Voltage: %.3f V, TempDiff: %.2f C\n", chargingDuty, currentVoltage, currentMeasurement.tempDiff);

        // Periodically re-calculate the target voltage:
        MeasurementData newUnloaded = getUnloadedVoltageMeasurement();
        float newV_unloaded = newUnloaded.voltage;
        MeasurementData newTest = takeMeasurement(TEST_DUTY_CYCLE, STABILIZATION_DELAY_MS);
        float newV_loaded_test = newTest.voltage;
        float newV_target = newV_unloaded - (newV_unloaded - newV_loaded_test) / 2.0;
        Serial.printf("Recalculated Target Voltage: %.3f V (Unloaded: %.3f V, Test Loaded: %.3f V)\n", newV_target, newV_unloaded, newV_loaded_test);

        // Adjust the charging duty slightly (a single-step adjustment) if the current loaded voltage deviates from the target.
        if (fabs(currentVoltage - newV_target) > VOLTAGE_TOLERANCE) {
            if (currentVoltage > newV_target && chargingDuty < PWM_MAX) {
                // Voltage too high: Increase charging current (i.e., increase duty cycle)
                chargingDuty++;
            } else if (currentVoltage < newV_target && chargingDuty > 0) {
                // Voltage too low: Decrease charging current (i.e., decrease duty cycle)
                chargingDuty--;
            }
            Serial.printf("Adjusting Charging Duty Cycle to: %d\n", chargingDuty);
        }

        // Check if temperature difference exceeds the safety threshold (0.8°C).
        if (currentMeasurement.tempDiff > 0.8) {
            Serial.printf("Temperature difference %.2f C exceeded threshold. Stopping charge.\n", currentMeasurement.tempDiff);
            break;
        }
    }
    // Stop charging by setting the PWM duty cycle to 0.
    analogWrite(pwmPin, 0);
    Serial.println("Battery charging completed.");
}

// --- Existing functions below remain unchanged ---
//
// void measureInternalResistance() { ... }
// ... and the helper functions in the anonymous namespace.
