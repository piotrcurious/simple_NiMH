#include <Arduino.h>
#include <limits> // Required for numeric_limits
#include <cmath>  // Required for fabs, exp, pow

// ... (Keep all previous includes, defines, constants like C_DL_SPECIFIC, etc.) ...
// ... (Keep global variables, enums, structs as before) ...
// ... (Keep helper functions: checkTimer, safeDivide) ...
// ... (Keep Butler-Volmer helper: calculate_current_density) ...
// ... (Keep computeMaxElectrolysisCurrent, measureDynamicResponse_Tick etc. as before) ...


//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V6 (Individual i0 Adjust)
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float internalResistance, float idleVoltage, float batteryTemperatureC) {

    const unsigned long STEADY_STATE_PULSE_MS = 2000;
    const unsigned long STEADY_STATE_SETTLE_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50;
    const int BISECTION_ITERATIONS = 15;
    const float BISECTION_TOLERANCE = 1e-4;

    static unsigned long timerTimestamp;
    static float V_cal, I_cal;
    static DynamicResponse dynamicResponseResult;

    // --- State Machine Logic (States IDLE to DYNAMIC_MEASURE remain the same) ---
    switch (currentCalibrationState) {
        // ... (Cases IDLE to DYNAMIC_MEASURE are identical to V5) ...
        case CalibrationState::IDLE: break;
        case CalibrationState::START_CALIBRATION:
             Serial.println("Calibration: Starting (V6 - Individual i0 Adjust)...");
             if (internalResistance <= 0) { /* ... error handling ... */ currentCalibrationState = CalibrationState::ERROR; break; }
             // ... start pulse ...
              pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, HIGH); timerTimestamp = millis();
             currentCalibrationState = CalibrationState::STEADY_STATE_PULSE;
             Serial.print("Calibration: Steady-state pulse ("); Serial.print(STEADY_STATE_PULSE_MS); Serial.println(" ms)");
             break;
        case CalibrationState::STEADY_STATE_PULSE:
             if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                 // ... read V_cal, I_cal ...
                 noInterrupts(); V_cal = batteryVoltage; I_cal = batteryCurrent; interrupts();
                 digitalWrite(PWM_PIN, LOW); timerTimestamp = millis();
                 currentCalibrationState = CalibrationState::STEADY_STATE_SETTLE;
                 Serial.print(" -> V_cal = "); Serial.print(V_cal, 4); Serial.print(" V, I_cal = "); Serial.println(I_cal, 4);
                 if (fabs(I_cal) < 1e-6) { Serial.println("Calibration Warning: Measured calibration current is near zero."); }
             }
             break;
        case CalibrationState::STEADY_STATE_SETTLE:
             if (checkTimer(timerTimestamp, STEADY_STATE_SETTLE_MS)) { currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS; }
             break;
        case CalibrationState::START_DYNAMIC_MEAS:
             Serial.println("Calibration: Starting dynamic measurement phase...");
             startDynamicMeasurement();
             currentCalibrationState = CalibrationState::DYNAMIC_MEASURE;
             break;
        case CalibrationState::DYNAMIC_MEASURE:
             measureDynamicResponse_Tick(dynamicResponseResult);
             if (getDynamicResponseResult(dynamicResponseResult)) {
                 if(dynamicResponseResult.valid) { currentCalibrationState = CalibrationState::CALCULATING; }
                 else { /* ... error handling ... */ }
             } else if (currentDynamicState == DynamicMeasureState::ERROR) { /* ... error handling ... */ }
             break;


        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments (V6)...");
             float T_kelvin = batteryTemperatureC + 273.15;

             // Store initial parameters for logging
             float A_CATHODE_start = A_CATHODE; float A_NODE_start = A_NODE;
             float i0_H2_start = i0_H2; float i0_O2_start = i0_O2;

             // === Part 0: Adjust i0 Individually based on Dynamic R_ct ===
             Serial.println("--- Step 0: Adjusting i0 Individually (Dynamic Response) ---");
             float correctionFactor_i0_H2 = 1.0; // Default to no correction
             float correctionFactor_i0_O2 = 1.0; // Default to no correction

             if (dynamicResponseResult.timeConstantMs <= 0) {
                 Serial.println("  Warning: Invalid time constant (<= 0 ms). Skipping i0 adjustment.");
             } else if (A_CATHODE <= 0 || A_NODE <= 0 || i0_H2 <= 0 || i0_O2 <= 0) {
                  Serial.println("  Warning: Invalid Area/i0 parameter (<= 0). Skipping i0 adjustment.");
             } else {
                 float C_dl_total_estimated = C_DL_SPECIFIC * (A_NODE + A_CATHODE); // Use current A
                 if (C_dl_total_estimated <= 0) {
                      Serial.println("  Warning: Estimated C_dl is zero or negative. Skipping i0 adjustment.");
                 } else {
                      float R_ct_estimated_sum = (dynamicResponseResult.timeConstantMs / 1000.0) / C_dl_total_estimated;

                      // Calculate individual theoretical R_ct (equilibrium approx.) using *current* parameters
                      float R_ct_H2_theory = safeDivide((R * T_kelvin), (n_H2 * F * i0_H2 * A_CATHODE), std::numeric_limits<float>::max());
                      float R_ct_O2_theory = safeDivide((R * T_kelvin), (n_O2 * F * i0_O2 * A_NODE), std::numeric_limits<float>::max());
                      float R_ct_theory_total = R_ct_H2_theory + R_ct_O2_theory;

                      if (fabs(R_ct_estimated_sum) < std::numeric_limits<float>::epsilon()) {
                          Serial.println("  Warning: Estimated R_ct sum is near zero. Skipping i0 adjustment.");
                      } else if (fabs(R_ct_theory_total) < std::numeric_limits<float>::epsilon()) {
                           Serial.println("  Warning: Theoretical R_ct sum is near zero. Skipping i0 adjustment.");
                      }
                      else {
                            float delta_R = R_ct_estimated_sum - R_ct_theory_total; // Difference to correct
                            float R_ct_H2_sq = pow(R_ct_H2_theory, 2);
                            float R_ct_O2_sq = pow(R_ct_O2_theory, 2);
                            float denom_R_ct_sq_sum = R_ct_H2_sq + R_ct_O2_sq;

                            if (fabs(denom_R_ct_sq_sum) < std::numeric_limits<float>::epsilon()) {
                                Serial.println("  Warning: Sum of squared theoretical R_ct is near zero. Skipping i0 adjustment.");
                            } else {
                                // Calculate individual correction factors based on min relative change
                                correctionFactor_i0_H2 = 1.0 - delta_R * R_ct_H2_theory / denom_R_ct_sq_sum;
                                correctionFactor_i0_O2 = 1.0 - delta_R * R_ct_O2_theory / denom_R_ct_sq_sum;

                                // Apply damping/limits individually
                                const float MAX_CORRECTION_I0 = 3.0; const float MIN_CORRECTION_I0 = 0.33;
                                if (correctionFactor_i0_H2 > MAX_CORRECTION_I0) correctionFactor_i0_H2 = MAX_CORRECTION_I0;
                                if (correctionFactor_i0_H2 < MIN_CORRECTION_I0) correctionFactor_i0_H2 = MIN_CORRECTION_I0;
                                if (correctionFactor_i0_O2 > MAX_CORRECTION_I0) correctionFactor_i0_O2 = MAX_CORRECTION_I0;
                                if (correctionFactor_i0_O2 < MIN_CORRECTION_I0) correctionFactor_i0_O2 = MIN_CORRECTION_I0;

                                // Logging
                                Serial.print("  Measured Time Constant: "); Serial.print(dynamicResponseResult.timeConstantMs, 2); Serial.println(" ms");
                                Serial.print("  Estimated C_dl_total: "); Serial.println(C_dl_total_estimated, 6);
                                Serial.print("  Estimated R_ct Sum (meas): "); Serial.println(R_ct_estimated_sum, 6);
                                Serial.print("  Theoretical R_ct H2: "); Serial.print(R_ct_H2_theory, 6); Serial.print(", O2: "); Serial.print(R_ct_O2_theory, 6); Serial.print(", Total: "); Serial.println(R_ct_theory_total, 6);
                                Serial.print("  Delta R_ct needed: "); Serial.println(delta_R, 6);
                                Serial.print("  Correction Factor i0 H2: "); Serial.print(correctionFactor_i0_H2, 4); Serial.print(", O2: "); Serial.println(correctionFactor_i0_O2, 4);
                            }
                      }
                 }
             }
             // Apply individual i0 correction factors
             i0_H2 *= correctionFactor_i0_H2;
             i0_O2 *= correctionFactor_i0_O2;
             // Apply minimum limits
             const float MIN_I0_VALUE = 1e-9;
             if (i0_H2 < MIN_I0_VALUE) i0_H2 = MIN_I0_VALUE;
             if (i0_O2 < MIN_I0_VALUE) i0_O2 = MIN_I0_VALUE;
             Serial.print("  i0_H2: "); Serial.print(i0_H2_start, 6); Serial.print(" -> "); Serial.println(i0_H2, 6);
             Serial.print("  i0_O2: "); Serial.print(i0_O2_start, 6); Serial.print(" -> "); Serial.println(i0_O2, 6);


             // === Part 1: Find Consistent Overpotential Split (using *updated* i0) ===
             Serial.println("--- Step 1: Finding Overpotential Split (using updated i0) ---");
             // ... (This section is identical to V5 - calculates eta_total, runs bisection) ...
             float V_H2_eq = V_H2_EVOL_STD; float V_O2_eq = V_O2_EVOL_STD; float V_cell_eq = V_O2_eq - V_H2_eq;
             float V_electrodes = V_cal - I_cal * internalResistance; float eta_total = V_electrodes - V_cell_eq;
             float eta_O2_cal = 0.0; float eta_H2_cal = 0.0; bool split_found = false;
             Serial.print("  eta_total = "); Serial.println(eta_total, 4);
             if (eta_total <= 1e-6) { /* ... */ split_found = true; }
             else if (A_NODE <= 0 || A_CATHODE <= 0 || i0_O2 <= 0 || i0_H2 <= 0) { /* ... */ }
             else {
                 float eta_O2_low = 0.0; float eta_O2_high = eta_total; float current_diff = 0.0;
                 for (int i = 0; i < BISECTION_ITERATIONS; ++i) {
                     eta_O2_cal = (eta_O2_low + eta_O2_high) / 2.0; eta_H2_cal = eta_O2_cal - eta_total;
                     // Use UPDATED i0, current A
                     float i_O2_pred = calculate_current_density(eta_O2_cal, i0_O2, n_O2, alpha, T_kelvin);
                     float i_H2_pred = calculate_current_density(eta_H2_cal, i0_H2, n_H2, alpha, T_kelvin);
                     float I_anode_pred = i_O2_pred * A_NODE; float I_cathode_pred = fabs(i_H2_pred * A_CATHODE);
                     current_diff = I_anode_pred - I_cathode_pred;
                     if (fabs(current_diff) < BISECTION_TOLERANCE) { split_found = true; break; }
                     if (current_diff > 0) { eta_O2_high = eta_O2_cal; } else { eta_O2_low = eta_O2_cal; }
                 }
                 if (split_found) { Serial.print("  Bisection converged: eta_O2 = "); /* ... */ Serial.println(eta_H2_cal, 5); }
                 else { Serial.println("  Warning: Bisection for overpotential split did not converge."); }
             }

             // === Part 2: Adjust Area (using split found with updated i0) ===
             Serial.println("--- Step 2: Adjusting Area (Steady State) ---");
             // ... (This section is identical to V5 - calculates I_model_consistent, correctionFactorArea, applies it) ...
             float correctionFactorArea = 1.0; float I_model_consistent = 0.0;
             if (!split_found) { /* ... skip ... */ }
             else if (eta_total <= 1e-6) { /* ... skip ... */}
             else if (A_NODE <= 0 || A_CATHODE <= 0 || i0_O2 <= 0 || i0_H2 <= 0) { /* ... skip ... */}
             else {
                 // Use updated i0, current A
                 float i_O2_final = calculate_current_density(eta_O2_cal, i0_O2, n_O2, alpha, T_kelvin);
                 I_model_consistent = i_O2_final * A_NODE;
                 Serial.print("  Consistent Model Current (I_model_consistent): "); Serial.println(I_model_consistent, 6);
                 if (fabs(I_model_consistent) < 1e-9) { /* ... skip ... */ }
                 else if (fabs(I_cal) < 1e-9) { /* ... skip ... */ }
                 else {
                     correctionFactorArea = safeDivide(I_cal, I_model_consistent, 1.0f);
                     const float MAX_CORRECTION_AREA = 2.0; const float MIN_CORRECTION_AREA = 0.5;
                     /* ... apply limits ... */
                      if (correctionFactorArea > MAX_CORRECTION_AREA) correctionFactorArea = MAX_CORRECTION_AREA;
                      if (correctionFactorArea < MIN_CORRECTION_AREA) correctionFactorArea = MIN_CORRECTION_AREA;
                     Serial.print("  Area Correction Factor: "); Serial.println(correctionFactorArea, 4);
                 }
             }
             A_CATHODE *= correctionFactorArea; // Apply correction
             A_NODE *= correctionFactorArea;
             Serial.print("  A_CATHODE: "); Serial.print(A_CATHODE_start, 6); Serial.print(" -> "); Serial.println(A_CATHODE, 6);
             Serial.print("  A_NODE:    "); Serial.print(A_NODE_start, 6); Serial.print(" -> "); Serial.println(A_NODE, 6);


             // --- Final Output ---
             Serial.println("--- Calibration V6 Complete ---");
             // ... (Print final parameters A_CATHODE, A_NODE, i0_H2, i0_O2) ...
             Serial.println("Final Parameters:");
             Serial.print(" A_CATHODE: "); Serial.println(A_CATHODE, 6);
             Serial.print(" A_NODE:    "); Serial.println(A_NODE, 6);
             Serial.print(" i0_H2:     "); Serial.println(i0_H2, 6);
             Serial.print(" i0_O2:     "); Serial.println(i0_O2, 6);
             Serial.println("-----------------------------");

             currentCalibrationState = CalibrationState::COMPLETE;
             break; // End of CALCULATING case


        case CalibrationState::COMPLETE: break;
        case CalibrationState::ERROR: Serial.println("Calibration: Error State."); break;
    } // End switch

    return (currentCalibrationState != CalibrationState::IDLE &&
            currentCalibrationState != CalibrationState::COMPLETE &&
            currentCalibrationState != CalibrationState::ERROR);
}

// --- Rest of the code (setup, loop, mockExternalUpdate, helpers etc.) remains the same as V5 ---
// Make sure to include full implementations for all helper functions and state machines.

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("ESP32 Electrochemical Calibration V6 Started");
    Serial.print("Assumed C_DL_SPECIFIC: "); Serial.print(C_DL_SPECIFIC); Serial.println(" F/m^2");
    // ... (rest of setup) ...
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);
    // lastExternalUpdate = millis(); // Assuming mockExternalUpdate is used
}

// ... (loop, mock update, other helpers from V5) ...

// --- Mock external update variables --- (If using)
volatile float batteryVoltage = 3.7;
volatile float batteryCurrent = 0.0;
unsigned long lastExternalUpdate = 0;
void mockExternalUpdate() { /* ... (same as V2/V3/V4/V5) ... */ }

void loop() {
    // mockExternalUpdate(); // Call if using mock update

    noInterrupts();
    float currentVoltage = batteryVoltage; // Read volatile vars
    float currentCurrent = batteryCurrent;
    interrupts();

    static float internalResistance = 0.05;
    static float batteryTempC = 25.0; // Use a sensor! Remember Spain's current conditions if relevant.
    static bool calibrationRequested = false;

    bool calibrating = calibrateBatteryChemistryModel_Tick(internalResistance, currentVoltage, batteryTempC); // Pass current idle V

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if ((cmd == 'c' || cmd == 'C') && !calibrating) {
            calibrationRequested = true;
        }
    }

    if (calibrationRequested) {
        startCalibration();
        calibrationRequested = false;
    }

    static unsigned long lastPredictionTime = 0;
    unsigned long predictionInterval = 5000; // 5 seconds

    if (!calibrating && checkTimer(lastPredictionTime, predictionInterval)) {
         lastPredictionTime = millis();
         float idleVoltageForPrediction = currentVoltage; // Simple approximation
         float I_max_pred = computeMaxElectrolysisCurrent(idleVoltageForPrediction, batteryTempC, internalResistance);

         Serial.println("---------------------------");
         Serial.print(millis());
         Serial.print("ms - V="); Serial.print(currentVoltage, 4);
         Serial.print("V, I="); Serial.print(currentCurrent, 4);
         Serial.print("A || Params: A_cat="); Serial.print(A_CATHODE,4);
         Serial.print(", A_an="); Serial.print(A_NODE,4);
         Serial.print(", i0_H2="); Serial.print(i0_H2,4);
         Serial.print(", i0_O2="); Serial.println(i0_O2,4);
         Serial.print("Predicted max electrolysis onset current: ");
         Serial.println(I_max_pred, 6);
         Serial.println("---------------------------");
    }

    delay(10); // Prevent tight loop
}

// --- Include required helper functions ---
// bool checkTimer(...) { ... }
// float safeDivide(...) { ... }
// float calculate_current_density(...) { ... }
// void startCalibration() { ... }
// bool isCalibrationComplete() { ... }
// void startDynamicMeasurement() { ... }
// bool getDynamicResponseResult(...) { ... }
// bool measureDynamicResponse_Tick(...) { ... }
// float computeMaxElectrolysisCurrent(...) { ... }
