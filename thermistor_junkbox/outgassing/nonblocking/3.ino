//--------------------------------------------------------------
// calculate_current_density()
//
// Calculates current density using the full Butler-Volmer equation.
// eta: Overpotential (V)
// i0: Exchange current density (A/m^2)
// n: Number of electrons transferred
// alpha: Charge transfer coefficient
// T_kelvin: Temperature (K)
// Returns: Current density (A/m^2) (positive for anodic, negative for cathodic)
//--------------------------------------------------------------
float calculate_current_density(float eta, float i0, float n, float alpha, float T_kelvin) {
    const float RT = R * T_kelvin;
    if (RT <= 0 || n <= 0 || i0 < 0) { // Basic safety checks
        return 0.0;
    }
    // Prevent exp() overflow with large eta - exp(>~88) is problematic for float
    float exp_term1_arg = (alpha * n * F * eta) / RT;
    float exp_term2_arg = -((1.0 - alpha) * n * F * eta) / RT;

    // Clamp arguments to prevent overflow/underflow issues with exp()
    const float EXP_ARG_LIMIT = 80.0; // Adjust if needed, corresponds to e^80
    if (exp_term1_arg > EXP_ARG_LIMIT) exp_term1_arg = EXP_ARG_LIMIT;
    if (exp_term1_arg < -EXP_ARG_LIMIT) exp_term1_arg = -EXP_ARG_LIMIT;
    if (exp_term2_arg > EXP_ARG_LIMIT) exp_term2_arg = EXP_ARG_LIMIT;
    if (exp_term2_arg < -EXP_ARG_LIMIT) exp_term2_arg = -EXP_ARG_LIMIT;

    float term1 = exp(exp_term1_arg);
    float term2 = exp(exp_term2_arg);

    return i0 * (term1 - term2);
}

#include <Arduino.h>
#include <limits> // Required for numeric_limits
#include <cmath>  // Required for fabs, exp

// ... (Keep all previous includes, defines, constants like C_DL_SPECIFIC, etc.) ...
// ... (Keep global variables, enums, structs as before) ...
// ... (Keep helper functions: checkTimer, safeDivide) ...

// *** ADD Butler-Volmer Helper Function Here ***
float calculate_current_density(float eta, float i0, float n, float alpha, float T_kelvin) {
    // ... (Implementation as above) ...
     const float RT = R * T_kelvin;
    if (RT <= 0 || n <= 0 || i0 < 0) return 0.0;
    float exp_term1_arg = (alpha * n * F * eta) / RT;
    float exp_term2_arg = -((1.0 - alpha) * n * F * eta) / RT;
    const float EXP_ARG_LIMIT = 80.0;
    if (exp_term1_arg > EXP_ARG_LIMIT) exp_term1_arg = EXP_ARG_LIMIT;
    if (exp_term1_arg < -EXP_ARG_LIMIT) exp_term1_arg = -EXP_ARG_LIMIT;
    if (exp_term2_arg > EXP_ARG_LIMIT) exp_term2_arg = EXP_ARG_LIMIT;
    if (exp_term2_arg < -EXP_ARG_LIMIT) exp_term2_arg = -EXP_ARG_LIMIT;
    float term1 = exp(exp_term1_arg);
    float term2 = exp(exp_term2_arg);
    return i0 * (term1 - term2);
}


// ... (Keep computeMaxElectrolysisCurrent, measureDynamicResponse_Tick etc. as before) ...


//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V4 (Numerical Overpotential Split)
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float internalResistance, float idleVoltage, float batteryTemperatureC) {

    const unsigned long STEADY_STATE_PULSE_MS = 2000;
    const unsigned long STEADY_STATE_SETTLE_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50;
    // For Bisection Search
    const int BISECTION_ITERATIONS = 15; // ~15 iterations give good precision
    const float BISECTION_TOLERANCE = 1e-4; // Tolerance for current difference

    static unsigned long timerTimestamp;
    static float V_cal, I_cal;
    static DynamicResponse dynamicResponseResult;

    // --- State Machine Logic (States IDLE to DYNAMIC_MEASURE remain the same) ---
    switch (currentCalibrationState) {
        // ... (Cases IDLE to DYNAMIC_MEASURE are identical to V3) ...
        case CalibrationState::IDLE: break; // Add breaks for clarity
        case CalibrationState::START_CALIBRATION:
            Serial.println("Calibration: Starting (V4 - Numerical Split)...");
             if (internalResistance <= 0) { /* ... error handling ... */ currentCalibrationState = CalibrationState::ERROR; break;}
             pinMode(PWM_PIN, OUTPUT);
             digitalWrite(PWM_PIN, HIGH); // Start steady-state pulse
             timerTimestamp = millis();
             currentCalibrationState = CalibrationState::STEADY_STATE_PULSE;
             Serial.print("Calibration: Steady-state pulse ("); Serial.print(STEADY_STATE_PULSE_MS); Serial.println(" ms)");
             break;
        case CalibrationState::STEADY_STATE_PULSE:
             if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                noInterrupts(); V_cal = batteryVoltage; I_cal = batteryCurrent; interrupts();
                digitalWrite(PWM_PIN, LOW); // End pulse
                timerTimestamp = millis();
                currentCalibrationState = CalibrationState::STEADY_STATE_SETTLE;
                Serial.print(" -> V_cal = "); Serial.print(V_cal, 4); Serial.print(" V, I_cal = "); Serial.println(I_cal, 4);
                if (fabs(I_cal) < 1e-6) { Serial.println("Calibration Warning: Measured calibration current is near zero."); }
             }
             break;
        case CalibrationState::STEADY_STATE_SETTLE:
             if (checkTimer(timerTimestamp, STEADY_STATE_SETTLE_MS)) {
                 currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS;
             }
             break;
        case CalibrationState::START_DYNAMIC_MEAS:
             Serial.println("Calibration: Starting dynamic measurement phase...");
             startDynamicMeasurement(); // Initiate the dynamic response measurement
             currentCalibrationState = CalibrationState::DYNAMIC_MEASURE;
             break;
        case CalibrationState::DYNAMIC_MEASURE:
             measureDynamicResponse_Tick(dynamicResponseResult);
             if (getDynamicResponseResult(dynamicResponseResult)) {
                 if(dynamicResponseResult.valid) { currentCalibrationState = CalibrationState::CALCULATING; }
                 else { Serial.println("Calibration Error: Dynamic measurement failed."); currentCalibrationState = CalibrationState::ERROR; }
             } else if (currentDynamicState == DynamicMeasureState::ERROR) { Serial.println("Calibration Error: Dynamic measurement entered error state."); currentCalibrationState = CalibrationState::ERROR; }
             break;


        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments (V4)...");
             float T_kelvin = batteryTemperatureC + 273.15;

             // --- Pre-calculation ---
             float V_H2_eq = V_H2_EVOL_STD;
             float V_O2_eq = V_O2_EVOL_STD;
             float V_cell_eq = V_O2_eq - V_H2_eq; // Theoretical equilibrium cell voltage
             float V_electrodes = V_cal - I_cal * internalResistance; // Voltage across interfaces
             float eta_total = V_electrodes - V_cell_eq; // Total overpotential

             float eta_O2_cal = 0.0; // Result of bisection
             float eta_H2_cal = 0.0; // Result of bisection
             float I_model_consistent = 0.0; // Current predicted by model at found split
             bool split_found = false;

             Serial.print("  V_cal="); Serial.print(V_cal, 4); Serial.print(", I_cal="); Serial.print(I_cal, 4);
             Serial.print(", R_int="); Serial.print(internalResistance, 4); Serial.print(", T="); Serial.println(T_kelvin, 2);
             Serial.print("  V_cell_eq="); Serial.print(V_cell_eq, 4); Serial.print(", V_electrodes="); Serial.print(V_electrodes, 4);
             Serial.print(", eta_total="); Serial.println(eta_total, 4);


             // === Part 0: Find Consistent Overpotential Split (Bisection Method) ===
             Serial.println("--- Step 0: Finding Overpotential Split ---");
             if (eta_total <= 1e-6) { // If total overpotential is negligible or negative
                 Serial.println("  Warning: Total overpotential is negligible or negative. Assuming zero split, skipping Area adjust.");
                 eta_O2_cal = 0.0;
                 eta_H2_cal = 0.0;
                 I_model_consistent = 0.0; // Cannot determine model current for area scaling
                 split_found = true; // Allow proceeding to i0 adjustment, but area adjust will be skipped.
             } else if (A_NODE <= 0 || A_CATHODE <= 0 || i0_O2 <= 0 || i0_H2 <= 0) {
                 Serial.println("  Warning: Zero/negative model parameter (Area/i0). Cannot perform split search.");
                 // Keep split_found = false
             }
             else {
                 // Bisection search for eta_O2 within the range [0, eta_total]
                 float eta_O2_low = 0.0;
                 float eta_O2_high = eta_total;
                 float current_diff = 0.0;

                 for (int i = 0; i < BISECTION_ITERATIONS; ++i) {
                     eta_O2_cal = (eta_O2_low + eta_O2_high) / 2.0;
                     eta_H2_cal = eta_O2_cal - eta_total; // eta_H2 is negative

                     // Predict currents at this split using *current* model parameters
                     float i_O2_pred = calculate_current_density(eta_O2_cal, i0_O2, n_O2, alpha, T_kelvin);
                     float i_H2_pred = calculate_current_density(eta_H2_cal, i0_H2, n_H2, alpha, T_kelvin);

                     float I_anode_pred = i_O2_pred * A_NODE;
                     float I_cathode_pred = fabs(i_H2_pred * A_CATHODE); // Use absolute value for comparison

                     current_diff = I_anode_pred - I_cathode_pred;

                     // Debug print inside loop (optional, can be verbose)
                     // Serial.print("  Iter "); Serial.print(i); Serial.print(": eta_O2="); Serial.print(eta_O2_cal, 5);
                     // Serial.print(", I_anode="); Serial.print(I_anode_pred, 5); Serial.print(", I_cathode="); Serial.print(I_cathode_pred, 5);
                     // Serial.print(", diff="); Serial.println(current_diff, 5);

                     if (fabs(current_diff) < BISECTION_TOLERANCE) {
                         split_found = true; // Converged
                         break;
                     }

                     // Adjust search range
                     if (current_diff > 0) { // Anode current too high -> eta_O2 is too high relative to |eta_H2|
                         eta_O2_high = eta_O2_cal;
                     } else { // Cathode current too high -> eta_O2 is too low relative to |eta_H2|
                         eta_O2_low = eta_O2_cal;
                     }
                 } // End bisection loop

                 if (split_found) {
                     Serial.print("  Bisection converged: eta_O2 = "); Serial.print(eta_O2_cal, 5);
                     Serial.print(" V, eta_H2 = "); Serial.println(eta_H2_cal, 5);
                     // Calculate the model current at this converged split
                     float i_O2_final = calculate_current_density(eta_O2_cal, i0_O2, n_O2, alpha, T_kelvin);
                     I_model_consistent = i_O2_final * A_NODE; // This should be ~ equal to cathode current now
                     Serial.print("  Consistent Model Current (I_model_consistent): "); Serial.println(I_model_consistent, 6);

                 } else {
                     Serial.println("  Warning: Bisection for overpotential split did not converge within tolerance/iterations.");
                     // Use the last calculated value as a fallback? Or revert to 50/50? Best to skip calibration?
                     // For now, we'll signal failure by keeping split_found = false
                 }
             }


             // === Part 1: Adjust Area (only if split was found & I_model is valid) ===
             Serial.println("--- Step 1: Adjusting Area (Steady State) ---");
             float correctionFactorArea = 1.0; // Default to no correction
             if (!split_found) {
                  Serial.println("  Skipping Area adjustment: Overpotential split not determined.");
             } else if (fabs(I_model_consistent) < 1e-9) { // Avoid division by zero/tiny number
                  Serial.println("  Skipping Area adjustment: Consistent model current is near zero.");
             } else if (fabs(I_cal) < 1e-9) {
                  Serial.println("  Skipping Area adjustment: Measured calibration current is near zero.");
             }
             else {
                 // Calculate Area correction factor based on measured vs consistent model current
                 correctionFactorArea = safeDivide(I_cal, I_model_consistent, 1.0f);

                 const float MAX_CORRECTION_AREA = 2.0;
                 const float MIN_CORRECTION_AREA = 0.5;
                 if (correctionFactorArea > MAX_CORRECTION_AREA) correctionFactorArea = MAX_CORRECTION_AREA;
                 if (correctionFactorArea < MIN_CORRECTION_AREA) correctionFactorArea = MIN_CORRECTION_AREA;

                 Serial.print("  Area Correction Factor: "); Serial.println(correctionFactorArea, 4);
             }
             // Apply correction factor (even if 1.0)
             float A_CATHODE_prev = A_CATHODE;
             float A_NODE_prev = A_NODE;
             A_CATHODE *= correctionFactorArea;
             A_NODE *= correctionFactorArea;
             Serial.print("  A_CATHODE: "); Serial.print(A_CATHODE_prev, 6); Serial.print(" -> "); Serial.println(A_CATHODE, 6);
             Serial.print("  A_NODE:    "); Serial.print(A_NODE_prev, 6); Serial.print(" -> "); Serial.println(A_NODE, 6);


             // === Part 2: Adjust i0 (Dynamic Response - using *updated* Areas) ===
             Serial.println("--- Step 2: Adjusting i0 (Dynamic Response) ---");
             // This part remains identical to V3, using the potentially updated A_NODE/A_CATHODE
             if (dynamicResponseResult.timeConstantMs <= 0) { /* ... skip i0 adjust ... */ }
             else if (A_CATHODE <= 0 || A_NODE <= 0) { /* ... skip i0 adjust ... */ }
             else {
                 float C_dl_total_estimated = C_DL_SPECIFIC * (A_NODE + A_CATHODE);
                 if (C_dl_total_estimated <= 0) { /* ... skip i0 adjust ... */ }
                 else {
                      float R_ct_estimated = (dynamicResponseResult.timeConstantMs / 1000.0) / C_dl_total_estimated;
                      float R_ct_H2_theoretical = safeDivide((R * T_kelvin), (n_H2 * F * i0_H2 * A_CATHODE), std::numeric_limits<float>::max());
                      float R_ct_O2_theoretical = safeDivide((R * T_kelvin), (n_O2 * F * i0_O2 * A_NODE), std::numeric_limits<float>::max());
                      float R_ct_theoretical_total = R_ct_H2_theoretical + R_ct_O2_theoretical;

                      if(fabs(R_ct_estimated) < std::numeric_limits<float>::epsilon()) { /* ... skip i0 adjust ... */ }
                      else {
                            float correctionFactor_i0 = safeDivide(R_ct_theoretical_total, R_ct_estimated, 1.0f);
                            const float MAX_CORRECTION_I0 = 3.0; const float MIN_CORRECTION_I0 = 0.33;
                            if (correctionFactor_i0 > MAX_CORRECTION_I0) correctionFactor_i0 = MAX_CORRECTION_I0;
                            if (correctionFactor_i0 < MIN_CORRECTION_I0) correctionFactor_i0 = MIN_CORRECTION_I0;
                            float i0_H2_prev = i0_H2; float i0_O2_prev = i0_O2;
                            i0_H2 *= correctionFactor_i0; i0_O2 *= correctionFactor_i0;
                            const float MIN_I0_VALUE = 1e-9;
                            if (i0_H2 < MIN_I0_VALUE) i0_H2 = MIN_I0_VALUE; if (i0_O2 < MIN_I0_VALUE) i0_O2 = MIN_I0_VALUE;

                            // ... (Keep Serial.print lines for i0 adjustment details from V3) ...
                            Serial.print("  Measured Time Constant: "); Serial.print(dynamicResponseResult.timeConstantMs, 2); Serial.println(" ms");
                            Serial.print("  Estimated C_dl_total: "); Serial.println(C_dl_total_estimated, 6);
                            Serial.print("  Estimated R_ct (meas): "); Serial.println(R_ct_estimated, 6);
                            Serial.print("  Theoretical R_ct (model): "); Serial.println(R_ct_theoretical_total, 6);
                            Serial.print("  i0 Correction Factor: "); Serial.println(correctionFactor_i0, 4);
                            Serial.print("  i0_H2: "); Serial.print(i0_H2_prev, 6); Serial.print(" -> "); Serial.println(i0_H2, 6);
                            Serial.print("  i0_O2: "); Serial.print(i0_O2_prev, 6); Serial.print(" -> "); Serial.println(i0_O2, 6);
                      }
                 }
             }

             // --- Final Output ---
             Serial.println("--- Calibration V4 Complete ---");
             // ... (Print final parameters A_CATHODE, A_NODE, i0_H2, i0_O2) ...
             Serial.println("Final Parameters:");
             Serial.print(" A_CATHODE: "); Serial.println(A_CATHODE, 6);
             Serial.print(" A_NODE:    "); Serial.println(A_NODE, 6);
             Serial.print(" i0_H2:     "); Serial.println(i0_H2, 6);
             Serial.print(" i0_O2:     "); Serial.println(i0_O2, 6);
             Serial.println("-----------------------------");

             currentCalibrationState = CalibrationState::COMPLETE;
             break; // End of CALCULATING case


        case CalibrationState::COMPLETE:
             // ... (same as V3) ...
             break;

        case CalibrationState::ERROR:
             // ... (same as V3) ...
             Serial.println("Calibration: Error State.");
             break;
    } // End switch (currentCalibrationState)

    return (currentCalibrationState != CalibrationState::IDLE &&
            currentCalibrationState != CalibrationState::COMPLETE &&
            currentCalibrationState != CalibrationState::ERROR);
}

// --- Rest of the code (setup, loop, mockExternalUpdate, etc.) remains the same as V3 ---
// Remember to include implementations for checkTimer, safeDivide, computeMax... etc.

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("ESP32 Electrochemical Calibration V4 Started");
    Serial.print("Assumed C_DL_SPECIFIC: "); Serial.print(C_DL_SPECIFIC); Serial.println(" F/m^2");
    // ... (rest of setup) ...
     pinMode(PWM_PIN, OUTPUT);
     digitalWrite(PWM_PIN, LOW);
    // lastExternalUpdate = millis(); // Assuming mockExternalUpdate is used
}


// --- Mock external update variables --- (If using)
volatile float batteryVoltage = 3.7;
volatile float batteryCurrent = 0.0;
unsigned long lastExternalUpdate = 0;
void mockExternalUpdate() { /* ... (same as V2/V3) ... */ }


void loop() {
    // mockExternalUpdate(); // Call if using mock update

    noInterrupts();
    float currentVoltage = batteryVoltage; // Read volatile vars
    float currentCurrent = batteryCurrent;
    interrupts();

    static float internalResistance = 0.05;
    static float batteryTempC = 25.0;
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
    unsigned long predictionInterval = 5000;

    if (!calibrating && checkTimer(lastPredictionTime, predictionInterval)) {
         lastPredictionTime = millis();
         // Use a reasonable 'idle' voltage for prediction if possible
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

    delay(10);
}

// --- Include required helper functions ---
// bool checkTimer(...) { ... }
// float safeDivide(...) { ... }
// void startCalibration() { ... } // (Implementation from V2/V3)
// bool isCalibrationComplete() { ... } // (Implementation from V2/V3)
// void startDynamicMeasurement() { ... } // (Implementation from V2/V3)
// bool getDynamicResponseResult(...) { ... } // (Implementation from V2/V3)
// bool measureDynamicResponse_Tick(...) { ... } // (Implementation from V2/V3)
// float computeMaxElectrolysisCurrent(...) { ... } // (Implementation from V2/V3)
