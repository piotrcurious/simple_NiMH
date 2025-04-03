#include <Arduino.h>
#include <limits> // Required for numeric_limits
#include <cmath>  // Required for fabs, exp, pow

// ... (Pin definitions, constants F, R, alpha, n_H2, n_O2, E_OVERPOT_THRESHOLD, C_DL_SPECIFIC) ...
#define PWM_PIN  5
const unsigned long EXTERNAL_UPDATE_INTERVAL_MS = 500;
const float F = 96485.33212; const float R = 8.3145; const float alpha = 0.5;
const float n_H2 = 2.0; const float n_O2 = 4.0;
const float E_OVERPOT_THRESHOLD = 0.1;
const float C_DL_SPECIFIC = 0.2; // F/m^2 - ** Tune this value **

// --- Standard Potentials (Used for Initialization & Reset) ---
const float V_H2_EVOL_STD_DEFAULT = -0.83; // V vs SHE
const float V_O2_EVOL_STD_DEFAULT = 0.40;  // V vs SHE

// --- Calibratable Equilibrium Potentials & Measured R_internal ---
float V_H2_eff_std = V_H2_EVOL_STD_DEFAULT; // Effective H2 potential (calibratable)
float V_O2_eff_std = V_O2_EVOL_STD_DEFAULT; // Effective O2 potential (calibratable)
float R_internal_measured = 0.05; // Measured internal resistance (Ohms, updated by GI) - Initialize with a guess

// --- Global Measured Battery Values ---
extern volatile float batteryVoltage;
extern volatile float batteryCurrent;

// --- Electrochemical Model Parameters (Modifiable via Calibration) ---
float A_NODE = 1.2e-4; float A_CATHODE = 1.5e-4;
float i0_H2 = 1e-6; float i0_O2 = 2e-6;

// --- Calibration & Measurement State ---
enum class CalibrationState { IDLE, START_CALIBRATION, STEADY_STATE_PULSE, GI_MEASURE, STEADY_STATE_SETTLE, START_DYNAMIC_MEAS, DYNAMIC_MEASURE, CALCULATING, COMPLETE, ERROR };
CalibrationState currentCalibrationState = CalibrationState::IDLE;
enum class DynamicMeasureState { IDLE, START_MEASUREMENT, BASELINE_WAIT, PULSE, SETTLE, RECORDING, COMPLETE, ERROR };
DynamicMeasureState currentDynamicState = DynamicMeasureState::IDLE;

// DynamicResponse Struct (Keep as before)
struct DynamicResponse { /* ... */ };

// --- Helper Functions (checkTimer, safeDivide, calculate_current_density - Keep as before) ---
bool checkTimer(unsigned long &timerTimestamp, unsigned long interval) { return (millis() - timerTimestamp >= interval); }
float safeDivide(float numerator, float denominator, float fallback = 0.0f) { /* ... */ }
float calculate_current_density(float eta, float i0, float n, float alpha, float T_kelvin) { /* ... */ }

// --- computeMaxElectrolysisCurrent() - Modified to use calibrated potentials & R_internal ---
float computeMaxElectrolysisCurrent(float currentIdleVoltage, float batteryTemperatureC) {
    // Now uses global R_internal_measured and V_eff_std values
    if (R_internal_measured <= 0) {
         Serial.println("Error: Invalid internal resistance in computeMaxElectrolysisCurrent.");
         return 0.0;
    }
    float T_kelvin = batteryTemperatureC + 273.15;
    // Use effective potentials
    float V_H2_eq = V_H2_eff_std; // Simplified Nernst for now
    float V_O2_eq = V_O2_eff_std; // Simplified Nernst for now
    float eta_H2_thresh = -E_OVERPOT_THRESHOLD; float eta_O2_thresh = E_OVERPOT_THRESHOLD;

     float i_H2_at_thresh = calculate_current_density(eta_H2_thresh, i0_H2, n_H2, alpha, T_kelvin);
     float i_O2_at_thresh = calculate_current_density(eta_O2_thresh, i0_O2, n_O2, alpha, T_kelvin);

    float I_H2_thresh = fabs(i_H2_at_thresh) * A_CATHODE;
    float I_O2_thresh = i_O2_at_thresh * A_NODE;
    float I_electrolysis_thresh = I_H2_thresh + I_O2_thresh;

    if (I_electrolysis_thresh < 0) { /* ... warning ... */ return 0.0; }
    return I_electrolysis_thresh;
}

// --- measureDynamicResponse_Tick() and related helpers (Keep as before) ---
bool measureDynamicResponse_Tick(DynamicResponse &response) { /* ... */ }
void startDynamicMeasurement() { /* ... */ }
bool getDynamicResponseResult(DynamicResponse &response) { /* ... */ }


//--------------------------------------------------------------
// calibrateEquilibriumPotentials() - NEW FUNCTION
//
// Adjusts the effective standard potentials based on measured OCV.
// Call this when the system has been idle for a long time.
//--------------------------------------------------------------
void calibrateEquilibriumPotentials(float measuredOCV) {
    Serial.println("--- Calibrating Equilibrium Potentials ---");
    Serial.print("  Measured OCV: "); Serial.println(measuredOCV, 4);

    // Calculate the nominal standard cell potential
    float V_cell_std_nominal = V_O2_EVOL_STD_DEFAULT - V_H2_EVOL_STD_DEFAULT;
    Serial.print("  Nominal Std Cell Potential: "); Serial.println(V_cell_std_nominal, 4);

    // Calculate the deviation
    float deviation = measuredOCV - V_cell_std_nominal;
    Serial.print("  Deviation from Nominal: "); Serial.println(deviation, 4);

    // Apply symmetric shift to the effective standard potentials
    // Store previous values for logging
    float V_O2_prev = V_O2_eff_std;
    float V_H2_prev = V_H2_eff_std;

    V_O2_eff_std = V_O2_EVOL_STD_DEFAULT + deviation / 2.0;
    V_H2_eff_std = V_H2_EVOL_STD_DEFAULT - deviation / 2.0; // Minus deviation for H2

    // Add potential sanity checks/limits if needed
    // e.g., if (fabs(deviation) > 0.5) Serial.println("Warning: Large OCV deviation.");

    Serial.print("  V_O2_eff_std: "); Serial.print(V_O2_prev, 4); Serial.print(" -> "); Serial.println(V_O2_eff_std, 4);
    Serial.print("  V_H2_eff_std: "); Serial.print(V_H2_prev, 4); Serial.print(" -> "); Serial.println(V_H2_eff_std, 4);
    Serial.println("-----------------------------------------");
}


//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V7 (GI for R_internal)
//
// Uses GI to measure R_internal during calibration.
// Uses calibrated effective equilibrium potentials.
// Continues with V6 logic for i0/Area adjustment.
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float batteryTemperatureC) { // Removed R_internal input, removed idleVoltage (less critical now)

    const unsigned long STEADY_STATE_PULSE_MS = 2000;
    const unsigned long GI_MEASURE_DELAY_US = 50; // Very short delay after turning off before measuring V_drop
    const unsigned long STEADY_STATE_SETTLE_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50;
    const int BISECTION_ITERATIONS = 15;
    const float BISECTION_TOLERANCE = 1e-4;

    static unsigned long timerTimestamp;
    static float V_before_off, I_cal; // Store values for GI
    static DynamicResponse dynamicResponseResult;

    switch (currentCalibrationState) {
        case CalibrationState::IDLE: break;

        case CalibrationState::START_CALIBRATION:
             Serial.println("Calibration: Starting (V7 - GI R_internal & Eff Potentials)...");
             // Reset measured R_internal? Or keep last value? Keep last for now.
             pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, HIGH); timerTimestamp = millis();
             currentCalibrationState = CalibrationState::STEADY_STATE_PULSE;
             Serial.print("Calibration: Steady-state pulse ("); Serial.print(STEADY_STATE_PULSE_MS); Serial.println(" ms)");
             break;

        case CalibrationState::STEADY_STATE_PULSE:
             if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                 // Read values just *before* turning off pulse
                 noInterrupts(); V_before_off = batteryVoltage; I_cal = batteryCurrent; interrupts();
                 digitalWrite(PWM_PIN, LOW); // <<< --- CURRENT INTERRUPTED HERE ---
                 // Immediately transition to GI measurement state
                 timerTimestamp = micros(); // Use micros for short delay
                 currentCalibrationState = CalibrationState::GI_MEASURE;
                 Serial.print(" -> Pulse End: V_before_off = "); Serial.print(V_before_off, 4);
                 Serial.print(" V, I_cal = "); Serial.println(I_cal, 4);
                 if (fabs(I_cal) < 1e-6) { Serial.println("  Warning: I_cal near zero. R_internal measurement will be unreliable."); }
             }
             break;

         case CalibrationState::GI_MEASURE:
             // Wait a tiny amount for PWM to be fully off, then measure voltage drop
             if ((micros() - timerTimestamp) >= GI_MEASURE_DELAY_US) {
                 float V_after_off;
                 noInterrupts(); V_after_off = batteryVoltage; interrupts(); // Read V immediately after interruption
                 Serial.print(" -> GI Measure: V_after_off = "); Serial.println(V_after_off, 4);

                 // Calculate R_internal from instantaneous voltage drop
                 if (fabs(I_cal) > 1e-6) { // Avoid division by zero
                      float V_drop = V_before_off - V_after_off;
                      // Check if V_drop is reasonable (e.g., positive)
                      if(V_drop < 0) {
                           Serial.println("  Warning: V_drop during GI is negative. Check measurement timing/noise.");
                           // Keep previous R_internal_measured value
                      } else {
                           float R_internal_new = safeDivide(V_drop, I_cal, R_internal_measured); // Update R_internal
                           // Add sanity limits to measured R_internal
                           const float MIN_R_INTERNAL = 0.001; const float MAX_R_INTERNAL = 1.0;
                           if (R_internal_new < MIN_R_INTERNAL) R_internal_new = MIN_R_INTERNAL;
                           if (R_internal_new > MAX_R_INTERNAL) R_internal_new = MAX_R_INTERNAL;

                           Serial.print("  -> Measured R_internal: "); Serial.print(R_internal_new, 4); Serial.println(" Ohms");
                           // Optional: Apply smoothing/filtering (e.g., moving average)
                           // R_internal_measured = 0.8 * R_internal_measured + 0.2 * R_internal_new; // Example filter
                           R_internal_measured = R_internal_new; // Direct update for now
                      }
                 } else {
                     Serial.println("  Skipping R_internal update: I_cal was near zero.");
                 }
                 // Proceed to settling state
                 timerTimestamp = millis(); // Reset timer for settling period
                 currentCalibrationState = CalibrationState::STEADY_STATE_SETTLE;
             }
             break;

        case CalibrationState::STEADY_STATE_SETTLE:
             // Wait for system to settle (and maybe capture decay curve later)
             if (checkTimer(timerTimestamp, STEADY_STATE_SETTLE_MS)) {
                 currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS;
             }
             break;

        case CalibrationState::START_DYNAMIC_MEAS: /* ... (Same as V6) ... */ break;
        case CalibrationState::DYNAMIC_MEASURE: /* ... (Same as V6) ... */ break;

        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments (V7)...");
             // Use the R_internal_measured determined via GI
             Serial.print("  Using R_internal = "); Serial.println(R_internal_measured, 4);
             // Use the calibrated V_H2_eff_std, V_O2_eff_std
             Serial.print("  Using V_H2_eff_std = "); Serial.print(V_H2_eff_std, 4);
             Serial.print(", V_O2_eff_std = "); Serial.println(V_O2_eff_std, 4);

             float T_kelvin = batteryTemperatureC + 273.15;
             float A_CATHODE_start = A_CATHODE; float A_NODE_start = A_NODE;
             float i0_H2_start = i0_H2; float i0_O2_start = i0_O2;

             // === Part 0: Adjust i0 Individually (Dynamic R_ct) ===
             // ... (Identical logic to V6, using current A, i0) ...
             Serial.println("--- Step 0: Adjusting i0 Individually (Dynamic Response) ---");
             float correctionFactor_i0_H2 = 1.0; float correctionFactor_i0_O2 = 1.0;
             // ... (calculations for R_ct_estimated_sum, R_ct_H2/O2_theory, delta_R, correction factors) ...
              if (dynamicResponseResult.timeConstantMs <= 0) { /* Skip */ }
              else if (A_CATHODE <= 0 || A_NODE <= 0 || i0_H2 <= 0 || i0_O2 <= 0) { /* Skip */ }
              else { /* ... Calculate C_dl ... */ float C_dl_total_estimated = C_DL_SPECIFIC * (A_NODE + A_CATHODE);
                  if (C_dl_total_estimated <=0) {/* Skip */}
                  else { /* ... Calculate R_ct_estimated_sum ... */ float R_ct_estimated_sum = (dynamicResponseResult.timeConstantMs / 1000.0) / C_dl_total_estimated;
                      /* ... Calculate R_ct_H2/O2/Total_theory ... */
                      float R_ct_H2_theory = safeDivide((R * T_kelvin), (n_H2 * F * i0_H2 * A_CATHODE), std::numeric_limits<float>::max());
                      float R_ct_O2_theory = safeDivide((R * T_kelvin), (n_O2 * F * i0_O2 * A_NODE), std::numeric_limits<float>::max());
                      float R_ct_theory_total = R_ct_H2_theory + R_ct_O2_theory;
                      if (/* ... Check R_ct sums > eps ... */ fabs(R_ct_estimated_sum) >= 1e-9 && fabs(R_ct_theory_total) >= 1e-9) {
                          /* ... Calculate delta_R, denom_R_ct_sq_sum ... */
                          float delta_R = R_ct_estimated_sum - R_ct_theory_total;
                          float R_ct_H2_sq = pow(R_ct_H2_theory, 2); float R_ct_O2_sq = pow(R_ct_O2_theory, 2);
                          float denom_R_ct_sq_sum = R_ct_H2_sq + R_ct_O2_sq;
                          if (fabs(denom_R_ct_sq_sum) >= 1e-12 /* Adjust tolerance */) {
                              correctionFactor_i0_H2 = 1.0 - delta_R * R_ct_H2_theory / denom_R_ct_sq_sum;
                              correctionFactor_i0_O2 = 1.0 - delta_R * R_ct_O2_theory / denom_R_ct_sq_sum;
                              /* ... Apply Limits ... */
                              const float MAX_CORRECTION_I0 = 3.0; const float MIN_CORRECTION_I0 = 0.33;
                              if (correctionFactor_i0_H2 > MAX_CORRECTION_I0) correctionFactor_i0_H2 = MAX_CORRECTION_I0; if (correctionFactor_i0_H2 < MIN_CORRECTION_I0) correctionFactor_i0_H2 = MIN_CORRECTION_I0;
                              if (correctionFactor_i0_O2 > MAX_CORRECTION_I0) correctionFactor_i0_O2 = MAX_CORRECTION_I0; if (correctionFactor_i0_O2 < MIN_CORRECTION_I0) correctionFactor_i0_O2 = MIN_CORRECTION_I0;
                          } else { Serial.println(" Warning: Sum of sq R_ct near zero."); }
                      } else { Serial.println(" Warning: R_ct sums near zero."); }
                  }
              } // End i0 adjustment logic block
             // ... (Apply i0 corrections, MIN_I0_VALUE, logging) ...
             i0_H2 *= correctionFactor_i0_H2; i0_O2 *= correctionFactor_i0_O2;
             const float MIN_I0_VALUE = 1e-9; if (i0_H2 < MIN_I0_VALUE) i0_H2 = MIN_I0_VALUE; if (i0_O2 < MIN_I0_VALUE) i0_O2 = MIN_I0_VALUE;
             Serial.print("  i0_H2: "); Serial.print(i0_H2_start, 6); Serial.print(" -> "); Serial.println(i0_H2, 6); Serial.print("  i0_O2: "); Serial.print(i0_O2_start, 6); Serial.print(" -> "); Serial.println(i0_O2, 6);

             // === Part 1: Find Overpotential Split (using updated i0 & R_internal_measured) ===
             Serial.println("--- Step 1: Finding Overpotential Split ---");
             // Use calibrated effective potentials
             float V_H2_eq = V_H2_eff_std; float V_O2_eq = V_O2_eff_std; float V_cell_eq = V_O2_eq - V_H2_eq;
             // Use measured R_internal
             float V_electrodes = V_cal - I_cal * R_internal_measured;
             float eta_total = V_electrodes - V_cell_eq;
             float eta_O2_cal = 0.0; float eta_H2_cal = 0.0; bool split_found = false;
             // ... (Bisection search logic identical to V6, uses updated i0, current A) ...
              Serial.print("  eta_total = "); Serial.println(eta_total, 4);
              if (eta_total <= 1e-6) { /* ... */ split_found = true; }
              else if (A_NODE <= 0 || A_CATHODE <= 0 || i0_O2 <= 0 || i0_H2 <= 0) { /* ... */ }
              else { /* ... Bisection Loop ... */
                  float eta_O2_low = 0.0; float eta_O2_high = eta_total; float current_diff = 0.0;
                  for (int i = 0; i < BISECTION_ITERATIONS; ++i) { eta_O2_cal = (eta_O2_low + eta_O2_high) / 2.0; eta_H2_cal = eta_O2_cal - eta_total;
                      float i_O2_pred = calculate_current_density(eta_O2_cal, i0_O2, n_O2, alpha, T_kelvin); float i_H2_pred = calculate_current_density(eta_H2_cal, i0_H2, n_H2, alpha, T_kelvin);
                      float I_anode_pred = i_O2_pred * A_NODE; float I_cathode_pred = fabs(i_H2_pred * A_CATHODE);
                      current_diff = I_anode_pred - I_cathode_pred;
                      if (fabs(current_diff) < BISECTION_TOLERANCE) { split_found = true; break; } if (current_diff > 0) { eta_O2_high = eta_O2_cal; } else { eta_O2_low = eta_O2_cal; }
                  } if (!split_found) { Serial.println(" Warning: Bisection did not converge."); }
              } if(split_found && eta_total > 1e-6) { Serial.print("  Bisection converged: eta_O2 = "); Serial.print(eta_O2_cal, 5); Serial.print(" V, eta_H2 = "); Serial.println(eta_H2_cal, 5); }


             // === Part 2: Adjust Area (using split found with updated i0) ===
             Serial.println("--- Step 2: Adjusting Area (Steady State) ---");
             // ... (Identical logic to V6, calculates I_model_consistent, correctionFactorArea, applies it) ...
             float correctionFactorArea = 1.0; float I_model_consistent = 0.0;
             if (!split_found || eta_total <= 1e-6) { /* Skip */ }
             else if (A_NODE <= 0 || A_CATHODE <= 0 || i0_O2 <= 0 || i0_H2 <= 0) { /* Skip */ }
             else { /* Calculate I_model_consistent using updated i0, current A */
                 float i_O2_final = calculate_current_density(eta_O2_cal, i0_O2, n_O2, alpha, T_kelvin); I_model_consistent = i_O2_final * A_NODE;
                 Serial.print("  Consistent Model Current (I_model_consistent): "); Serial.println(I_model_consistent, 6);
                 if (fabs(I_model_consistent) >= 1e-9 && fabs(I_cal) >= 1e-9) { /* Calculate correctionFactorArea */
                     correctionFactorArea = safeDivide(I_cal, I_model_consistent, 1.0f);
                     const float MAX_CORRECTION_AREA = 2.0; const float MIN_CORRECTION_AREA = 0.5; /* Apply Limits */
                     if (correctionFactorArea > MAX_CORRECTION_AREA) correctionFactorArea = MAX_CORRECTION_AREA; if (correctionFactorArea < MIN_CORRECTION_AREA) correctionFactorArea = MIN_CORRECTION_AREA;
                     Serial.print("  Area Correction Factor: "); Serial.println(correctionFactorArea, 4);
                 } else { /* Skip */ }
             }
             A_CATHODE *= correctionFactorArea; A_NODE *= correctionFactorArea; /* Apply */
             Serial.print("  A_CATHODE: "); Serial.print(A_CATHODE_start, 6); Serial.print(" -> "); Serial.println(A_CATHODE, 6); Serial.print("  A_NODE: "); Serial.print(A_NODE_start, 6); Serial.print(" -> "); Serial.println(A_NODE, 6);

             // --- Final Output ---
             Serial.println("--- Calibration V7 Complete ---");
             /* ... (Print final A, i0 parameters) ... */
              Serial.println("Final Parameters:");
              Serial.print(" A_CATHODE: "); Serial.println(A_CATHODE, 6); Serial.print(" A_NODE: "); Serial.println(A_NODE, 6);
              Serial.print(" i0_H2: "); Serial.println(i0_H2, 6); Serial.print(" i0_O2: "); Serial.println(i0_O2, 6);
              Serial.print(" R_internal: "); Serial.println(R_internal_measured, 4);
              Serial.print(" V_H2_eff: "); Serial.print(V_H2_eff_std, 4); Serial.print(", V_O2_eff: "); Serial.println(V_O2_eff_std, 4);
              Serial.println("-----------------------------");

             currentCalibrationState = CalibrationState::COMPLETE;
             break; // End of CALCULATING case

        case CalibrationState::COMPLETE: break;
        case CalibrationState::ERROR: Serial.println("Calibration: Error State."); break;
    } // End switch

    r
