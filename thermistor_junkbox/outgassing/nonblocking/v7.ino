#include <Arduino.h>
#include <limits> // Required for numeric_limits
#include <cmath>  // Required for fabs, exp, pow, log

// ... (Pin definitions, constants F, R, alpha, n_H2, n_O2, E_OVERPOT_THRESHOLD, C_DL_SPECIFIC) ...
#define PWM_PIN  5
const unsigned long EXTERNAL_UPDATE_INTERVAL_MS = 100; // <<< Updated as per prompt
const float F = 96485.33212; const float R = 8.3145; const float alpha = 0.5;
const float n_H2 = 2.0; const float n_O2 = 4.0;
const float E_OVERPOT_THRESHOLD = 0.1;
const float C_DL_SPECIFIC = 0.2; // F/m^2 - ** Tune this value **

// --- Standard Potentials (Used for Initialization & Reset) ---
const float V_H2_EVOL_STD_DEFAULT = -0.83; // V vs SHE
const float V_O2_EVOL_STD_DEFAULT = 0.40;  // V vs SHE

// --- Calibratable Equilibrium Potentials & Measured R_internal ---
float V_H2_eff_std = V_H2_EVOL_STD_DEFAULT;
float V_O2_eff_std = V_O2_EVOL_STD_DEFAULT;
float R_internal_measured = 0.05; // Ohms - Updated by GI

// --- Global Measured Battery Values ---
extern volatile float batteryVoltage;
extern volatile float batteryCurrent;

// --- Electrochemical Model Parameters (Modifiable via Calibration) ---
float A_NODE = 1.2e-4; float A_CATHODE = 1.5e-4;
float i0_H2 = 1e-6; float i0_O2 = 2e-6;

// --- Decay Curve Analysis Data ---
const int DECAY_MAX_POINTS = 6; // Max points to capture (e.g., 0ms, 100ms, 200ms .. 500ms)
float decay_times_ms[DECAY_MAX_POINTS];
float decay_voltages[DECAY_MAX_POINTS];
int decay_points_captured = 0;
float tau_decay_ms = -1.0; // Estimated decay time constant, -1 if invalid

// --- Calibration & Measurement State ---
enum class CalibrationState { IDLE, START_CALIBRATION, STEADY_STATE_PULSE, GI_MEASURE, DECAY_CAPTURE, START_DYNAMIC_MEAS, DYNAMIC_MEASURE, CALCULATING, COMPLETE, ERROR }; // Added DECAY_CAPTURE
CalibrationState currentCalibrationState = CalibrationState::IDLE;
enum class DynamicMeasureState { IDLE, START_MEASUREMENT, BASELINE_WAIT, PULSE, SETTLE, RECORDING, COMPLETE, ERROR };
DynamicMeasureState currentDynamicState = DynamicMeasureState::IDLE;

// DynamicResponse Struct - Add tau_dynamic field for clarity
struct DynamicResponse {
  float tau_dynamic_ms; // Renamed from timeConstantMs
  float maxVoltageStep; float maxCurrentStep;
  float steadyStateVoltage; float steadyStateCurrent;
  bool valid = false;
  // We won't store intermediate pulse points due to 100ms sample rate limitation
};

// --- Helper Functions (checkTimer, safeDivide, calculate_current_density - Keep) ---
bool checkTimer(unsigned long &timerTimestamp, unsigned long interval) { /* ... */ }
float safeDivide(float numerator, float denominator, float fallback = 0.0f) { /* ... */ }
float calculate_current_density(float eta, float i0, float n, float alpha, float T_kelvin) { /* ... */ }

// --- computeMaxElectrolysisCurrent() - Uses calibrated potentials & R_internal (Keep V7) ---
float computeMaxElectrolysisCurrent(float currentIdleVoltage, float batteryTemperatureC) { /* ... */ }

// --- measureDynamicResponse_Tick() and helpers (Keep V7, rename result field) ---
// Modify to store result in tau_dynamic_ms
bool measureDynamicResponse_Tick(DynamicResponse &response) {
    // ... (Previous logic) ...
    // In RECORDING state, when calculating results:
    // response.tau_dynamic_ms = /* calculated time constant */; // Instead of timeConstantMs
    // ...
     // Ensure all internal logic refers to tau_dynamic_ms field of the response struct.
     // The calculation itself remains the same approximate method based on 63% rise.
    return (currentDynamicState != DynamicMeasureState::IDLE && /* ... */);
 }
void startDynamicMeasurement() { /* ... */ }
bool getDynamicResponseResult(DynamicResponse &response) { /* ... */ }

// --- calibrateEquilibriumPotentials() (Keep V7) ---
void calibrateEquilibriumPotentials(float measuredOCV) { /* ... */ }


//--------------------------------------------------------------
// analyzeDecayCurve() - NEW FUNCTION
//
// Attempts to estimate tau from captured decay points using linearized fit.
// V(t) = V_final + A * exp(-t / tau)
// ln(V(t) - V_final) = ln(A) - (1/tau) * t  (y = c + m*x)
//--------------------------------------------------------------
float analyzeDecayCurve(float times[], float voltages[], int num_points) {
    Serial.println("--- Analyzing Decay Curve ---");
    if (num_points < 3) { // Need at least 3 points for a meaningful fit attempt
        Serial.println("  Error: Not enough points captured for decay analysis.");
        return -1.0; // Invalid tau
    }

    // Estimate V_final as the last measured voltage point
    float V_final = voltages[num_points - 1];
    Serial.print("  Using V_final estimate: "); Serial.println(V_final, 4);

    // Prepare data for linear regression: x = time, y = ln(V(t) - V_final)
    // We need V(t) > V_final. Skip points where this isn't true or difference is too small.
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n_fit = 0;

    Serial.print("  Points used for fit (t_ms, V, ln(V-Vf)):");
    for (int i = 0; i < num_points - 1; ++i) { // Exclude the last point used as V_final
        float delta_V = voltages[i] - V_final;
        if (delta_V > 1e-4) { // Check if voltage is significantly above V_final & avoid log(<=0)
            float t_sec = times[i] / 1000.0; // Use time in seconds for tau in seconds
            float y = log(delta_V);
            sum_x += t_sec;
            sum_y += y;
            sum_xy += t_sec * y;
            sum_x2 += t_sec * t_sec;
            n_fit++;
            Serial.print(" ("); Serial.print(times[i], 0); Serial.print(", "); Serial.print(voltages[i], 4); Serial.print(", "); Serial.print(y, 4); Serial.print(")");
        }
    }
    Serial.println();

    if (n_fit < 2) { // Need at least 2 points for a line
        Serial.println("  Error: Not enough valid points for linear regression.");
        return -1.0;
    }

    // Calculate slope (m) using standard linear regression formula
    // m = (n * sum(xy) - sum(x) * sum(y)) / (n * sum(x^2) - sum(x)^2)
    float denominator = (n_fit * sum_x2) - (sum_x * sum_x);
    if (fabs(denominator) < 1e-9) {
        Serial.println("  Error: Denominator near zero in regression slope calculation.");
        return -1.0;
    }
    float slope = ((n_fit * sum_xy) - (sum_x * sum_y)) / denominator;

    Serial.print("  Fit slope (m = -1/tau): "); Serial.println(slope, 6);

    // Check if slope is negative (expected for decay)
    if (slope >= 0) {
        Serial.println("  Warning: Decay curve fit slope is non-negative. Invalid tau.");
        return -1.0;
    }

    // Calculate tau = -1 / slope
    float tau_seconds = -1.0 / slope;
    float tau_milliseconds = tau_seconds * 1000.0;

    // Add sanity checks for tau
    if (tau_milliseconds <= 0 || tau_milliseconds > 10000) { // e.g., reject <0ms or >10s
         Serial.print("  Warning: Calculated tau_decay ("); Serial.print(tau_milliseconds, 1); Serial.println(" ms) out of expected range.");
         return -1.0;
    }

    Serial.print("  Estimated tau_decay: "); Serial.print(tau_milliseconds, 2); Serial.println(" ms");
    Serial.println("-----------------------------");
    return tau_milliseconds;
}


//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V8 (Decay Analysis)
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float batteryTemperatureC) {

    const unsigned long STEADY_STATE_PULSE_MS = 2000;
    const unsigned long GI_MEASURE_DELAY_US = 50;
    const unsigned long DECAY_CAPTURE_INTERVAL_MS = EXTERNAL_UPDATE_INTERVAL_MS; // Match external update
    const unsigned long DECAY_CAPTURE_DURATION_MS = (DECAY_MAX_POINTS -1) * DECAY_CAPTURE_INTERVAL_MS;

    static unsigned long timerTimestamp; // Used for multiple states
    static unsigned long interruptionTimestampMs; // Time GI happened
    static float V_before_off, I_cal;
    static DynamicResponse dynamicResponseResult;
    static int decay_capture_index;
    static unsigned long nextDecayCaptureTime;


    switch (currentCalibrationState) {
        case CalibrationState::IDLE: break;

        case CalibrationState::START_CALIBRATION:
             Serial.println("Calibration: Starting (V8 - Decay Analysis)...");
             tau_decay_ms = -1.0; // Reset decay result
             pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, HIGH); timerTimestamp = millis();
             currentCalibrationState = CalibrationState::STEADY_STATE_PULSE;
             /* ... logging ... */
             break;

        case CalibrationState::STEADY_STATE_PULSE:
             if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                 noInterrupts(); V_before_off = batteryVoltage; I_cal = batteryCurrent; interrupts();
                 digitalWrite(PWM_PIN, LOW); // --- CURRENT INTERRUPTED ---
                 interruptionTimestampMs = millis(); // Record interruption time
                 timerTimestamp = micros(); // Switch to micros for GI delay
                 currentCalibrationState = CalibrationState::GI_MEASURE;
                 /* ... logging ... */
             }
             break;

         case CalibrationState::GI_MEASURE:
             if ((micros() - timerTimestamp) >= GI_MEASURE_DELAY_US) {
                 float V_after_off;
                 noInterrupts(); V_after_off = batteryVoltage; interrupts();
                 Serial.print(" -> GI Measure: V_after_off = "); Serial.println(V_after_off, 4);
                 // Calculate R_internal (same as V7)
                 if (fabs(I_cal) > 1e-6) { /* ... Calculate R_internal_measured ... */ }
                 else { /* ... Skip R_internal update ... */ }

                 // Initialize Decay Capture
                 decay_points_captured = 0;
                 decay_times_ms[decay_points_captured] = 0; // Time relative to interruption
                 decay_voltages[decay_points_captured] = V_after_off; // First point is immediate post-GI voltage
                 decay_points_captured++;
                 decay_capture_index = 1; // Start looking for the next point
                 nextDecayCaptureTime = interruptionTimestampMs + DECAY_CAPTURE_INTERVAL_MS * decay_capture_index;
                 currentCalibrationState = CalibrationState::DECAY_CAPTURE;
                 Serial.println(" -> Starting Decay Curve Capture...");
             }
             break;

         case CalibrationState::DECAY_CAPTURE:
             // Check if it's time to capture the next point
             if (millis() >= nextDecayCaptureTime) {
                  if (decay_points_captured < DECAY_MAX_POINTS) {
                       float currentV;
                       noInterrupts(); currentV = batteryVoltage; interrupts();
                       decay_times_ms[decay_points_captured] = millis() - interruptionTimestampMs; // Store relative time
                       decay_voltages[decay_points_captured] = currentV;
                       Serial.print("  Decay point "); Serial.print(decay_points_captured);
                       Serial.print(": t = "); Serial.print(decay_times_ms[decay_points_captured], 0);
                       Serial.print(" ms, V = "); Serial.println(currentV, 4);
                       decay_points_captured++;
                       decay_capture_index++;
                       // Set time for the next capture, ensuring we don't lag too far
                       nextDecayCaptureTime = interruptionTimestampMs + DECAY_CAPTURE_INTERVAL_MS * decay_capture_index;

                  }
             }

             // Check if capture duration is over or max points reached
              if (decay_points_captured >= DECAY_MAX_POINTS || (millis() - interruptionTimestampMs) >= (DECAY_CAPTURE_DURATION_MS + DECAY_CAPTURE_INTERVAL_MS)) {
                 Serial.println(" -> Decay Curve Capture Finished.");
                 // Analyze the captured curve
                 tau_decay_ms = analyzeDecayCurve(decay_times_ms, decay_voltages, decay_points_captured);
                 // Proceed to dynamic measurement phase
                 currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS;
             }
             break;


        case CalibrationState::START_DYNAMIC_MEAS: /* ... (Same as V7) ... */ break;
        case CalibrationState::DYNAMIC_MEASURE: /* ... (Same as V7) ... */ break;

        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments (V8)...");
             Serial.print("  Using R_internal = "); Serial.println(R_internal_measured, 4);
             Serial.print("  Using V_H2_eff_std = "); Serial.print(V_H2_eff_std, 4); /* ... */

             float T_kelvin = batteryTemperatureC + 273.15;
             float A_CATHODE_start = A_CATHODE; /* ... */ float i0_O2_start = i0_O2;

             // === Part 0: Adjust i0 Individually (Dynamic R_ct using best Tau) ===
             Serial.println("--- Step 0: Adjusting i0 Individually ---");
             float correctionFactor_i0_H2 = 1.0; float correctionFactor_i0_O2 = 1.0;
             float tau_used_ms = -1.0;

             // Prioritize tau_decay if valid, otherwise use tau_dynamic
             if (tau_decay_ms > 0) {
                 tau_used_ms = tau_decay_ms;
                 Serial.print("  Using tau_decay = "); Serial.println(tau_used_ms, 2);
             } else if (dynamicResponseResult.valid && dynamicResponseResult.tau_dynamic_ms > 0) {
                 tau_used_ms = dynamicResponseResult.tau_dynamic_ms;
                 Serial.print("  Using tau_dynamic = "); Serial.println(tau_used_ms, 2);
             } else {
                 Serial.println("  Warning: No valid time constant available. Skipping i0 adjustment.");
             }

             if (tau_used_ms <= 0) { /* Skip i0 adjust */ }
             else if (A_CATHODE <= 0 || A_NODE <= 0 || i0_H2 <= 0 || i0_O2 <= 0) { /* Skip i0 adjust */ }
             else {
                 // Logic identical to V7 Step 0, but uses tau_used_ms
                 float C_dl_total_estimated = C_DL_SPECIFIC * (A_NODE + A_CATHODE);
                 if (C_dl_total_estimated <= 0) { /* Skip */ }
                 else {
                      float R_ct_estimated_sum = (tau_used_ms / 1000.0) / C_dl_total_estimated;
                      /* ... Calculate R_ct_H2/O2/Total_theory ... */
                       float R_ct_H2_theory = safeDivide((R * T_kelvin), (n_H2 * F * i0_H2 * A_CATHODE), std::numeric_limits<float>::max());
                       float R_ct_O2_theory = safeDivide((R * T_kelvin), (n_O2 * F * i0_O2 * A_NODE), std::numeric_limits<float>::max());
                       float R_ct_theory_total = R_ct_H2_theory + R_ct_O2_theory;
                      if (/* ... Check R_ct sums > eps ... */) {
                          /* ... Calculate delta_R, denom_R_ct_sq_sum ... */
                           float delta_R = R_ct_estimated_sum - R_ct_theory_total;
                           float R_ct_H2_sq = pow(R_ct_H2_theory, 2); float R_ct_O2_sq = pow(R_ct_O2_theory, 2);
                           float denom_R_ct_sq_sum = R_ct_H2_sq + R_ct_O2_sq;
                          if (/* ... Check denom > eps ... */) {
                              correctionFactor_i0_H2 = 1.0 - delta_R * R_ct_H2_theory / denom_R_ct_sq_sum;
                              correctionFactor_i0_O2 = 1.0 - delta_R * R_ct_O2_theory / denom_R_ct_sq_sum;
                              /* ... Apply Limits ... */
                               const float MAX_CORRECTION_I0 = 3.0; const float MIN_CORRECTION_I0 = 0.33; if (correctionFactor_i0_H2 > MAX_CORRECTION_I0) correctionFactor_i0_H2 = MAX_CORRECTION_I0; if (correctionFactor_i0_H2 < MIN_CORRECTION_I0) correctionFactor_i0_H2 = MIN_CORRECTION_I0; if (correctionFactor_i0_O2 > MAX_CORRECTION_I0) correctionFactor_i0_O2 = MAX_CORRECTION_I0; if (correctionFactor_i0_O2 < MIN_CORRECTION_I0) correctionFactor_i0_O2 = MIN_CORRECTION_I0;
                          } else { /* Warning */ }
                      } else { /* Warning */ }
                 } // End C_dl check
             } // End initial checks
             /* ... Apply i0 corrections, MIN_I0_VALUE, logging ... */
             i0_H2 *= correctionFactor_i0_H2; i0_O2 *= correctionFactor_i0_O2; const float MIN_I0_VALUE = 1e-9; if (i0_H2 < MIN_I0_VALUE) i0_H2 = MIN_I0_VALUE; if (i0_O2 < MIN_I0_VALUE) i0_O2 = MIN_I0_VALUE;
             Serial.print("  i0_H2: "); /* ... Log change ... */ Serial.print("  i0_O2: "); /* ... Log change ... */

             // === Part 1: Find Overpotential Split (using updated i0 & R_internal_measured) ===
             Serial.println("--- Step 1: Finding Overpotential Split ---");
             // ... (Identical logic to V7, uses updated i0, current A, R_internal_measured, V_eff_std) ...
             float V_H2_eq = V_H2_eff_std; float V_O2_eq = V_O2_eff_std; float V_cell_eq = V_O2_eq - V_H2_eq; float V_electrodes = V_cal - I_cal * R_internal_measured; float eta_total = V_electrodes - V_cell_eq; float eta_O2_cal = 0.0; float eta_H2_cal = 0.0; bool split_found = false;
             // ... Bisection search ...
              if (/* ... Conditions for search ... */) { /* Bisection */ } else { /* Handle inability to search */ } if(split_found) { /* Log split */ }


             // === Part 2: Adjust Area (using split found with updated i0) ===
             Serial.println("--- Step 2: Adjusting Area (Steady State) ---");
             // ... (Identical logic to V7, calculates I_model_consistent, correctionFactorArea, applies it) ...
             float correctionFactorArea = 1.0; float I_model_consistent = 0.0;
              if (/* ... Conditions for area adjust ... */) { /* Calc I_model_consistent, correctionFactorArea */ } else { /* Skip */ }
              A_CATHODE *= correctionFactorArea; A_NODE *= correctionFactorArea; /* Apply */
              /* Log area changes */

             // --- Final Output ---
             Serial.println("--- Calibration V8 Complete ---");
             /* ... (Print final A, i0, R_int, V_eff parameters) ... */
              Serial.println("Final Parameters:"); /* ... Print ... */
              Serial.println("-----------------------------");

             currentCalibrationState = CalibrationState::COMPLETE;
             break; // End of CALCULATING case

        case CalibrationState::COMPLETE: break;
        case CalibrationState::ERROR: Serial.println("Calibration: Error State."); break;
    } // End switch

    return (currentCalibrationState != CalibrationState::IDLE && /* ... */);
}

// --- Rest of the code (setup, loop, mock update, helpers etc.) remains the same as V7 ---
// Ensure mockExternalUpdate reflects 100ms interval if used for testing.
// Ensure all helper functions are included.

void setup() {
    Serial.begin(115200); while (!Serial);
    Serial.println("ESP32 Electrochemical Calibration V8 Started");
    Serial.print("External Update Interval: "); Serial.print(EXTERNAL_UPDATE_INTERVAL_MS); Serial.println(" ms");
    Serial.print("Assumed C_DL_SPECIFIC: "); Serial.print(C_DL_SPECIFIC); Serial.println(" F/m^2");
    /* ... rest of setup from V7 ... */
    pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, LOW); lastActivityTime = millis();
}

// ... (loop, mock update, other helpers from V7) ...
