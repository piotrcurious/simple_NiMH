#include <Arduino.h>
#include <limits>
#include <cmath>

// ... (Keep Pin definitions, Physical Constants, Default Potentials) ...
#define PWM_PIN  5
// ... F, R, alpha, n_H2, n_O2, E_OVERPOT_THRESHOLD, C_DL_SPECIFIC ...
const float V_H2_EVOL_STD_DEFAULT = -0.83; const float V_O2_EVOL_STD_DEFAULT = 0.40;
const float C_DL_SPECIFIC = 0.2;

// --- Calibratable Potentials & Measured R_internal/Tau ---
float V_H2_eff_std = V_H2_EVOL_STD_DEFAULT;
float V_O2_eff_std = V_O2_EVOL_STD_DEFAULT;
float R_internal_measured = 0.05; // Updated by GI analysis
float tau_decay_measured = 50.0; // Decay time constant (ms), updated by GI analysis
float tau_pulse_measured = 50.0; // Pulse rise time constant (ms), updated by dynamic analysis

// --- Global Measured Battery Values (Updated every 20ms externally) ---
extern volatile float batteryVoltage;
extern volatile float batteryCurrent;

// --- Model Parameters ---
float A_NODE = 1.2e-4; float A_CATHODE = 1.5e-4;
float i0_H2 = 1e-6; float i0_O2 = 2e-6;

// === NEW: Transient Capture Setup ===
const int MAX_TRANSIENT_SAMPLES = 100; // Max points to store for decay/pulse analysis. Adjust based on RAM!
const unsigned long TRANSIENT_SAMPLE_INTERVAL_US = 1000; // Target 1ms sampling interval. Check ADC speed!
const unsigned long GI_DECAY_CAPTURE_DURATION_MS = 100; // Capture decay for 100ms

struct TransientData {
    unsigned long time_us[MAX_TRANSIENT_SAMPLES];
    float voltage[MAX_TRANSIENT_SAMPLES];
    float current[MAX_TRANSIENT_SAMPLES];
    int sample_count = 0;
};

// Data storage for different phases
TransientData giDecayData;
TransientData dynamicPulseData; // Could potentially store data for each dynamic pulse length

// --- State Machines Enums ---
// Added GI_DECAY_CAPTURE, GI_DECAY_ANALYSIS
enum class CalibrationState { IDLE, START_CALIBRATION, STEADY_STATE_PULSE, GI_DECAY_CAPTURE, GI_DECAY_ANALYSIS, STEADY_STATE_SETTLE, START_DYNAMIC_MEAS, DYNAMIC_MEASURE, CALCULATING, COMPLETE, ERROR };
CalibrationState currentCalibrationState = CalibrationState::IDLE;
// Added DYNAMIC_PULSE_CAPTURE, DYNAMIC_ANALYSIS
enum class DynamicMeasureState { IDLE, START_MEASUREMENT, BASELINE_WAIT, DYNAMIC_PULSE_CAPTURE, DYNAMIC_ANALYSIS, SETTLE, /* RECORDING removed/merged */ COMPLETE, ERROR };
DynamicMeasureState currentDynamicState = DynamicMeasureState::IDLE;

// DynamicResponse Struct - Updated
struct DynamicResponse {
    // float timeConstantMs; // Replaced by tau_pulse / tau_decay
    float tau_pulse_ms = 0.0;  // From fitting pulse rise
    float tau_decay_ms = 0.0; // From fitting pulse fall (optional)
    float R_internal_pulse_ohm = 0.0; // From initial pulse step
    float maxVoltageStep = 0.0; // Keep for reference
    float maxCurrentStep = 0.0; // Keep for reference
    float steadyStateVoltage = 0.0;
    float steadyStateCurrent = 0.0;
    bool valid = false;
};

//--------------------------------------------------------------
// captureTransient() - NEW HELPER
//
// Performs high-frequency sampling for a short duration.
// NOTE: Assumes analogRead() is fast enough for sample_interval_us.
//       Uses micros() for timing. Blocking during capture.
// Returns: Number of samples actually captured.
//--------------------------------------------------------------
int captureTransient(unsigned long duration_ms, unsigned long sample_interval_us, TransientData &data) {
    data.sample_count = 0;
    unsigned long start_time_us = micros();
    unsigned long end_time_us = start_time_us + duration_ms * 1000UL;
    unsigned long next_sample_time_us = start_time_us;

    // Potential: Configure ADC for faster reads if needed/possible before loop

    while (micros() < end_time_us && data.sample_count < MAX_TRANSIENT_SAMPLES) {
        if (micros() >= next_sample_time_us) {
            // Perform ADC reads - use direct global read for simplicity now
            // For higher accuracy, might need multiple reads and averaging
            noInterrupts(); // Minimize interruption during reads if globals are ISR updated
            data.voltage[data.sample_count] = batteryVoltage;
            data.current[data.sample_count] = batteryCurrent;
            interrupts();
            data.time_us[data.sample_count] = micros() - start_time_us; // Relative time

            data.sample_count++;
            next_sample_time_us += sample_interval_us;

            // Small delay to prevent overwhelming CPU if interval is very short
            // and allow other tasks (like external updates if cooperative)
            // delayMicroseconds(sample_interval_us / 4); // Adjust as needed
        }
        // Yield can be important in ESP32 FreeRTOS environment if loop is tight
        // yield(); // Or vTaskDelay(1); if using FreeRTOS tasks
    }

    // Potential: Restore ADC configuration if changed

    return data.sample_count;
}

//--------------------------------------------------------------
// linearRegression() - NEW HELPER
//
// Performs simple linear regression y = slope * x + intercept.
// Returns true on success, false if insufficient data.
//--------------------------------------------------------------
bool linearRegression(const float x[], const float y[], int n, float &slope, float &intercept) {
    if (n < 2) {
        return false; // Need at least 2 points
    }

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < n; ++i) {
        sumX += x[i];
        sumY += y[i];
        sumXY += (double)x[i] * y[i]; // Use double for intermediate sums
        sumX2 += (double)x[i] * x[i];
    }

    double denom = (double)n * sumX2 - sumX * sumX;
    if (fabs(denom) < 1e-9) { // Avoid division by zero (vertical line)
        return false;
    }

    slope = (n * sumXY - sumX * sumY) / denom;
    intercept = (sumY - slope * sumX) / n;
    return true;
}

//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V8 (GI Decay Analysis)
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float batteryTemperatureC) {
    // ... (Constants STEADY_STATE_PULSE_MS etc. as before) ...
    const unsigned long GI_MEASURE_DELAY_US = 50; // Still useful for initial drop timing if possible
    const unsigned long STEADY_STATE_SETTLE_MS = 50; // Shorter settle after decay capture

    static unsigned long timerTimestamp;
    static float V_before_off, I_cal;
    static DynamicResponse dynamicResponseResult; // Now reused in main calibration too

    switch (currentCalibrationState) {
        case CalibrationState::IDLE: break;
        case CalibrationState::START_CALIBRATION: /* ... (Same) ... */ break;

        case CalibrationState::STEADY_STATE_PULSE:
             if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                 // Read just before turning off
                 // Critical Timing: Ideally read V, turn off, read V again very fast.
                 // Reading globals might be too slow. This is an approximation.
                 noInterrupts(); V_before_off = batteryVoltage; I_cal = batteryCurrent; interrupts();
                 digitalWrite(PWM_PIN, LOW); // --- CURRENT INTERRUPTED ---

                 Serial.print(" -> Pulse End: V_before_off = "); Serial.print(V_before_off, 4);
                 Serial.print(" V, I_cal = "); Serial.println(I_cal, 4);
                 if (fabs(I_cal) < 1e-6) { Serial.println("  Warning: I_cal near zero."); }

                 // Immediately start capturing decay
                 currentCalibrationState = CalibrationState::GI_DECAY_CAPTURE;
             }
             break;

         case CalibrationState::GI_DECAY_CAPTURE: // NEW STATE
             Serial.println(" -> GI Decay Capture...");
             captureTransient(GI_DECAY_CAPTURE_DURATION_MS, TRANSIENT_SAMPLE_INTERVAL_US, giDecayData);
             Serial.print("    Captured "); Serial.print(giDecayData.sample_count); Serial.println(" decay points.");
             currentCalibrationState = CalibrationState::GI_DECAY_ANALYSIS;
             break;

         case CalibrationState::GI_DECAY_ANALYSIS: // NEW STATE
             Serial.println(" -> GI Decay Analysis...");
             if (giDecayData.sample_count < 5) { // Need sufficient points
                 Serial.println("  Error: Insufficient decay points captured.");
                 // Keep old R_internal, tau_decay values? Or error out?
                 currentCalibrationState = CalibrationState::ERROR; // Error out for now
                 break;
             }

             // --- R_internal Calculation ---
             // Use V_before_off and the *first* point captured after interruption
             float V_after_off = giDecayData.voltage[0];
             float V_drop = V_before_off - V_after_off;
             if (fabs(I_cal) > 1e-6) {
                 if(V_drop < 0) { Serial.println("  Warning: V_drop negative."); }
                 else {
                     float R_internal_new = safeDivide(V_drop, I_cal, R_internal_measured);
                     /* ... Apply Limits (MIN/MAX_R_INTERNAL) ... */
                      const float MIN_R_INTERNAL = 0.001; const float MAX_R_INTERNAL = 1.0;
                      if (R_internal_new < MIN_R_INTERNAL) R_internal_new = MIN_R_INTERNAL; if (R_internal_new > MAX_R_INTERNAL) R_internal_new = MAX_R_INTERNAL;
                     R_internal_measured = R_internal_new; // Direct update
                     Serial.print("  -> GI Measured R_internal: "); Serial.print(R_internal_measured, 4); Serial.println(" Ohms");
                 }
             } else { Serial.println("  Skipping R_internal update: I_cal was near zero."); }

             // --- Tau Decay Calculation ---
             // Estimate V_ocv_local from the last few points of decay
             float V_ocv_local_est = 0;
             int num_tail_points = min(5, giDecayData.sample_count);
             for (int i = giDecayData.sample_count - num_tail_points; i < giDecayData.sample_count; ++i) {
                 V_ocv_local_est += giDecayData.voltage[i];
             }
             V_ocv_local_est /= num_tail_points;
             Serial.print("  Estimated V_ocv_local (decay end): "); Serial.println(V_ocv_local_est, 4);

             // Prepare data for linear regression: x = time (sec), y = ln(V(t) - V_ocv_local)
             float time_sec[MAX_TRANSIENT_SAMPLES];
             float ln_delta_V[MAX_TRANSIENT_SAMPLES];
             int fit_points_count = 0;
             for (int i = 0; i < giDecayData.sample_count; ++i) {
                 float delta_V = giDecayData.voltage[i] - V_ocv_local_est;
                 // Only use points where V > V_ocv_local and avoid initial ohmic drop points?
                 // Start fitting after a few points (e.g., after ~5% of capture time?)
                 unsigned long time_us_point = giDecayData.time_us[i];
                 if (delta_V > 1e-4 && time_us_point > (GI_DECAY_CAPTURE_DURATION_MS * 1000UL * 0.05) ) { // Check delta_V > 0 and skip initial part
                     time_sec[fit_points_count] = (float)time_us_point / 1000000.0f;
                     ln_delta_V[fit_points_count] = log(delta_V);
                     fit_points_count++;
                 }
             }

             float slope, intercept;
             if (fit_points_count >= 5 && linearRegression(time_sec, ln_delta_V, fit_points_count, slope, intercept)) {
                 if (slope < -1e-3) { // Check for negative slope
                     float tau_decay_sec = -1.0 / slope;
                     tau_decay_measured = tau_decay_sec * 1000.0; // Convert to ms
                     // Apply sanity limits? e.g. 1ms to 5000ms
                     if(tau_decay_measured < 1.0) tau_decay_measured = 1.0;
                     if(tau_decay_measured > 5000.0) tau_decay_measured = 5000.0;
                     Serial.print("  -> GI Fit tau_decay: "); Serial.print(tau_decay_measured, 2); Serial.println(" ms");
                 } else { Serial.println("  Warning: Decay fit slope not negative or too small."); }
             } else { Serial.println("  Warning: Decay linear regression failed (insufficient points or poor fit)."); }

             // Proceed to settling state
             timerTimestamp = millis(); // Reset timer for short settle period
             currentCalibrationState = CalibrationState::STEADY_STATE_SETTLE;
             break;

        case CalibrationState::STEADY_STATE_SETTLE:
             // Short settle after analysis before dynamic pulses
             if (checkTimer(timerTimestamp, STEADY_STATE_SETTLE_MS)) {
                 currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS;
             }
             break;

        // --- Dynamic Measurement Trigger ---
        case CalibrationState::START_DYNAMIC_MEAS:
             Serial.println("Calibration: Starting dynamic measurement phase...");
             startDynamicMeasurement(); // Initiate the dynamic response measurement
             currentCalibrationState = CalibrationState::DYNAMIC_MEASURE;
             break;

        case CalibrationState::DYNAMIC_MEASURE:
             // Run the dynamic measurement state machine
             // It now stores results (tau_pulse, R_internal_pulse) in dynamicResponseResult
             measureDynamicResponse_Tick(dynamicResponseResult);
             // Check if it's finished
             if (getDynamicResponseResult(dynamicResponseResult)) {
                 if(dynamicResponseResult.valid) {
                     // --- Combine Tau Results? ---
                     // Decide which tau to use for i0 calibration. Decay might be better.
                     // tau_for_i0_adjust = tau_decay_measured; // Prioritize decay tau
                     // Or average? tau_for_i0_adjust = (tau_decay_measured + dynamicResponseResult.tau_pulse_ms)/2.0;
                     Serial.print("  Dynamic measurement complete. Using tau_decay = ");
                     Serial.print(tau_decay_measured, 2);
                     Serial.print(" ms, tau_pulse = ");
                     Serial.print(dynamicResponseResult.tau_pulse_ms, 2);
                     Serial.print(" ms, R_int_pulse = ");
                     Serial.print(dynamicResponseResult.R_internal_pulse_ohm, 4);
                     Serial.println(" Ohms for calibration.");
                     // Update global tau_pulse_measured if needed
                     tau_pulse_measured = dynamicResponseResult.tau_pulse_ms;

                     currentCalibrationState = CalibrationState::CALCULATING;
                 } else { /* ... error handling ... */ }
             } else if (currentDynamicState == DynamicMeasureState::ERROR) { /* ... error handling ... */ }
             break;


        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments (V8)...");
             Serial.print("  Using R_internal (GI) = "); Serial.println(R_internal_measured, 4);
             Serial.print("  Using V_H2_eff_std = "); /* ... */ Serial.print(V_H2_eff_std, 4);
             Serial.print(", V_O2_eff_std = "); /* ... */ Serial.println(V_O2_eff_std, 4);
             Serial.print("  Using tau_decay = "); Serial.println(tau_decay_measured, 2); // Indicate which tau is used

             float T_kelvin = batteryTemperatureC + 273.15;
             /* ... Store start params A, i0 ... */

             // === Part 0: Adjust i0 Individually (using tau_decay_measured) ===
             Serial.println("--- Step 0: Adjusting i0 Individually (Dynamic Response) ---");
             float correctionFactor_i0_H2 = 1.0; float correctionFactor_i0_O2 = 1.0;
             float tau_to_use = tau_decay_measured; // *** Use decay tau ***

             if (tau_to_use <= 0) { /* Skip */ }
             else if (A_CATHODE <= 0 || A_NODE <= 0 || i0_H2 <= 0 || i0_O2 <= 0) { /* Skip */ }
             else { /* ... Calculate C_dl ... */
                 float C_dl_total_estimated = C_DL_SPECIFIC * (A_NODE + A_CATHODE);
                 if (C_dl_total_estimated <= 0) { /* Skip */ }
                 else { /* ... Calculate R_ct_estimated_sum from tau_to_use ... */
                     float R_ct_estimated_sum = (tau_to_use / 1000.0) / C_dl_total_estimated;
                     /* ... Calculate R_ct_H2/O2/Total_theory ... */
                     /* ... Calculate delta_R, denom_R_ct_sq_sum ... */
                     /* ... Calculate correctionFactor_i0_H2/O2 ... */
                     /* ... Apply Limits ... */
                     /* ... Logging ... */
                 }
             }
             /* ... Apply i0 corrections, MIN_I0_VALUE, logging ... */

             // === Part 1: Find Overpotential Split (using updated i0 & R_internal_measured) ===
             Serial.println("--- Step 1: Finding Overpotential Split ---");
             /* ... Use V_eff_std, R_internal_measured ... */
             /* ... Bisection search logic identical to V7 ... */

             // === Part 2: Adjust Area (using split found with updated i0) ===
             Serial.println("--- Step 2: Adjusting Area (Steady State) ---");
             /* ... Identical logic to V7 ... */

             // --- Final Output ---
             Serial.println("--- Calibration V8 Complete ---");
             /* ... Print final A, i0, R_int, V_eff parameters ... */
             /* ... Also print tau_decay and tau_pulse ? ... */
              Serial.println("Final Parameters:");
              Serial.print(" A_CATHODE: "); Serial.println(A_CATHODE, 6); Serial.print(" A_NODE: "); Serial.println(A_NODE, 6);
              Serial.print(" i0_H2: "); Serial.println(i0_H2, 6); Serial.print(" i0_O2: "); Serial.println(i0_O2, 6);
              Serial.print(" R_internal: "); Serial.println(R_internal_measured, 4);
              Serial.print(" tau_decay: "); Serial.print(tau_decay_measured, 2); Serial.print("ms, tau_pulse: "); Serial.println(tau_pulse_measured, 2);
              Serial.print(" V_H2_eff: "); Serial.print(V_H2_eff_std, 4); Serial.print(", V_O2_eff: "); Serial.println(V_O2_eff_std, 4);
              Serial.println("-----------------------------");

             currentCalibrationState = CalibrationState::COMPLETE;
             break;

        case CalibrationState::COMPLETE: break;
        case CalibrationState::ERROR: Serial.println("Calibration: Error State."); break;
    }
    return (currentCalibrationState != CalibrationState::IDLE && /* ... */);
}

//--------------------------------------------------------------
// measureDynamicResponse_Tick() - V4 (Pulse Shape Capture/Analysis)
//--------------------------------------------------------------
bool measureDynamicResponse_Tick(DynamicResponse &response) {

    const int NUM_PULSES = 5; // Keep multiple pulses for now? Or just one long one? Let's keep 5.
    const unsigned long PULSE_INCREMENT_MS = 100; // Increment pulse length
    const unsigned long SETTLE_TIME_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50;
    const unsigned long BASELINE_STABLE_TIME_MS = 1000;

    static int currentPulseIndex;
    static unsigned long currentPulseDurationMs;
    static float baselineVoltage;
    static float baselineCurrent;
    // static float voltageResponses[NUM_PULSES]; // No longer just end points
    // static float currentResponses[NUM_PULSES]; // No longer just end points
    static unsigned long timerTimestamp;
    static TransientData currentPulseTransientData; // Store data for the current pulse

    switch (currentDynamicState) {
        case DynamicMeasureState::IDLE: break;

        case DynamicMeasureState::START_MEASUREMENT:
            Serial.println("Dynamic Measurement: Starting (V4 - Pulse Shape)...");
            response.valid = false; pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, LOW);
            currentPulseIndex = 0; timerTimestamp = millis();
            currentDynamicState = DynamicMeasureState::BASELINE_WAIT;
            break;

        case DynamicMeasureState::BASELINE_WAIT:
            if (checkTimer(timerTimestamp, BASELINE_STABLE_TIME_MS)) {
                 noInterrupts(); baselineVoltage = batteryVoltage; baselineCurrent = batteryCurrent; interrupts();
                 Serial.print("Dynamic Measurement: Baseline V="); Serial.print(baselineVoltage, 4); /* ... */
                 currentDynamicState = DynamicMeasureState::DYNAMIC_PULSE_CAPTURE; // Go direct to capture
                 // Prepare for first pulse capture
                 currentPulseDurationMs = PULSE_INCREMENT_MS * (currentPulseIndex + 1);
                 Serial.print("Dynamic Measurement: Pulse "); Serial.print(currentPulseIndex + 1); /* ... */ Serial.println(" ms) - Capturing...");
                 digitalWrite(PWM_PIN, HIGH); // Start PWM *before* capture
                 // Allow brief moment for PWM hardware to start? delayMicroseconds(10);
            }
            break;

        case DynamicMeasureState::DYNAMIC_PULSE_CAPTURE: // NEW STATE (replaces PULSE)
             // Capture the transient during the pulse
             captureTransient(currentPulseDurationMs, TRANSIENT_SAMPLE_INTERVAL_US, currentPulseTransientData); // This is blocking for duration_ms
             digitalWrite(PWM_PIN, LOW); // Turn off PWM after capture completes
             Serial.print("    Captured "); Serial.print(currentPulseTransientData.sample_count); Serial.println(" pulse points.");
             currentDynamicState = DynamicMeasureState::DYNAMIC_ANALYSIS; // Go to analysis
             break;

        case DynamicMeasureState::DYNAMIC_ANALYSIS: // NEW STATE (replaces RECORDING)
             Serial.println("  Analyzing Pulse Shape...");
             if (currentPulseTransientData.sample_count < 5) {
                 Serial.println("  Error: Insufficient pulse points captured.");
                 currentDynamicState = DynamicMeasureState::ERROR; break;
             }

             // --- R_internal Estimation (from initial step) ---
             // Find initial current step (first few points relative to baseline)
             // This is tricky if baselineCurrent is not precisely zero or stable.
             // Alternative: V step at start. V_step = V[1] - baselineVoltage. I_step = I[1] - baselineCurrent; R = V_step / I_step.
             // Requires very fast sampling to catch the step before C_dl starts charging significantly.
             // Let's try using first two points if sample interval is fast enough.
             float V0 = currentPulseTransientData.voltage[0]; float I0 = currentPulseTransientData.current[0];
             float V1 = currentPulseTransientData.voltage[1]; float I1 = currentPulseTransientData.current[1];
             // If baseline current was non-zero, I_step is relative to baseline
             float I_step = I1 - baselineCurrent; // Approximation
             float V_step = V1 - baselineVoltage; // Approximation

             if (fabs(I_step) > 1e-3) { // Avoid division by zero
                 float R_int_pulse = safeDivide(V_step, I_step, -1.0);
                 if (R_int_pulse > 0) {
                      Serial.print("    Pulse R_internal estimate: "); Serial.println(R_int_pulse, 4);
                      response.R_internal_pulse_ohm = R_int_pulse; // Store it
                 } else { Serial.println("    Pulse R_internal estimate failed (V/I step issue)."); }
             }

             // --- Tau Pulse Estimation (from rise) ---
             // Estimate V_final from last few points
             float V_final_est = 0;
             int num_tail_points = min(5, currentPulseTransientData.sample_count);
             for (int i = currentPulseTransientData.sample_count - num_tail_points; i < currentPulseTransientData.sample_count; ++i) {
                 V_final_est += currentPulseTransientData.voltage[i];
             }
             V_final_est /= num_tail_points;
             Serial.print("    Estimated V_final (pulse end): "); Serial.println(V_final_est, 4);

             // Prepare data: x = time (sec), y = ln(V_final - V(t))
             float time_sec[MAX_TRANSIENT_SAMPLES];
             float ln_delta_V[MAX_TRANSIENT_SAMPLES];
             int fit_points_count = 0;
             // Start fit after initial jump (~5%?) and end before last few points used for V_final_est
             int fit_end_index = currentPulseTransientData.sample_count - num_tail_points - 1;
             for (int i = 0; i < fit_end_index; ++i) {
                 float delta_V = V_final_est - currentPulseTransientData.voltage[i];
                 unsigned long time_us_point = currentPulseTransientData.time_us[i];
                 // Use points after initial jump (~5% duration?)
                 if (delta_V > 1e-4 && time_us_point > (currentPulseDurationMs * 1000UL * 0.05)) {
                     time_sec[fit_points_count] = (float)time_us_point / 1000000.0f;
                     ln_delta_V[fit_points_count] = log(delta_V);
                     fit_points_count++;
                 }
             }

             float slope, intercept;
             if (fit_points_count >= 5 && linearRegression(time_sec, ln_delta_V, fit_points_count, slope, intercept)) {
                 if (slope < -1e-3) {
                     float tau_pulse_sec = -1.0 / slope;
                     response.tau_pulse_ms = tau_pulse_sec * 1000.0;
                     /* ... Apply Limits ... */
                     if(response.tau_pulse_ms < 1.0) response.tau_pulse_ms = 1.0; if(response.tau_pulse_ms > 5000.0) response.tau_pulse_ms = 5000.0;
                     Serial.print("    -> Pulse Fit tau_pulse: "); Serial.print(response.tau_pulse_ms, 2); Serial.println(" ms");
                 } else { Serial.println("    Warning: Pulse fit slope not negative or too small."); }
             } else { Serial.println("    Warning: Pulse linear regression failed."); }

             // Store other results for reference (use last point as steady state for this pulse)
             response.steadyStateVoltage = currentPulseTransientData.voltage[currentPulseTransientData.sample_count - 1];
             response.steadyStateCurrent = currentPulseTransientData.current[currentPulseTransientData.sample_count - 1];
             response.maxVoltageStep = response.steadyStateVoltage - baselineVoltage;
             response.maxCurrentStep = response.steadyStateCurrent - baselineCurrent;

             // Move to settle state
             timerTimestamp = millis();
             currentDynamicState = DynamicMeasureState::SETTLE;
             break;

        case DynamicMeasureState::SETTLE:
            if (checkTimer(timerTimestamp, SETTLE_TIME_MS)) {
                // Finished settling for this pulse
                currentPulseIndex++;
                if (currentPulseIndex < NUM_PULSES) {
                    // Prepare for next pulse capture
                    currentPulseDurationMs = PULSE_INCREMENT_MS * (currentPulseIndex + 1);
                    Serial.print("Dynamic Measurement: Pulse "); Serial.print(currentPulseIndex + 1); /* ... */ Serial.println(" ms) - Capturing...");
                    digitalWrite(PWM_PIN, HIGH); // Start PWM
                    // delayMicroseconds(10); // Small delay?
                    currentDynamicState = DynamicMeasureState::DYNAMIC_PULSE_CAPTURE;
                } else {
                    // All pulses done
                    Serial.println("Dynamic Measurement: All pulses complete.");
                    response.valid = true; // Mark results as valid
                    currentDynamicState = DynamicMeasureState::COMPLETE;
                }
            }
            break;

        case DynamicMeasureState::COMPLETE: break;
        case DynamicMeasureState::ERROR: Serial.println("Dynamic Measurement: Error State."); break;
    }

    return (currentDynamicState != DynamicMeasureState::IDLE && /* ... */);
}

// --- Main loop (`setup`, `loop`, `mockExternalUpdate`, OCV logic - Keep as in V7) ---
// Ensure OCV calibration uses calibrated potentials correctly.
// Ensure loop() passes only temperature to main calibration tick.
// Ensure computeMax... uses globals correctly.

void setup() {
    Serial.begin(115200); while (!Serial);
    Serial.println("ESP32 Electrochemical Calibration V8 Started");
    Serial.print("Assumed C_DL_SPECIFIC: "); Serial.print(C_DL_SPECIFIC); Serial.println(" F/m^2");
    Serial.print("Initial R_internal guess: "); Serial.println(R_internal_measured, 4);
    Serial.print("Initial V_H2_eff_std: "); Serial.print(V_H2_eff_std, 4); Serial.print(", V_O2_eff_std: "); Serial.println(V_O2_eff_std, 4);
    // ... (rest of setup) ...
}

void loop() {
     // ... (Identical to V7 loop logic: mock update, read globals, handle activity/OCV calib trigger) ...
     // ... (Handle main calibration trigger 'c', OCV trigger 'o') ...

     // Run the main calibration state machine if active
     static float batteryTempC = 25.0; // Use sensor!
     bool calibrating = calibrateBatteryChemistryModel_Tick(batteryTempC);

     // ... (Handle calibrationRequested trigger) ...

     // --- Periodic Calculation ---
     static unsigned long lastPredictionTime = 0;
     unsigned long predictionInterval = 5000;
     if (!calibrating && !ocvCalibrationPending && checkTimer(lastPredictionTime, predictionInterval)) {
          // ... (Identical to V7 prediction logic: use currentVoltage, batteryTempC, call computeMax...) ...
          // ... (Serial print V, I, Params, Predicted I_max) ...
     }

     delay(10);
}


// --- Include ALL required helper function implementations ---
// bool checkTimer(...) { ... }
// float safeDivide(...) { ... }
// float calculate_current_density(...) { ... }
// void calibrateEquilibriumPotentials(...) { ... } // From V7
// void startCalibration() { ... }
// bool isCalibrationComplete() { ... }
// void startDynamicMeasurement() { ... }
// bool getDynamicResponseResult(...) { ... }
// // bool measureDynamicResponse_Tick(...) { /* Implemented above */ }
// // bool calibrateBatteryChemistryModel_Tick(...) { /* Implemented above */ }
// float computeMaxElectrolysisCurrent(...) { /* As modified in V7 */ }
// // int captureTransient(...) { /* Implemented above */ }
// // bool linearRegression(...) { /* Implemented above */ }

