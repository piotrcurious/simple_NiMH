#include <Arduino.h>
#include <limits> // Required for numeric_limits

// Pin definitions (adjust based on your hardware)
#define PWM_PIN  5  

// --- Configuration & Constants ---
const unsigned long EXTERNAL_UPDATE_INTERVAL_MS = 500; // How often external task updates voltage/current
const float F = 96485.33212;   // Faraday's constant (C/mol)
const float R = 8.3145;        // Universal gas constant (J/(mol*K))
const float V_H2_EVOL_STD = -0.83; // Standard hydrogen evolution potential (V vs SHE) - NOTE: Often concentration/pH dependent
const float V_O2_EVOL_STD = 0.40;  // Standard oxygen evolution potential (V vs SHE) - NOTE: Often concentration/pH dependent
const float alpha = 0.5;       // Charge transfer coefficient (typical assumption)
const float n_H2 = 2.0;        // Number of electrons for hydrogen evolution
const float n_O2 = 4.0;        // Number of electrons for oxygen evolution
const float E_OVERPOT_THRESHOLD = 0.1; // Overpotential threshold defining electrolysis onset (V)

// --- Global Measured Battery Values ---
// Assumed to be updated by a separate system task/ISR every EXTERNAL_UPDATE_INTERVAL_MS
// Using volatile as they are modified asynchronously (best practice)
extern volatile float batteryVoltage;  // Battery voltage (V)
extern volatile float batteryCurrent;  // Battery current (A)

// --- Electrochemical Model Parameters (Modifiable via Calibration) ---
// Initial guesses - calibration will refine these
float A_NODE = 1.2e-4;         // Effective anode surface area (m^2)
float A_CATHODE = 1.5e-4;      // Effective cathode surface area (m^2)
float i0_H2 = 1e-6;            // Exchange current density for H2 evolution (A/m^2) - Highly sensitive
float i0_O2 = 2e-6;            // Exchange current density for O2 evolution (A/m^2) - Highly sensitive

// --- Calibration & Measurement State ---
enum class CalibrationState {
    IDLE,
    START_CALIBRATION,
    STEADY_STATE_PULSE,
    STEADY_STATE_SETTLE,
    START_DYNAMIC_MEAS,
    DYNAMIC_PULSE,
    DYNAMIC_SETTLE,
    CALCULATING,
    COMPLETE,
    ERROR
};
CalibrationState currentCalibrationState = CalibrationState::IDLE;

enum class DynamicMeasureState {
    IDLE,
    START_MEASUREMENT,
    BASELINE_WAIT,
    PULSE,
    SETTLE,
    RECORDING,
    COMPLETE,
    ERROR
};
DynamicMeasureState currentDynamicState = DynamicMeasureState::IDLE;


// Structure for storing dynamic response results
struct DynamicResponse {
  float timeConstantMs;    // Inferred time constant (ms)
  float maxVoltageStep;    // Max observed voltage step (V)
  float maxCurrentStep;    // Max observed current step (A)
  float steadyStateVoltage; // Voltage during longest pulse
  float steadyStateCurrent; // Current during longest pulse
  bool valid = false;      // Flag indicating if the measurement completed successfully
};

// --- Helper Function ---
// Non-blocking delay check
bool checkTimer(unsigned long &timerTimestamp, unsigned long interval) {
    return (millis() - timerTimestamp >= interval);
}

// Safe division helper
float safeDivide(float numerator, float denominator, float fallback = 0.0f) {
    if (fabs(denominator) < std::numeric_limits<float>::epsilon()) {
        return fallback; // Avoid division by zero or near-zero
    }
    return numerator / denominator;
}


//--------------------------------------------------------------
// computeMaxElectrolysisCurrent() - V2
//
// Predicts the maximum current before significant electrolysis begins,
// based on the *currently calibrated* model parameters and *current* conditions.
// Does NOT perform active measurements itself.
//--------------------------------------------------------------
float computeMaxElectrolysisCurrent(float currentIdleVoltage, float batteryTemperatureC, float internalResistance) {
    if (internalResistance <= 0) {
         Serial.println("Error: Invalid internal resistance in computeMaxElectrolysisCurrent.");
         return 0.0; // Or handle error appropriately
    }

    float T_kelvin = batteryTemperatureC + 273.15; // Convert to Kelvin

    // --- Calculate Nernst Potentials (Simplified - assumes standard conditions/activity=1) ---
    // More advanced: Adjust these based on concentration/pH if known
    float V_H2_eq = V_H2_EVOL_STD;
    float V_O2_eq = V_O2_EVOL_STD;

    // --- Calculate Threshold Voltages based on Overpotential ---
    // These represent the electrode potentials relative to their equilibrium where significant reaction starts
    // Note: Butler-Volmer uses overpotential (eta = V_electrode - V_equilibrium)
    float eta_H2_thresh = -E_OVERPOT_THRESHOLD; // Negative overpotential for reduction (H2 evolution)
    float eta_O2_thresh = E_OVERPOT_THRESHOLD;  // Positive overpotential for oxidation (O2 evolution)

    // Calculate current densities at the threshold overpotential using Butler-Volmer
    // Simplified form (assuming large overpotential dominates one exponential term) - more accurate full form below
    // float i_H2_at_thresh = i0_H2 * exp(- (1 - alpha) * n_H2 * F * eta_H2_thresh / (R * T_kelvin)); // Simplified cathodic
    // float i_O2_at_thresh = i0_O2 * exp(   alpha      * n_O2 * F * eta_O2_thresh / (R * T_kelvin)); // Simplified anodic

    // Using the full Butler-Volmer equation for better accuracy near equilibrium
     float i_H2_at_thresh = i0_H2 * (exp((alpha * n_H2 * F * eta_H2_thresh) / (R * T_kelvin)) - exp(-((1 - alpha) * n_H2 * F * eta_H2_thresh) / (R * T_kelvin)));
     float i_O2_at_thresh = i0_O2 * (exp((alpha * n_O2 * F * eta_O2_thresh) / (R * T_kelvin)) - exp(-((1 - alpha) * n_O2 * F * eta_O2_thresh) / (R * T_kelvin)));

    // Calculate total current contribution at the onset threshold
    // Current is positive by convention, take absolute value of H2 current density
    float I_H2_thresh = fabs(i_H2_at_thresh) * A_CATHODE;
    float I_O2_thresh = i_O2_at_thresh * A_NODE;

    // Total predicted electrolysis current *at the threshold overpotential*
    float I_electrolysis_thresh = I_H2_thresh + I_O2_thresh;

    // --- Consider Internal Resistance ---
    // The cell's terminal voltage needs to reach the sum of the equilibrium potential difference
    // plus the overpotentials plus the IR drop to sustain this current.
    // V_terminal = (V_O2_eq - V_H2_eq) + eta_O2_thresh + abs(eta_H2_thresh) + I_electrolysis_thresh * internalResistance;
    // This function predicts the current *when* electrolysis starts according to the model.
    // It doesn't necessarily predict the absolute max current the battery *can* source,
    // but the current level associated with the defined electrolysis onset.

    if (I_electrolysis_thresh < 0) { // Sanity check
         Serial.println("Warning: Predicted electrolysis threshold current is negative.");
         return 0.0;
    }

    return I_electrolysis_thresh;
}


//--------------------------------------------------------------
// measureDynamicResponse_Tick() - V2 (Non-blocking)
//
// Manages the state machine for probing the battery's transient response.
// Call this function repeatedly in the main loop.
// Returns true if measurement is in progress, false when idle or complete.
// Results are stored in the provided DynamicResponse struct reference.
//--------------------------------------------------------------
bool measureDynamicResponse_Tick(DynamicResponse &response) {

    const int NUM_PULSES = 5;
    const unsigned long PULSE_INCREMENT_MS = 100; // Increment pulse length
    const unsigned long SETTLE_TIME_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50; // Wait for external update + buffer
    const unsigned long BASELINE_STABLE_TIME_MS = 1000; // Time to wait for baseline stability

    static int currentPulseIndex;
    static unsigned long currentPulseDurationMs;
    static float baselineVoltage;
    static float baselineCurrent;
    static float voltageResponses[NUM_PULSES]; // Store V at end of each pulse
    static float currentResponses[NUM_PULSES]; // Store I at end of each pulse
    static unsigned long timerTimestamp;

    switch (currentDynamicState) {
        case DynamicMeasureState::IDLE:
            // Waiting to start
            break;

        case DynamicMeasureState::START_MEASUREMENT:
            Serial.println("Dynamic Measurement: Starting...");
            response.valid = false; // Invalidate previous results
            pinMode(PWM_PIN, OUTPUT);
            digitalWrite(PWM_PIN, LOW); // Ensure output is off initially
            currentPulseIndex = 0;
            timerTimestamp = millis();
            currentDynamicState = DynamicMeasureState::BASELINE_WAIT;
            break;

        case DynamicMeasureState::BASELINE_WAIT:
             // Wait for a stable baseline reading before starting pulses
            if (checkTimer(timerTimestamp, BASELINE_STABLE_TIME_MS)) {
                 // Read reliable baseline values after settling
                 noInterrupts(); // Briefly disable interrupts for atomic read
                 baselineVoltage = batteryVoltage;
                 baselineCurrent = batteryCurrent;
                 interrupts(); // Re-enable interrupts
                 Serial.print("Dynamic Measurement: Baseline V=");
                 Serial.print(baselineVoltage, 4);
                 Serial.print(" V, I=");
                 Serial.println(baselineCurrent, 4);
                 // Start the first pulse
                 currentPulseDurationMs = PULSE_INCREMENT_MS * (currentPulseIndex + 1);
                 digitalWrite(PWM_PIN, HIGH);
                 timerTimestamp = millis();
                 currentDynamicState = DynamicMeasureState::PULSE;
                 Serial.print("Dynamic Measurement: Pulse ");
                 Serial.print(currentPulseIndex + 1);
                 Serial.print(" (");
                 Serial.print(currentPulseDurationMs);
                 Serial.println(" ms)");
             }
             break;

        case DynamicMeasureState::PULSE:
            // Wait for the current pulse to finish
            if (checkTimer(timerTimestamp, currentPulseDurationMs)) {
                // Pulse finished, read values *immediately* before turning off
                 noInterrupts();
                 voltageResponses[currentPulseIndex] = batteryVoltage;
                 currentResponses[currentPulseIndex] = batteryCurrent;
                 interrupts();
                 // Turn off PWM
                 digitalWrite(PWM_PIN, LOW);
                 timerTimestamp = millis();
                 currentDynamicState = DynamicMeasureState::SETTLE;
            }
            break;

        case DynamicMeasureState::SETTLE:
            // Wait for the system to settle after the pulse
            if (checkTimer(timerTimestamp, SETTLE_TIME_MS)) {
                currentDynamicState = DynamicMeasureState::RECORDING;
            }
            break;

        case DynamicMeasureState::RECORDING:
             Serial.print("  -> V_end = "); Serial.print(voltageResponses[currentPulseIndex], 4);
             Serial.print(" V, I_end = "); Serial.println(currentResponses[currentPulseIndex], 4);

             currentPulseIndex++;
             if (currentPulseIndex < NUM_PULSES) {
                 // Start next pulse
                 currentPulseDurationMs = PULSE_INCREMENT_MS * (currentPulseIndex + 1);
                 digitalWrite(PWM_PIN, HIGH);
                 timerTimestamp = millis();
                 currentDynamicState = DynamicMeasureState::PULSE;
                 Serial.print("Dynamic Measurement: Pulse ");
                 Serial.print(currentPulseIndex + 1);
                 Serial.print(" (");
                 Serial.print(currentPulseDurationMs);
                 Serial.println(" ms)");
             } else {
                 // All pulses done, calculate results
                 float maxDeltaV = 0;
                 float maxDeltaI = 0;
                 int maxDeltaVIndex = -1;
                 // Calculate step changes relative to baseline
                 for (int i = 0; i < NUM_PULSES; i++) {
                     float deltaV = voltageResponses[i] - baselineVoltage;
                     float deltaI = currentResponses[i] - baselineCurrent;
                      if (fabs(deltaV) > fabs(maxDeltaV)) maxDeltaV = deltaV;
                      if (fabs(deltaI) > fabs(maxDeltaI)) maxDeltaI = deltaI;
                 }
                 response.maxVoltageStep = maxDeltaV;
                 response.maxCurrentStep = maxDeltaI;
                 response.steadyStateVoltage = voltageResponses[NUM_PULSES-1]; // Voltage during longest pulse
                 response.steadyStateCurrent = currentResponses[NUM_PULSES-1]; // Current during longest pulse

                 // Estimate time constant: Find pulse duration where response reaches ~63% of max *observed* step
                 // This is still an approximation but better than the fixed fraction of total time.
                 float targetDeltaV = maxDeltaV * 0.632; // Tau definition for first-order system
                 float targetDeltaI = maxDeltaI * 0.632;
                 response.timeConstantMs = -1.0; // Default if not found

                 for(int i = 0; i < NUM_PULSES; ++i) {
                      float stepV = voltageResponses[i] - baselineVoltage;
                      // Check if this step crosses the 63% threshold for V (prioritize V)
                      if ( (maxDeltaV > 0 && stepV >= targetDeltaV) || (maxDeltaV < 0 && stepV <= targetDeltaV) ) {
                          // Basic linear interpolation between this point and the previous one if available
                          if (i > 0) {
                               float prevStepV = voltageResponses[i-1] - baselineVoltage;
                               float fraction = safeDivide(targetDeltaV - prevStepV, stepV - prevStepV, 1.0);
                               response.timeConstantMs = (float)(PULSE_INCREMENT_MS * i) + fraction * PULSE_INCREMENT_MS;
                          } else {
                               response.timeConstantMs = (float)(PULSE_INCREMENT_MS * (i + 1)); // Use this pulse end time
                          }
                          break; // Found estimate based on Voltage
                      }
                 }
                 // Fallback: If V didn't yield a result (e.g., noisy V, stable I), try with Current
                 if(response.timeConstantMs < 0) {
                     for(int i = 0; i < NUM_PULSES; ++i) {
                          float stepI = currentResponses[i] - baselineCurrent;
                          if ( (maxDeltaI > 0 && stepI >= targetDeltaI) || (maxDeltaI < 0 && stepI <= targetDeltaI) ) {
                               if (i > 0) {
                                    float prevStepI = currentResponses[i-1] - baselineCurrent;
                                    float fraction = safeDivide(targetDeltaI - prevStepI, stepI - prevStepI, 1.0);
                                    response.timeConstantMs = (float)(PULSE_INCREMENT_MS * i) + fraction * PULSE_INCREMENT_MS;
                               } else {
                                    response.timeConstantMs = (float)(PULSE_INCREMENT_MS * (i + 1));
                               }
                              break; // Found estimate based on Current
                          }
                     }
                 }
                  // Last resort fallback: if no threshold crossed, use the longest pulse time * 0.63
                 if(response.timeConstantMs < 0) {
                    response.timeConstantMs = (float)(PULSE_INCREMENT_MS * NUM_PULSES) * 0.632;
                     Serial.println("Warning: Could not reliably determine time constant, using fallback estimate.");
                 }

                 Serial.println("Dynamic Measurement: Calculation Complete.");
                 Serial.print(" -> Max Delta V = "); Serial.println(response.maxVoltageStep, 4);
                 Serial.print(" -> Max Delta I = "); Serial.println(response.maxCurrentStep, 4);
                 Serial.print(" -> Estimated Time Constant = "); Serial.println(response.timeConstantMs, 2); Serial.println(" ms");
                 response.valid = true;
                 currentDynamicState = DynamicMeasureState::COMPLETE;
             }
             break;

        case DynamicMeasureState::COMPLETE:
            // Measurement finished, stay here until reset externally
            break;

        case DynamicMeasureState::ERROR:
            // An error occurred, stay here
            Serial.println("Dynamic Measurement: Error State.");
            break;
    }

    // Return true if measurement is ongoing
    return (currentDynamicState != DynamicMeasureState::IDLE &&
            currentDynamicState != DynamicMeasureState::COMPLETE &&
            currentDynamicState != DynamicMeasureState::ERROR);
}

// Function to start the dynamic measurement process
void startDynamicMeasurement() {
    if (currentDynamicState == DynamicMeasureState::IDLE || currentDynamicState == DynamicMeasureState::COMPLETE || currentDynamicState == DynamicMeasureState::ERROR) {
        currentDynamicState = DynamicMeasureState::START_MEASUREMENT;
    } else {
        Serial.println("Warning: Dynamic measurement already in progress.");
    }
}

// Function to check if dynamic measurement is complete and get results
bool getDynamicResponseResult(DynamicResponse &response) {
    if (currentDynamicState == DynamicMeasureState::COMPLETE) {
        // Optionally reset state here if you want it to be ready for another run immediately
        // currentDynamicState = DynamicMeasureState::IDLE;
        return true; // Results are ready and valid
    }
    return false; // Not finished or results not valid
}


//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V2 (Non-blocking)
//
// Manages the state machine for performing calibration.
// Incorporates dynamic response measurement.
// Call this function repeatedly in the main loop.
// Returns true if calibration is in progress, false when idle or complete.
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float internalResistance, float idleVoltage, float batteryTemperatureC) {

    const unsigned long STEADY_STATE_PULSE_MS = 2000; // Duration for steady-state calibration pulse
    const unsigned long STEADY_STATE_SETTLE_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50; // Settle time after pulse

    static unsigned long timerTimestamp;
    static float V_cal, I_cal; // Measured values during steady-state pulse
    static DynamicResponse dynamicResponseResult; // To store results from dynamic measurement

    switch (currentCalibrationState) {
        case CalibrationState::IDLE:
            // Waiting to start
            break;

        case CalibrationState::START_CALIBRATION:
            Serial.println("Calibration: Starting...");
            if (internalResistance <= 0) {
                 Serial.println("Calibration Error: Invalid internal resistance.");
                 currentCalibrationState = CalibrationState::ERROR;
                 break;
            }
             pinMode(PWM_PIN, OUTPUT);
             digitalWrite(PWM_PIN, HIGH); // Start steady-state pulse
             timerTimestamp = millis();
             currentCalibrationState = CalibrationState::STEADY_STATE_PULSE;
             Serial.print("Calibration: Steady-state pulse (");
             Serial.print(STEADY_STATE_PULSE_MS);
             Serial.println(" ms)");
             break;

        case CalibrationState::STEADY_STATE_PULSE:
            if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                // Read calibration values just before turning off pulse
                noInterrupts();
                V_cal = batteryVoltage;
                I_cal = batteryCurrent;
                interrupts();
                digitalWrite(PWM_PIN, LOW); // End pulse
                timerTimestamp = millis();
                currentCalibrationState = CalibrationState::STEADY_STATE_SETTLE;
                Serial.print(" -> V_cal = "); Serial.print(V_cal, 4);
                Serial.print(" V, I_cal = "); Serial.println(I_cal, 4);
                if (fabs(I_cal) < 1e-6) { // Check if calibration current is too low
                    Serial.println("Calibration Warning: Measured calibration current is near zero. Check connection/PWM.");
                    // Could potentially error out or try again here
                }
            }
            break;

        case CalibrationState::STEADY_STATE_SETTLE:
            if (checkTimer(timerTimestamp, STEADY_STATE_SETTLE_MS)) {
                 // Start the dynamic measurement phase
                 currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS;
            }
            break;

         case CalibrationState::START_DYNAMIC_MEAS:
             Serial.println("Calibration: Starting dynamic measurement phase...");
             startDynamicMeasurement(); // Initiate the dynamic response measurement
             currentCalibrationState = CalibrationState::DYNAMIC_MEASURE;
             break;

         case CalibrationState::DYNAMIC_MEASURE:
             // Run the dynamic measurement state machine
             measureDynamicResponse_Tick(dynamicResponseResult);
             // Check if it's finished
             if (getDynamicResponseResult(dynamicResponseResult)) {
                 if(dynamicResponseResult.valid) {
                     currentCalibrationState = CalibrationState::CALCULATING;
                 } else {
                     Serial.println("Calibration Error: Dynamic measurement failed.");
                     currentCalibrationState = CalibrationState::ERROR;
                 }
             } else if (currentDynamicState == DynamicMeasureState::ERROR) {
                 // Propagate error from dynamic measurement
                 Serial.println("Calibration Error: Dynamic measurement entered error state.");
                 currentCalibrationState = CalibrationState::ERROR;
             }
             // else: dynamic measurement still running
             break;


        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments...");
             float T_kelvin = batteryTemperatureC + 273.15;

             // --- Improved Steady-State Calibration ---
             // Calculate the effective voltage driving the electrochemical reactions,
             // accounting for the IR drop during the calibration pulse.
             float V_electrodes = V_cal - I_cal * internalResistance;

             // Estimate Nernst potentials (as before, simplified)
             float V_H2_eq = V_H2_EVOL_STD;
             float V_O2_eq = V_O2_EVOL_STD;
             float V_cell_eq = V_O2_eq - V_H2_eq; // Theoretical equilibrium cell voltage

             // Total overpotential available across both electrodes
             float eta_total = V_electrodes - V_cell_eq;

             if (eta_total < 0) {
                  Serial.println("Calibration Warning: Negative total overpotential during calibration pulse. Model may be inaccurate.");
                  eta_total = 0; // Prevent issues in calculations below
             }

             // How this total overpotential splits between anode and cathode is complex.
             // Simplification: Assume it splits proportionally based on some factor, or just evenly.
             // Let's assume even split for simplicity (can be refined later if needed)
             float eta_H2_cal = -eta_total / 2.0; // Cathodic overpotential
             float eta_O2_cal = eta_total / 2.0;  // Anodic overpotential

             // Calculate the *expected* current density from the model using *current* parameters and *calculated* overpotentials
             float i_H2_model = i0_H2 * (exp((alpha * n_H2 * F * eta_H2_cal) / (R * T_kelvin)) - exp(-((1 - alpha) * n_H2 * F * eta_H2_cal) / (R * T_kelvin)));
             float i_O2_model = i0_O2 * (exp((alpha * n_O2 * F * eta_O2_cal) / (R * T_kelvin)) - exp(-((1 - alpha) * n_O2 * F * eta_O2_cal) / (R * T_kelvin)));

             // Expected total model current for the *measured* V_cal, I_cal conditions
             float I_model = (fabs(i_H2_model) * A_CATHODE) + (i_O2_model * A_NODE);

             // Calculate correction factor
             float correctionFactor = safeDivide(I_cal, I_model, 1.0f); // Default to 1 if I_model is zero

             // Apply correction factor to effective areas (most direct parameter to adjust based on total current)
             // Add damping/limits to prevent wild swings
             const float MAX_CORRECTION = 2.0;
             const float MIN_CORRECTION = 0.5;
             if (correctionFactor > MAX_CORRECTION) correctionFactor = MAX_CORRECTION;
             if (correctionFactor < MIN_CORRECTION) correctionFactor = MIN_CORRECTION;

             A_CATHODE *= correctionFactor;
             A_NODE *= correctionFactor;

             // --- Use Dynamic Response Data (Informative for now) ---
             // The dynamic data (time constant, step response) gives insight into kinetics (i0) and double-layer capacitance,
             // or mass transport limitations. Directly mapping these to parameter adjustments without a more complex
             // model (e.g., including capacitance, diffusion) is non-trivial.
             // For now, we just log the results. A future step could involve:
             // - Correlating time constant to C_dl * R_ct (where R_ct relates to i0).
             // - Using step magnitude consistency across pulses to validate model.
             Serial.println("--- Calibration Results ---");
             Serial.print("Steady-State V_cal: "); Serial.println(V_cal, 4);
             Serial.print("Steady-State I_cal: "); Serial.println(I_cal, 4);
             Serial.print("Model Prediction I_model: "); Serial.println(I_model, 6);
             Serial.print("Correction Factor Applied: "); Serial.println(correctionFactor, 6);
             Serial.print("New A_CATHODE: "); Serial.println(A_CATHODE, 6);
             Serial.print("New A_NODE: "); Serial.println(A_NODE, 6);
             Serial.println("--- Dynamic Response Measured ---");
             Serial.print("Time Constant (ms): "); Serial.println(dynamicResponseResult.timeConstantMs, 2);
             Serial.print("Max Voltage Step (V): "); Serial.println(dynamicResponseResult.maxVoltageStep, 4);
             Serial.print("Max Current Step (A): "); Serial.println(dynamicResponseResult.maxCurrentStep, 4);
             Serial.print("Steady State V (Dynamic): "); Serial.println(dynamicResponseResult.steadyStateVoltage, 4);
             Serial.print("Steady State I (Dynamic): "); Serial.println(dynamicResponseResult.steadyStateCurrent, 4);
             Serial.println("---------------------------");

             currentCalibrationState = CalibrationState::COMPLETE;
             break;

        case CalibrationState::COMPLETE:
            // Calibration finished
            break;

        case CalibrationState::ERROR:
            // Error occurred, stay here
            Serial.println("Calibration: Error State.");
            break;
    }

    // Return true if calibrating
    return (currentCalibrationState != CalibrationState::IDLE &&
            currentCalibrationState != CalibrationState::COMPLETE &&
            currentCalibrationState != CalibrationState::ERROR);
}

// Function to start the calibration process
void startCalibration() {
    if (currentCalibrationState == CalibrationState::IDLE || currentCalibrationState == CalibrationState::COMPLETE || currentCalibrationState == CalibrationState::ERROR) {
         // Also ensure dynamic measurement is not running independently
         if (currentDynamicState == DynamicMeasureState::IDLE || currentDynamicState == DynamicMeasureState::COMPLETE || currentDynamicState == DynamicMeasureState::ERROR) {
            currentCalibrationState = CalibrationState::START_CALIBRATION;
         } else {
             Serial.println("Error: Cannot start calibration while dynamic measurement is active.");
         }
    } else {
        Serial.println("Warning: Calibration already in progress.");
    }
}

// Function to check if calibration is complete
bool isCalibrationComplete() {
    return currentCalibrationState == CalibrationState::COMPLETE;
}


//--------------------------------------------------------------
// Example setup and loop routines (NON-BLOCKING)
//--------------------------------------------------------------

// --- Mock external update variables (replace with your actual mechanism) ---
volatile float batteryVoltage = 3.7;
volatile float batteryCurrent = 0.0;
unsigned long lastExternalUpdate = 0;

void mockExternalUpdate() {
    // Simulate external task updating values periodically
    if (millis() - lastExternalUpdate >= EXTERNAL_UPDATE_INTERVAL_MS) {
        lastExternalUpdate = millis();
        // Simulate some change based on PWM state (very crude)
        // In reality, this update comes from your measurement task/ISR
        bool pwmState = digitalRead(PWM_PIN); // Check if PWM is supposed to be on
        if (pwmState && currentCalibrationState != CalibrationState::IDLE ) { // Example: If PWM on during calib
            batteryVoltage -= 0.05; // Voltage drop under load
            batteryCurrent += 0.1; // Current increases
        } else if (pwmState && currentDynamicState != DynamicMeasureState::IDLE) { // Example: If PWM on during dynamic meas
             // Simulate dynamic response - needs to be more sophisticated for realistic test
             batteryVoltage -= 0.02;
             batteryCurrent += 0.05;
        }
         else {
             // Simulate returning to idle
             if (batteryVoltage < 3.7) batteryVoltage += 0.01; else batteryVoltage = 3.7;
             if (batteryCurrent > 0.0) batteryCurrent -= 0.02; else batteryCurrent = 0.0;
        }
         // Add noise?
         // batteryVoltage += random(-10, 10) / 1000.0;
         // batteryCurrent += random(-5, 5) / 1000.0;
    }
}
// --- End Mock ---


void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial connection
    Serial.println("ESP32 Electrochemical Calibration V2 Started");

    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);

    // Initialize mock update timer
    lastExternalUpdate = millis();

    // (Other initialization code as needed)
}

void loop() {
    // --- Simulate external sensor updates ---
    mockExternalUpdate(); // Replace with your actual update mechanism trigger

    // --- Read current state safely ---
    // It's good practice to read volatile variables into local copies
    // if used multiple times, especially if interrupts are involved.
    noInterrupts(); // Briefly disable interrupts for atomic read
    float currentVoltage = batteryVoltage;
    float currentCurrent = batteryCurrent;
    interrupts(); // Re-enable interrupts

    // --- Manage State Machines ---
    // Static variables for example inputs - replace with real measurements/config
    static float internalResistance = 0.05;    // Example internal resistance (ohms) - Should ideally be measured/calibrated too!
    static float batteryTempC = 25.0;          // Example ambient temperature in °C - Use a sensor!
    static bool calibrationRequested = false;  // Flag to trigger calibration

    // Run the calibration state machine if active
    bool calibrating = calibrateBatteryChemistryModel_Tick(internalResistance, currentVoltage, batteryTempC);

    // Example: Trigger calibration once via Serial
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' && !calibrating) {
            calibrationRequested = true;
        }
    }

    if (calibrationRequested) {
        startCalibration();
        calibrationRequested = false; // Reset trigger
    }

    // --- Periodic Calculation Example ---
    static unsigned long lastPredictionTime = 0;
    unsigned long predictionInterval = 5000; // Calculate every 5 seconds

    // Only predict if NOT calibrating (to avoid using stale params during cal)
    if (!calibrating && checkTimer(lastPredictionTime, predictionInterval)) {
        lastPredictionTime = millis();

        // Compute the maximum electrolysis current based on the *current* model and measurements
        // Use the most recent 'idle' voltage reading (or approximate it)
        // For this example, we just use the latest reading, assuming it's close to idle if not pulsing.
        float I_max_pred = computeMaxElectrolysisCurrent(currentVoltage, batteryTempC, internalResistance);

        Serial.println("---------------------------");
        Serial.print(millis());
        Serial.print("ms - Current State: V=");
        Serial.print(currentVoltage, 4);
        Serial.print("V, I=");
        Serial.print(currentCurrent, 4);
        Serial.println("A");
        Serial.print("Predicted max electrolysis onset current: ");
        Serial.println(I_max_pred, 6);
        Serial.println("---------------------------");
    }

     // Add a small delay to prevent loop from running too fast if nothing else is happening
     // Avoids pegging the CPU unnecessarily. Adjust as needed.
     delay(10);
}
