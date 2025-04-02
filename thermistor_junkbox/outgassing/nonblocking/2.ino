#include <Arduino.h>
#include <limits> // Required for numeric_limits
#include <cmath>  // Required for fabs

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

// *** NEW: Assumed Specific Double Layer Capacitance ***
// Value is highly dependent on electrode material, electrolyte, and surface roughness.
// Typical values range from 10-40 uF/cm^2 for smooth metals. Porous materials are much higher.
// 20 uF/cm^2 = 0.2 F/m^2. Adjust based on your system or further calibration.
const float C_DL_SPECIFIC = 0.2; // Specific capacitance (F/m^2)

// --- Global Measured Battery Values ---
extern volatile float batteryVoltage;  // Battery voltage (V)
extern volatile float batteryCurrent;  // Battery current (A)

// --- Electrochemical Model Parameters (Modifiable via Calibration) ---
float A_NODE = 1.2e-4;         // Effective anode surface area (m^2)
float A_CATHODE = 1.5e-4;      // Effective cathode surface area (m^2)
float i0_H2 = 1e-6;            // Exchange current density for H2 evolution (A/m^2)
float i0_O2 = 2e-6;            // Exchange current density for O2 evolution (A/m^2)

// --- Calibration & Measurement State --- (Keep Enums as before)
enum class CalibrationState { IDLE, START_CALIBRATION, STEADY_STATE_PULSE, STEADY_STATE_SETTLE, START_DYNAMIC_MEAS, DYNAMIC_MEASURE, CALCULATING, COMPLETE, ERROR };
CalibrationState currentCalibrationState = CalibrationState::IDLE;
enum class DynamicMeasureState { IDLE, START_MEASUREMENT, BASELINE_WAIT, PULSE, SETTLE, RECORDING, COMPLETE, ERROR };
DynamicMeasureState currentDynamicState = DynamicMeasureState::IDLE;

// DynamicResponse Struct (Keep as before)
struct DynamicResponse {
  float timeConstantMs;
  float maxVoltageStep;
  float maxCurrentStep;
  float steadyStateVoltage;
  float steadyStateCurrent;
  bool valid = false;
};

// --- Helper Functions (Keep as before) ---
bool checkTimer(unsigned long &timerTimestamp, unsigned long interval) { /* ... */ }
float safeDivide(float numerator, float denominator, float fallback = 0.0f) { /* ... */ }

// --- computeMaxElectrolysisCurrent() (Keep as before) ---
float computeMaxElectrolysisCurrent(float currentIdleVoltage, float batteryTemperatureC, float internalResistance) { /* ... */ }

// --- measureDynamicResponse_Tick() and related helpers (Keep as before) ---
bool measureDynamicResponse_Tick(DynamicResponse &response) { /* ... */ }
void startDynamicMeasurement() { /* ... */ }
bool getDynamicResponseResult(DynamicResponse &response) { /* ... */ }


//--------------------------------------------------------------
// calibrateBatteryChemistryModel_Tick() - V3 (Includes Dynamic Data Use)
//
// Manages the state machine for performing calibration.
// Adjusts Area based on steady-state.
// Adjusts i0 based on dynamic time constant.
//--------------------------------------------------------------
bool calibrateBatteryChemistryModel_Tick(float internalResistance, float idleVoltage, float batteryTemperatureC) {

    const unsigned long STEADY_STATE_PULSE_MS = 2000;
    const unsigned long STEADY_STATE_SETTLE_MS = EXTERNAL_UPDATE_INTERVAL_MS + 50;

    static unsigned long timerTimestamp;
    static float V_cal, I_cal;
    static DynamicResponse dynamicResponseResult;

    // --- State Machine Logic (States IDLE to DYNAMIC_MEASURE remain the same) ---
    switch (currentCalibrationState) {
        case CalibrationState::IDLE:
            // ... (same as V2)
            break;

        case CalibrationState::START_CALIBRATION:
            // ... (same as V2)
            Serial.println("Calibration: Starting (V3 - Dynamic i0 Adjust)...");
             if (internalResistance <= 0) { /* ... error handling ... */ }
             // ... start pulse ...
             break;

        case CalibrationState::STEADY_STATE_PULSE:
             // ... (same as V2 - wait for pulse, read V_cal, I_cal, stop pulse) ...
             if (checkTimer(timerTimestamp, STEADY_STATE_PULSE_MS)) {
                  // ... read V_cal, I_cal ...
                  digitalWrite(PWM_PIN, LOW);
                  timerTimestamp = millis();
                  currentCalibrationState = CalibrationState::STEADY_STATE_SETTLE;
                  // ... print V_cal, I_cal ...
             }
             break;

        case CalibrationState::STEADY_STATE_SETTLE:
             // ... (same as V2 - wait for settle) ...
              if (checkTimer(timerTimestamp, STEADY_STATE_SETTLE_MS)) {
                 currentCalibrationState = CalibrationState::START_DYNAMIC_MEAS;
            }
             break;

         case CalibrationState::START_DYNAMIC_MEAS:
              // ... (same as V2 - start dynamic measurement) ...
              startDynamicMeasurement();
              currentCalibrationState = CalibrationState::DYNAMIC_MEASURE;
             break;

         case CalibrationState::DYNAMIC_MEASURE:
             // ... (same as V2 - run dynamic measurement state machine) ...
              measureDynamicResponse_Tick(dynamicResponseResult);
             if (getDynamicResponseResult(dynamicResponseResult)) {
                 if(dynamicResponseResult.valid) {
                     currentCalibrationState = CalibrationState::CALCULATING;
                 } else { /* ... error handling ... */ }
             } else if (currentDynamicState == DynamicMeasureState::ERROR) { /* ... error handling ... */ }
             break;


        case CalibrationState::CALCULATING:
             Serial.println("Calibration: Calculating parameter adjustments (V3)...");
             float T_kelvin = batteryTemperatureC + 273.15;

             // === Part 1: Steady-State Calibration (Adjust Area) ===
             Serial.println("--- Step 1: Adjusting Area (Steady State) ---");
             float V_electrodes = V_cal - I_cal * internalResistance;
             float V_H2_eq = V_H2_EVOL_STD;
             float V_O2_eq = V_O2_EVOL_STD;
             float V_cell_eq = V_O2_eq - V_H2_eq;
             float eta_total = V_electrodes - V_cell_eq;

             if (eta_total < 0) { eta_total = 0; /* Add Warning */ }

             float eta_H2_cal = -eta_total / 2.0;
             float eta_O2_cal = eta_total / 2.0;

             // Expected current density using *current* i0 and *calculated* eta_cal
             float i_H2_model = i0_H2 * (exp((alpha * n_H2 * F * eta_H2_cal) / (R * T_kelvin)) - exp(-((1 - alpha) * n_H2 * F * eta_H2_cal) / (R * T_kelvin)));
             float i_O2_model = i0_O2 * (exp((alpha * n_O2 * F * eta_O2_cal) / (R * T_kelvin)) - exp(-((1 - alpha) * n_O2 * F * eta_O2_cal) / (R * T_kelvin)));

             // Expected total model current using *current* parameters (A, i0) at V_cal condition
             float I_model = (fabs(i_H2_model) * A_CATHODE) + (i_O2_model * A_NODE);

             // Calculate Area correction factor
             float correctionFactorArea = safeDivide(I_cal, I_model, 1.0f);

             const float MAX_CORRECTION_AREA = 2.0;
             const float MIN_CORRECTION_AREA = 0.5;
             if (correctionFactorArea > MAX_CORRECTION_AREA) correctionFactorArea = MAX_CORRECTION_AREA;
             if (correctionFactorArea < MIN_CORRECTION_AREA) correctionFactorArea = MIN_CORRECTION_AREA;

             // Store previous values for logging
             float A_CATHODE_prev = A_CATHODE;
             float A_NODE_prev = A_NODE;

             // Apply correction factor to effective areas
             A_CATHODE *= correctionFactorArea;
             A_NODE *= correctionFactorArea;

             Serial.print("  I_cal: "); Serial.print(I_cal, 6); Serial.print(" A, I_model_pred: "); Serial.println(I_model, 6);
             Serial.print("  Area Correction Factor: "); Serial.println(correctionFactorArea, 4);
             Serial.print("  A_CATHODE: "); Serial.print(A_CATHODE_prev, 6); Serial.print(" -> "); Serial.println(A_CATHODE, 6);
             Serial.print("  A_NODE:    "); Serial.print(A_NODE_prev, 6); Serial.print(" -> "); Serial.println(A_NODE, 6);


             // === Part 2: Dynamic Calibration (Adjust i0 using Time Constant) ===
             Serial.println("--- Step 2: Adjusting i0 (Dynamic Response) ---");
             if (dynamicResponseResult.timeConstantMs <= 0) {
                 Serial.println("  Warning: Invalid time constant (<= 0 ms). Skipping i0 adjustment.");
             } else if (A_CATHODE <= 0 || A_NODE <= 0) {
                  Serial.println("  Warning: Invalid area (<= 0). Skipping i0 adjustment.");
             }
             else {
                 // 2a. Estimate total Double Layer Capacitance using *new* areas
                 float C_dl_total_estimated = C_DL_SPECIFIC * (A_NODE + A_CATHODE);
                 if (C_dl_total_estimated <= 0) {
                      Serial.println("  Warning: Estimated C_dl is zero or negative. Skipping i0 adjustment.");
                 } else {
                      // 2b. Estimate total Charge Transfer Resistance from measurement
                      float R_ct_estimated = (dynamicResponseResult.timeConstantMs / 1000.0) / C_dl_total_estimated; // Convert ms to s

                      // 2c. Calculate *theoretical* total Charge Transfer Resistance using *current* i0 and *new* areas
                      // (Using linearized Butler-Volmer approximation R_ct = RT / (nFi0A))
                      float R_ct_H2_theoretical = safeDivide((R * T_kelvin), (n_H2 * F * i0_H2 * A_CATHODE), std::numeric_limits<float>::max());
                      float R_ct_O2_theoretical = safeDivide((R * T_kelvin), (n_O2 * F * i0_O2 * A_NODE), std::numeric_limits<float>::max());
                      float R_ct_theoretical_total = R_ct_H2_theoretical + R_ct_O2_theoretical;

                      // Avoid division by zero if theoretical Rct is somehow zero (e.g., huge i0/A)
                      if(fabs(R_ct_estimated) < std::numeric_limits<float>::epsilon()) {
                          Serial.println("  Warning: Estimated R_ct is near zero. Skipping i0 adjustment.");
                      }
                      else {
                            // 2d. Calculate i0 correction factor
                            // correction = R_ct_theoretical / R_ct_estimated
                            // If R_ct_estimated is higher than theory, correction < 1 (reduce i0)
                            // If R_ct_estimated is lower than theory, correction > 1 (increase i0)
                            float correctionFactor_i0 = safeDivide(R_ct_theoretical_total, R_ct_estimated, 1.0f);

                            // 2e. Apply damping/limits to i0 correction
                            const float MAX_CORRECTION_I0 = 3.0; // Allow larger changes for i0 initially?
                            const float MIN_CORRECTION_I0 = 0.33;
                            if (correctionFactor_i0 > MAX_CORRECTION_I0) correctionFactor_i0 = MAX_CORRECTION_I0;
                            if (correctionFactor_i0 < MIN_CORRECTION_I0) correctionFactor_i0 = MIN_CORRECTION_I0;

                            // Store previous values
                            float i0_H2_prev = i0_H2;
                            float i0_O2_prev = i0_O2;

                            // 2f. Apply correction factor to exchange current densities
                            i0_H2 *= correctionFactor_i0;
                            i0_O2 *= correctionFactor_i0;

                            // Add minimum limits to i0 to prevent them from going to zero/negative
                            const float MIN_I0_VALUE = 1e-9; // A/m^2
                            if (i0_H2 < MIN_I0_VALUE) i0_H2 = MIN_I0_VALUE;
                            if (i0_O2 < MIN_I0_VALUE) i0_O2 = MIN_I0_VALUE;


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
             Serial.println("--- Calibration V3 Complete ---");
             Serial.println("Final Parameters:");
             Serial.print(" A_CATHODE: "); Serial.println(A_CATHODE, 6);
             Serial.print(" A_NODE:    "); Serial.println(A_NODE, 6);
             Serial.print(" i0_H2:     "); Serial.println(i0_H2, 6);
             Serial.print(" i0_O2:     "); Serial.println(i0_O2, 6);
             Serial.println("-----------------------------");

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

    return (currentCalibrationState != CalibrationState::IDLE &&
            currentCalibrationState != CalibrationState::COMPLETE &&
            currentCalibrationState != CalibrationState::ERROR);
}

// --- startCalibration() and isCalibrationComplete() (Keep as before) ---
void startCalibration() { /* ... */ }
bool isCalibrationComplete() { /* ... */ }


//--------------------------------------------------------------
// Example setup and loop (Keep as before, maybe add C_DL_SPECIFIC to output)
//--------------------------------------------------------------
// --- Mock external update variables ---
volatile float batteryVoltage = 3.7;
volatile float batteryCurrent = 0.0;
unsigned long lastExternalUpdate = 0;

void mockExternalUpdate() { /* ... (same as V2) ... */ }

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("ESP32 Electrochemical Calibration V3 Started");
    Serial.print("Assumed C_DL_SPECIFIC: "); Serial.print(C_DL_SPECIFIC); Serial.println(" F/m^2");

    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);
    lastExternalUpdate = millis();
}

void loop() {
    mockExternalUpdate();

    noInterrupts();
    float currentVoltage = batteryVoltage;
    float currentCurrent = batteryCurrent;
    interrupts();

    static float internalResistance = 0.05;
    static float batteryTempC = 25.0;
    static bool calibrationRequested = false;

    bool calibrating = calibrateBatteryChemistryModel_Tick(internalResistance, currentVoltage, batteryTempC);

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' && !calibrating) {
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
        float I_max_pred = computeMaxElectrolysisCurrent(currentVoltage, batteryTempC, internalResistance);
        // ... (Serial printout as before) ...
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

// Make sure to include the full implementations for functions omitted with /* ... */
// bool checkTimer(...) { return (millis() - timerTimestamp >= interval); }
// float safeDivide(...) { if (fabs(denominator) < std::numeric_limits<float>::epsilon()) return fallback; return numerator / denominator; }
// And the full state machine tick functions where indicated.
