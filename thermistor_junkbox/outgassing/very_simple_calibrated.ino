#include <Arduino.h>

// Pin used to enable battery charging PWM (must be defined per your hardware)
#define PWM_PIN  5  

// Global measured battery values (updated by a separate system task every 500ms)
extern float batteryVoltage;  // Battery voltage (V)
extern float batteryCurrent;  // Battery current (A)

// Electrochemical constants and global parameters (modifiable via calibration)
const float F = 96485.33212;   // Faraday's constant (C/mol)
const float R = 8.3145;        // Universal gas constant (J/(mol*K))
const float V_H2_EVOL = -0.83; // Standard hydrogen evolution potential (V)
const float V_O2_EVOL = 0.40;  // Standard oxygen evolution potential (V)
float A_NODE = 1.2e-4;         // Effective anode surface area (m^2) [modifiable]
float A_CATHODE = 1.5e-4;      // Effective cathode surface area (m^2) [modifiable]
const float alpha = 0.5;       // Charge transfer coefficient
float i0_H2 = 1e-6;          // Exchange current density for H2 evolution (A/m^2) [modifiable if needed]
float i0_O2 = 2e-6;          // Exchange current density for O2 evolution (A/m^2) [modifiable if needed]
const float n_H2 = 2.0;        // Number of electrons for hydrogen evolution
const float n_O2 = 4.0;        // Number of electrons for oxygen evolution
const float E_overpot = 0.1;   // Overpotential threshold for electrolysis onset (V)

//--------------------------------------------------------------
// computeMaxElectrolysisCurrent()
// 
// Uses an electrochemical model (Nernst and Butler-Volmer equations)
// and additional measured data (via PWM load) to predict the maximum
// threshold current before electrolyte electrolysis occurs.
//--------------------------------------------------------------
float computeMaxElectrolysisCurrent(float internalResistance, float idleVoltage, float batteryTemperature) {
  float T = batteryTemperature + 273.15;  // Convert to Kelvin

  // Compute Nernst potentials (assuming electrolyte activity ~1)
  float V_H2 = V_H2_EVOL;  // No log correction when activity == 1
  float V_O2 = V_O2_EVOL;

  // Overpotential-adjusted thresholds
  float V_H2_thresh = V_H2 + E_overpot;
  float V_O2_thresh = V_O2 + E_overpot;

  // Predicted cell voltage for simultaneous H2 and O2 evolution
  float electrolysisVoltage = fabs(V_H2_thresh) + fabs(V_O2_thresh);

  // Perform a controlled measurement: enable charging PWM and sample values
  digitalWrite(PWM_PIN, HIGH);
  delay(1000);  // Allow system to stabilize
  float V_measured = batteryVoltage;
  float I_measured = batteryCurrent;
  digitalWrite(PWM_PIN, LOW);

  // Model adjustment based on observed voltage drop (optional)
  float V_drop = idleVoltage - V_measured;
  if (V_drop > 0.1) { // if significant drop, adjust threshold slightly
    electrolysisVoltage += 0.05;
  }

  // Butler-Volmer equation applied to both interfaces
  float i_H2 = i0_H2 * (exp((alpha * n_H2 * F * V_H2_thresh) / (R * T)) - exp(-((1 - alpha) * n_H2 * F * V_H2_thresh) / (R * T)));
  float i_O2 = i0_O2 * (exp((alpha * n_O2 * F * V_O2_thresh) / (R * T)) - exp(-((1 - alpha) * n_O2 * F * V_O2_thresh) / (R * T)));

  // Total electrolysis current from both electrodes
  float I_H2 = i_H2 * A_CATHODE;
  float I_O2 = i_O2 * A_NODE;
  float I_electrolysis = I_H2 + I_O2;

  // Compute maximum current by comparing with voltage drop across the internal resistance
  float I_max = (electrolysisVoltage - idleVoltage) / internalResistance;
  I_max = max(I_max, I_electrolysis);

  return I_max;
}

//--------------------------------------------------------------
// calibrateBatteryChemistryModel()
// 
// In calibration mode the system applies the PWM charging signal,
// waits for stabilization, and then compares the measured battery current
// against the predicted current from the model. A correction factor is then
// computed and applied to the effective electrode areas.
//--------------------------------------------------------------
void calibrateBatteryChemistryModel(float internalResistance, float idleVoltage, float batteryTemperature) {
  // Set up calibration measurement by enabling PWM
  digitalWrite(PWM_PIN, HIGH);
  delay(2000);  // Wait longer for calibration stabilization
  float V_cal = batteryVoltage;
  float I_cal = batteryCurrent;
  digitalWrite(PWM_PIN, LOW);

  // Recompute model prediction for the calibration conditions
  float T = batteryTemperature + 273.15;  // in Kelvin
  float V_H2 = V_H2_EVOL;
  float V_O2 = V_O2_EVOL;
  float V_H2_thresh = V_H2 + E_overpot;
  float V_O2_thresh = V_O2 + E_overpot;
  float i_H2 = i0_H2 * (exp((alpha * n_H2 * F * V_H2_thresh) / (R * T)) - exp(-((1 - alpha) * n_H2 * F * V_H2_thresh) / (R * T)));
  float i_O2 = i0_O2 * (exp((alpha * n_O2 * F * V_O2_thresh) / (R * T)) - exp(-((1 - alpha) * n_O2 * F * V_O2_thresh) / (R * T)));
  float I_model = i_H2 * A_CATHODE + i_O2 * A_NODE;

  // Calculate correction factor based on measured vs. predicted current
  float correctionFactor = I_cal / I_model;
  
  // Update the effective electrode areas (or alternatively adjust i0 values)
  A_CATHODE *= correctionFactor;
  A_NODE *= correctionFactor;

  // Output calibration results
  Serial.print(\"Calibration complete. Correction factor: \"); Serial.println(correctionFactor);
  Serial.print(\"New A_CATHODE: \"); Serial.println(A_CATHODE, 6);
  Serial.print(\"New A_NODE: \"); Serial.println(A_NODE, 6);
}

//--------------------------------------------------------------
// Example setup and loop routines
//--------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  // (Other initialization code as needed)
}

void loop() {
  // Example usage:
  float internalResistance = 0.05;  // example value in ohms
  float idleVoltage = batteryVoltage; // assuming measured at idle
  float batteryTempC = 25.0;          // example ambient temperature in °C

  // Compute maximum electrolysis current based on current model and measurements
  float I_max = computeMaxElectrolysisCurrent(internalResistance, idleVoltage, batteryTempC);
  Serial.print(\"Predicted max electrolysis current: \"); Serial.println(I_max, 6);

  // Optionally perform calibration periodically (or via command/trigger)
  // calibrateBatteryChemistryModel(internalResistance, idleVoltage, batteryTempC);

  delay(5000);  // Loop delay for demonstration
}
