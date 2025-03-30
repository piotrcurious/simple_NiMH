#include <Arduino.h>

// Global variables updated by a separate system task every 500ms extern float batteryVoltage;  // Measured battery voltage extern float batteryCurrent;  // Measured battery current

// Constants for Ni-MH electrolysis threshold calculations const float F = 96485.33212;  // Faraday's constant (C/mol) const float R = 8.3145;       // Universal gas constant (J/(mol*K)) const float V_H2_EVOL = -0.83; // Standard hydrogen evolution potential (V) const float V_O2_EVOL = 0.40;  // Standard oxygen evolution potential (V) const float A_NODE = 1.2e-4;   // Effective anode surface area (m^2) const float A_CATHODE = 1.5e-4; // Effective cathode surface area (m^2) const float alpha = 0.5;       // Charge transfer coefficient const float i0_H2 = 1e-6;      // Exchange current density for H2 evolution (A/m^2) const float i0_O2 = 2e-6;      // Exchange current density for O2 evolution (A/m^2) const float n_H2 = 2.0;        // Number of electrons for hydrogen evolution const float n_O2 = 4.0;        // Number of electrons for oxygen evolution const float E_overpot = 0.1;   // Overpotential threshold for electrolysis onset (V)

float computeMaxElectrolysisCurrent(float internalResistance, float idleVoltage, float batteryTemperature) { float T = batteryTemperature + 273.15;  // Convert to Kelvin

// Nernst equation for hydrogen evolution potential
float V_H2 = V_H2_EVOL - (R * T / (n_H2 * F)) * log(1.0); // Activity ~1 in electrolyte

// Nernst equation for oxygen evolution potential
float V_O2 = V_O2_EVOL - (R * T / (n_O2 * F)) * log(1.0);

// Overpotential-adjusted electrolysis onset potentials
float V_H2_thresh = V_H2 + E_overpot;
float V_O2_thresh = V_O2 + E_overpot;

// Predicted electrolysis voltage threshold
float electrolysisVoltage = abs(V_H2_thresh) + abs(V_O2_thresh);

// Confirm model with real-time measurements
digitalWrite(PWM_PIN, HIGH);
delay(1000);
float V_measured = batteryVoltage;
float I_measured = batteryCurrent;
digitalWrite(PWM_PIN, LOW);

// Validate chemistry model by comparing idle voltage and loaded voltage
float V_drop = idleVoltage - V_measured;
if (V_drop > 0.1) { // Significant voltage drop detected
    electrolysisVoltage += 0.05; // Adjust model threshold
}

// Butler-Volmer equation for current density at the electrode interface
float i_H2 = i0_H2 * (exp((alpha * n_H2 * F * V_H2_thresh) / (R * T)) - exp(-((1 - alpha) * n_H2 * F * V_H2_thresh) / (R * T)));
float i_O2 = i0_O2 * (exp((alpha * n_O2 * F * V_O2_thresh) / (R * T)) - exp(-((1 - alpha) * n_O2 * F * V_O2_thresh) / (R * T)));

// Total electrolysis current (sum of anode and cathode contributions)
float I_H2 = i_H2 * A_CATHODE;
float I_O2 = i_O2 * A_NODE;
float I_electrolysis = I_H2 + I_O2;

// Consider the internal resistance effect
float I_max = (electrolysisVoltage - idleVoltage) / internalResistance;
I_max = max(I_max, I_electrolysis);

return I_max;

}

