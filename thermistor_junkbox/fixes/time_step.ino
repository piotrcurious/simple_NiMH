// Helper function to update the battery temperature over a time step.
// Inputs:
//   currentTemp   : current battery temperature (°C)
//   batteryMass   : battery mass (kg)
//   internalRes   : internal resistance (ohm)
//   current       : current flowing (A)
//   voltageZero   : battery voltage at zero current (V) [for calibration]
//   voltageLoad   : battery voltage with current applied (V) [for calibration]
//   dt            : time step (seconds)
// Returns the updated battery temperature (°C)
float updateBatteryTemperature(float currentTemp, float batteryMass, 
                               float internalRes, float current, 
                               float voltageZero, float voltageLoad, 
                               float dt) {
  // Assumed constants
  const float ambientTemp = 25.0;      // Ambient temperature in °C (could be set by voltageZero calibration)
  const float specificHeat = 1000.0;     // Specific heat capacity (J/kg·K)
  const float A = 0.0015;              // Estimated battery surface area (m^2)
  const float h = 10.0;                // Convection coefficient (W/m^2·K)
  const float epsilon = 0.9;           // Emissivity (dimensionless)
  const float sigma = 5.67e-8;         // Stefan-Boltzmann constant (W/m^2·K^4)
  
  // Convert ambient temperature to Kelvin for radiation term
  float T_ambient_K = ambientTemp + 273.15;
  
  // Electrical power dissipated in the battery (W)
  float power = current * current * internalRes;
  
  // Effective thermal conductance (W/K) using convection and linearized radiation:
  float G = h * A + 4 * epsilon * sigma * A * pow(T_ambient_K, 3);
  
  // Net power available to raise the battery's temperature.
  // Losses are assumed proportional to the temperature difference (linearized losses).
  float netPower = power - G * (currentTemp - ambientTemp);
  
  // Compute the rate of temperature change (°C/s)
  float dTdt = netPower / (batteryMass * specificHeat);
  
  // Update the battery temperature over the time step dt.
  float newTemp = currentTemp + dTdt * dt;
  
  return newTemp;
}

// Example usage:
void setup() {
  Serial.begin(9600);
  
  // Example parameters:
  float currentTemp = 25.0;       // initial battery temperature in °C
  float batteryMass = 0.015;      // estimated mass in kg (15 g typical for AAA NiMH)
  float internalRes = 0.1;        // internal resistance in ohms
  float current = 1.0;            // applied current in A
  float voltageZero = 1.3;        // zero-current voltage (V)
  float voltageLoad = 1.2;        // voltage under load (V)
  float dt = 1.0;                 // time step in seconds
  
  // Simulate temperature update over one time step
  float updatedTemp = updateBatteryTemperature(currentTemp, batteryMass,
                                               internalRes, current, 
                                               voltageZero, voltageLoad, dt);
  
  Serial.print("Updated battery temperature: ");
  Serial.print(updatedTemp);
  Serial.println(" °C");
}

void loop() {
  // In a real application, update the battery temperature repeatedly
  // as new measurements become available.
}
