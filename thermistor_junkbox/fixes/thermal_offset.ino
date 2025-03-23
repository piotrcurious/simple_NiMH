// Function to estimate temperature difference (in °C) due to heating by current
float estimateTempDiff(float voltageUnderLoad, float voltageNoLoad, 
                         float current, float internalResistance, 
                         float ambientTempC) {
  // Calculate electrical power dissipated (in Watts)
  // (This assumes that the voltage drop due to the internal resistance is given by I*R)
  float power = current * current * internalResistance;
  
  // Assumed battery parameters for a AAA NiMH cell:
  // Estimated surface area (m^2) of a typical AAA cell (~44mm x 10mm cylinder)
  float A = 0.0015; // m^2
  
  // Convective heat transfer coefficient (W/m^2·K)
  float h = 10.0;
  
  // Radiation parameters
  float epsilon = 0.9;           // Emissivity (dimensionless)
  float sigma = 5.67e-8;         // Stefan-Boltzmann constant (W/m^2·K^4)
  
  // Convert ambient temperature from Celsius to Kelvin
  float T_ambient = ambientTempC + 273.15;
  
  // Effective thermal conductance (W/K)
  // For small ΔT, the radiation term can be linearized: 
  //   Radiation loss ≈ 4 * ε * σ * A * T_ambient^3 * ΔT
  float G = h * A + 4 * epsilon * sigma * A * pow(T_ambient, 3);
  
  // Estimate temperature rise (ΔT in Kelvin, equivalent to °C difference)
  float deltaT = power / G;
  
  return deltaT;
}

// Example usage:
void setup() {
  Serial.begin(9600);
  
  // Example values:
  // Voltage across cell with current (V): 1.2 V (under load)
  // Voltage across cell with no current (V): 1.3 V (open-circuit)
  // Current (A): 1.0 A
  // Internal resistance (ohms): 0.1 ohm
  // Ambient temperature (°C): 25.0 °C
  float V_load = 1.2;
  float V_no_load = 1.3;
  float I = 1.0;
  float R = 0.1;
  float ambientC = 25.0;
  
  float tempRise = estimateTempDiff(V_load, V_no_load, I, R, ambientC);
  
  Serial.print("Estimated temperature rise: ");
  Serial.print(tempRise);
  Serial.println(" °C");
}

void loop() {
  // Nothing to do in loop.
}
