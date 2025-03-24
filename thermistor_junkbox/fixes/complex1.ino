#include <cmath>

// Define constants for Butler-Volmer equation and other electrochemical parameters
const float R_GAS = 8.314; // J/(mol*K)
const float F_FARADAY = 96485.0; // C/mol
const int N_ELECTRONS = 1; // Number of electrons transferred (approximate for NiMH)

/**
 * Estimates temperature change of a AAA NiMH cell due to internal heating and cooling effects.
 *
 * This function now incorporates more complex electrochemical models including:
 * - Activation and concentration polarization
 * - Simplified electrolyte electrolysis
 * - Simplified gas recombination
 *
 * @param voltageLoaded     Voltage across cell with current flowing (V)
 * @param voltageUnloaded   Voltage across cell without current (V)
 * @param current           Current through the cell (A)
 * @param internalRes       Baseline internal resistance of cell (Ohms)
 * @param ambientTemp       Ambient temperature (°C)
 * @param cellTemp          Current cell temperature (°C)
 * @param airflowSpeed      Air speed for forced convection (m/s), 0 for natural convection
 * @param contactAreaFrac   Fraction of battery surface in contact with conductive materials (0-1)
 * @param contactThermalRes Thermal resistance of contacted material (K/W)
 * @param cellDiameter      Diameter of AAA cell (m), default 10.5mm
 * @param cellLength        Length of AAA cell (m), default 44.5mm
 * @param timeInterval      Time interval for temperature change calculation (s)
 * @param alpha_a           Anodic transfer coefficient (dimensionless)
 * @param alpha_c           Cathodic transfer coefficient (dimensionless)
 * @param j0                Exchange current density (A/m^2), adjust based on cell area
 * @param R_conc            Concentration polarization resistance (Ohms), adjust based on current/SOC
 * @param U_electrolysis    Voltage threshold for significant electrolysis (V)
 * @param k_electrolysis    Electrolysis rate constant (A/V), adjust based on cell
 * @param k_recombination   Recombination heat transfer coefficient (W/A), adjust based on cell
 * @return                  Estimated temperature change in °C over the specified time interval
 */
float estimateBatteryTempDelta(
  float voltageLoaded,
  float voltageUnloaded,
  float current,
  float internalRes,
  float ambientTemp,
  float cellTemp,
  float airflowSpeed = 0.0,
  float contactAreaFrac = 0.0,
  float contactThermalRes = 10.0,
  float cellDiameter = 0.0105,
  float cellLength = 0.0445,
  float timeInterval = 1.0,
  float alpha_a = 0.5,
  float alpha_c = 0.5,
  float j0 = 0.1, // Example value, needs calibration
  float R_conc = 0.005, // Example value, needs calibration
  float U_electrolysis = 1.5,
  float k_electrolysis = 0.1, // Example value, needs calibration
  float k_recombination = 0.05 // Example value, needs calibration
) {
  // Physical constants
  const float STEFAN_BOLTZMANN = 5.67e-8;
  const float EMISSIVITY = 0.9;
  const float AIR_THERMAL_CONDUCTIVITY = 0.026;
  const float AIR_KINEMATIC_VISCOSITY = 1.568e-5;
  const float AIR_PRANDTL_NUMBER = 0.707;
  const float SPECIFIC_HEAT_CAPACITY_NOMINAL = 1100.0;
  const float DENSITY_NOMINAL = 3000.0;

  // Infer state of charge from unloaded voltage - Improved model
  float stateOfCharge = 0.0;
  if (voltageUnloaded <= 1.0) {
    stateOfCharge = 0.0;
  } else if (voltageUnloaded >= 1.45) {
    stateOfCharge = 1.0;
  } else if (voltageUnloaded <= 1.15) {
    stateOfCharge = (voltageUnloaded - 1.0) / 0.15 * 0.3;
  } else if (voltageUnloaded >= 1.35) {
    stateOfCharge = 0.7 + (voltageUnloaded - 1.35) / 0.1 * 0.3;
  } else {
    stateOfCharge = 0.3 + (voltageUnloaded - 1.15) / 0.2 * 0.4;
  }
  stateOfCharge = constrain(stateOfCharge, 0.0, 1.0);

  // Adjust internal resistance based on battery charge and temperature - Improved model
  float tempFactor = 1.0 + (0.005 + (1.0 - stateOfCharge) * 0.01) * (25.0 - cellTemp);
  float chargeFactor = 1.0;
  if (stateOfCharge < 0.1) {
    chargeFactor = 1.0 + 3.0 * (0.1 - stateOfCharge);
  } else if (stateOfCharge < 0.2) {
    chargeFactor = 1.0 + 1.0 * (0.2 - stateOfCharge);
  } else if (stateOfCharge > 0.9) {
    chargeFactor = 1.0 + 0.8 * (stateOfCharge - 0.9) * 10.0;
  }
  float adjustedInternalRes = internalRes * tempFactor * chargeFactor;

  // Calculate battery dimensions and properties
  float radius = cellDiameter / 2.0;
  float surfaceArea = 2.0 * PI * radius * radius + 2.0 * PI * radius * cellLength;
  float volume = PI * radius * radius * cellLength;
  float density = DENSITY_NOMINAL;
  float specificHeatCapacity = SPECIFIC_HEAT_CAPACITY_NOMINAL;
  float mass = volume * density;
  float thermalCapacity = mass * specificHeatCapacity;
  float effectiveSurfaceArea = surfaceArea * (1.0 - contactAreaFrac);

  // Convert temperatures to Kelvin
  float cellTempK = cellTemp + 273.15;
  float ambientTempK = ambientTemp + 273.15;

  // 1. Ohmic Heating
  float powerFromResistance = current * current * adjustedInternalRes;

  // 2. Activation Polarization Heating (Simplified Butler-Volmer)
  float overpotential_act = 0.0;
  if (abs(current) > 1e-6 && surfaceArea > 0) {
    float currentDensity = current / surfaceArea;
    float RT_nF = R_GAS * cellTempK / (N_ELECTRONS * F_FARADAY);
    overpotential_act = RT_nF * asinh(currentDensity / (2 * j0)); // Simplified for symmetric coefficients
  }
  float powerFromActivation = abs(current) * abs(overpotential_act);

  // 3. Concentration Polarization Heating (Simplified)
  float overpotential_conc = abs(current) * R_conc;
  float powerFromConcentration = abs(current) * abs(overpotential_conc);

  // 4. Electrolysis Heating (Simplified)
  float powerFromElectrolysis = 0.0;
  if (voltageUnloaded > U_electrolysis && current < 0) {
    powerFromElectrolysis = (voltageUnloaded - U_electrolysis) * abs(current) * k_electrolysis;
  }

  // 5. Recombination Heating (Simplified)
  float powerFromRecombination = 0.0;
  if (stateOfCharge > 0.95 && current < 0) {
    powerFromRecombination = abs(current) * k_recombination * (stateOfCharge - 0.95) * 20.0;
  }

  // Total electrochemical heating
  float electrochemicalHeating = powerFromActivation + powerFromConcentration + powerFromElectrolysis + powerFromRecombination;

  // Total heat generation
  float powerGenerated = powerFromResistance + electrochemicalHeating;

  // Calculate heat loss through convection
  float convectionCoeff;
  if (airflowSpeed <= 0.001) {
    // Natural convection
    float rayleighNumber = 9.81 * (1.0/ambientTempK) * abs(cellTemp - ambientTemp) *
                            pow(cellDiameter, 3) / (AIR_KINEMATIC_VISCOSITY * AIR_KINEMATIC_VISCOSITY) *
                            AIR_PRANDTL_NUMBER;
    float nusseltNumber;
    if (rayleighNumber < 1e-4) nusseltNumber = 0.4;
    else if (rayleighNumber < 1e12) nusseltNumber = 0.53 * pow(rayleighNumber, 0.25);
    else nusseltNumber = 0.13 * pow(rayleighNumber, 0.33);
    convectionCoeff = nusseltNumber * AIR_THERMAL_CONDUCTIVITY / cellDiameter;
  } else {
    // Forced convection
    float reynoldsNumber = airflowSpeed * cellDiameter / AIR_KINEMATIC_VISCOSITY;
    float nusseltNumber;
    if (reynoldsNumber < 4000) nusseltNumber = 0.3 + 0.62 * sqrt(reynoldsNumber) * pow(AIR_PRANDTL_NUMBER, 0.33) * pow(1 + pow(reynoldsNumber/282000, 0.625), 0.8);
    else nusseltNumber = 0.024 * pow(reynoldsNumber, 0.8) * pow(AIR_PRANDTL_NUMBER, 0.33);
    convectionCoeff = nusseltNumber * AIR_THERMAL_CONDUCTIVITY / cellDiameter;
  }
  float convectionLoss = convectionCoeff * effectiveSurfaceArea * (cellTemp - ambientTemp);

  // Calculate heat loss through radiation
  float radiationLoss = EMISSIVITY * STEFAN_BOLTZMANN * effectiveSurfaceArea *
                          (pow(cellTempK, 4) - pow(ambientTempK, 4));

  // Calculate heat loss through conduction
  float conductionLoss = 0.0;
  if (contactAreaFrac > 0.0) {
    conductionLoss = (cellTemp - ambientTemp) / contactThermalRes;
  }

  // Calculate net heat flow
  float netHeatFlow = powerGenerated - convectionLoss - radiationLoss - conductionLoss;

  // Calculate temperature change
  float tempChange = (netHeatFlow * timeInterval) / thermalCapacity;

  if (!isfinite(tempChange) || abs(tempChange) > 100.0) {
    tempChange = (tempChange > 0) ? 5.0 : -5.0;
  }

  return tempChange;
}

/**
 * Returns detailed battery heating components for analysis with increased electrochemical model complexity
 */
struct BatteryHeatingComponents {
  float resistiveHeating;
  float activationPolarizationHeating;
  float concentrationPolarizationHeating;
  float electrolysisHeating;
  float recombinationHeating;
  float convectionCooling;
  float radiationCooling;
  float conductionCooling;
  float netHeating;
  float stateOfCharge;
  float tempChangeRate;
};

BatteryHeatingComponents analyzeBatteryHeating(
  float voltageLoaded,
  float voltageUnloaded,
  float current,
  float internalRes,
  float ambientTemp,
  float cellTemp,
  float airflowSpeed = 0.0,
  float contactAreaFrac = 0.0,
  float contactThermalRes = 10.0,
  float alpha_a = 0.5,
  float alpha_c = 0.5,
  float j0 = 0.1,
  float R_conc = 0.005,
  float U_electrolysis = 1.5,
  float k_electrolysis = 0.1,
  float k_recombination = 0.05
) {
  // Physical constants
  const float STEFAN_BOLTZMANN = 5.67e-8;
  const float EMISSIVITY = 0.9;
  const float AIR_THERMAL_CONDUCTIVITY = 0.026;
  const float AIR_KINEMATIC_VISCOSITY = 1.568e-5;
  const float AIR_PRANDTL_NUMBER = 0.707;
  const float SPECIFIC_HEAT_CAPACITY_NOMINAL = 1100.0;
  const float DENSITY_NOMINAL = 3000.0;

  BatteryHeatingComponents result;

  float cellDiameter = 0.0105;
  float cellLength = 0.0445;
  float radius = cellDiameter / 2.0;
  float surfaceArea = 2.0 * PI * radius * radius + 2.0 * PI * radius * cellLength;
  float volume = PI * radius * radius * cellLength;
  float density = DENSITY_NOMINAL;
  float specificHeatCapacity = SPECIFIC_HEAT_CAPACITY_NOMINAL;
  float mass = volume * density;
  float thermalCapacity = mass * specificHeatCapacity;
  float effectiveSurfaceArea = surfaceArea * (1.0 - contactAreaFrac);

  // Infer state of charge from unloaded voltage
  if (voltageUnloaded <= 1.0) result.stateOfCharge = 0.0;
  else if (voltageUnloaded >= 1.45) result.stateOfCharge = 1.0;
  else if (voltageUnloaded <= 1.15) result.stateOfCharge = (voltageUnloaded - 1.0) / 0.15 * 0.3;
  else if (voltageUnloaded >= 1.35) result.stateOfCharge = 0.7 + (voltageUnloaded - 1.35) / 0.1 * 0.3;
  else result.stateOfCharge = 0.3 + (voltageUnloaded - 1.15) / 0.2 * 0.4;
  result.stateOfCharge = constrain(result.stateOfCharge, 0.0, 1.0);

  // Adjust internal resistance
  float tempFactor = 1.0 + (0.005 + (1.0 - result.stateOfCharge) * 0.01) * (25.0 - cellTemp);
  float chargeFactor = 1.0;
  if (result.stateOfCharge < 0.1) chargeFactor = 1.0 + 3.0 * (0.1 - result.stateOfCharge);
  else if (result.stateOfCharge < 0.2) chargeFactor = 1.0 + 1.0 * (0.2 - result.stateOfCharge);
  else if (result.stateOfCharge > 0.9) chargeFactor = 1.0 + 0.8 * (result.stateOfCharge - 0.9) * 10.0;
  float adjustedInternalRes = internalRes * tempFactor * chargeFactor;

  // 1. Ohmic Heating
  result.resistiveHeating = current * current * adjustedInternalRes;

  // 2. Activation Polarization Heating
  result.activationPolarizationHeating = 0.0;
  if (abs(current) > 1e-6 && surfaceArea > 0) {
    float cellTempK = cellTemp + 273.15;
    float currentDensity = current / surfaceArea;
    float RT_nF = R_GAS * cellTempK / (N_ELECTRONS * F_FARADAY);
    float overpotential_act = RT_nF * asinh(currentDensity / (2 * j0));
    result.activationPolarizationHeating = abs(current) * abs(overpotential_act);
  }

  // 3. Concentration Polarization Heating
  result.concentrationPolarizationHeating = abs(current) * abs(current) * R_conc;

  // 4. Electrolysis Heating
  result.electrolysisHeating = 0.0;
  if (voltageUnloaded > U_electrolysis && current < 0) {
    result.electrolysisHeating = (voltageUnloaded - U_electrolysis) * abs(current) * k_electrolysis;
  }

  // 5. Recombination Heating
  result.recombinationHeating = 0.0;
  if (result.stateOfCharge > 0.95 && current < 0) {
    result.recombinationHeating = abs(current) * k_recombination * (result.stateOfCharge - 0.95) * 20.0;
  }

  // Calculate cooling components
  float cellTempK = cellTemp + 273.15;
  float ambientTempK = ambientTemp + 273.15;

  float convectionCoeff;
  if (airflowSpeed <= 0.001) {
    float rayleighNumber = 9.81 * (1.0/ambientTempK) * abs(cellTemp - ambientTemp) *
                            pow(cellDiameter, 3) / (AIR_KINEMATIC_VISCOSITY * AIR_KINEMATIC_VISCOSITY) *
                            AIR_PRANDTL_NUMBER;
    float nusseltNumber;
    if (rayleighNumber < 1e-4) nusseltNumber = 0.4;
    else if (rayleighNumber < 1e12) nusseltNumber = 0.53 * pow(rayleighNumber, 0.25);
    else nusseltNumber = 0.13 * pow(rayleighNumber, 0.33);
    convectionCoeff = nusseltNumber * AIR_THERMAL_CONDUCTIVITY / cellDiameter;
  } else {
    float reynoldsNumber = airflowSpeed * cellDiameter / AIR_KINEMATIC_VISCOSITY;
    float nusseltNumber;
    if (reynoldsNumber < 4000) nusseltNumber = 0.3 + 0.62 * sqrt(reynoldsNumber) * pow(AIR_PRANDTL_NUMBER, 0.33) * pow(1 + pow(reynoldsNumber/282000, 0.625), 0.8);
    else nusseltNumber = 0.024 * pow(reynoldsNumber, 0.8) * pow(AIR_PRANDTL_NUMBER, 0.33);
    convectionCoeff = nusseltNumber * AIR_THERMAL_CONDUCTIVITY / cellDiameter;
  }
  result.convectionCooling = convectionCoeff * effectiveSurfaceArea * (cellTemp - ambientTemp);

  result.radiationCooling = EMISSIVITY * STEFAN_BOLTZMANN * effectiveSurfaceArea *
                            (pow(cellTempK, 4) - pow(ambientTempK, 4));

  result.conductionCooling = 0.0;
  if (contactAreaFrac > 0.0) {
    result.conductionCooling = (cellTemp - ambientTemp) / contactThermalRes;
  }

  // Calculate net heating and temperature change rate
  result.netHeating = result.resistiveHeating + result.activationPolarizationHeating + result.concentrationPolarizationHeating + result.electrolysisHeating + result.recombinationHeating -
                      result.convectionCooling - result.radiationCooling - result.conductionCooling;

  result.tempChangeRate = result.netHeating / thermalCapacity;

  return result;
}

/**
 * Function to perform various tests and calibrate the electrochemical model parameters.
 * This is a conceptual outline and would require significant implementation.
 *
 * @param initialTemp       Initial cell temperature (°C)
 * @param ambientTemp       Ambient temperature (°C)
 * @param cellDiameter      Diameter of AAA cell (m)
 * @param cellLength        Length of AAA cell (m)
 * @param capacity_mAh      Nominal capacity of the cell in mAh
 * @param initialVoltage    Initial open circuit voltage (V)
 * @param testData          Structure to hold measured voltage and temperature data for different tests
 * @param numDataPoints     Number of data points in the testData
 * @param timestep          Time interval for measurements (s)
 * @return                  A structure containing the calibrated parameters
 */
// Structure to hold test data (conceptual)
struct TestDataPoint {
  float time;
  float voltage;
  float temperature;
  float current; // Applied current during the test
};

// Structure to hold calibrated parameters
struct CalibrationParameters {
  float baselineInternalResistance;
  float alpha_a;
  float alpha_c;
  float j0;
  float R_conc_coeff; // Coefficient for concentration resistance dependency
  float U_electrolysis;
  float k_electrolysis;
  float k_recombination;
};

CalibrationParameters calibrateBatteryModel(
  float initialTemp,
  float ambientTemp,
  float cellDiameter,
  float cellLength,
  float capacity_mAh,
  float initialVoltage,
  const TestDataPoint* testData,
  int numDataPoints,
  float timestep
) {
  CalibrationParameters params;
  // Initialize parameters with default or initial guess values
  params.baselineInternalResistance = 0.05; // Example
  params.alpha_a = 0.5;
  params.alpha_c = 0.5;
  params.j0 = 0.1;
  params.R_conc_coeff = 0.001;
  params.U_electrolysis = 1.5;
  params.k_electrolysis = 0.1;
  params.k_recombination = 0.05;

  // --- Calibration Procedure Outline ---

  // 1. Internal Resistance Calibration:
  //    - Perform discharge pulses at different SOC and temperatures.
  //    - Measure voltage drop and current to estimate resistance.
  //    - Fit a model for internal resistance as a function of SOC and temperature
  //      (e.g., using the tempFactor and chargeFactor from the main model).
  //    - This would involve iterating through the test data and potentially using
  //      a least-squares fitting algorithm.

  // 2. Polarization Parameter Calibration (alpha_a, alpha_c, j0):
  //    - Perform charge and discharge at various constant current rates.
  //    - Measure the voltage response and compare it to the model predictions
  //      that include the Butler-Volmer equation.
  //    - Adjust alpha_a, alpha_c, and j0 to minimize the error between the model
  //      and the measured data. This might require numerical optimization techniques.

  // 3. Concentration Polarization Resistance (R_conc_coeff):
  //    - Focus on high current discharge data where concentration polarization is more significant.
  //    - Fit the model by adjusting R_conc (which might be modeled as R_conc_coeff * current or a function of current and SOC).

  // 4. Electrolysis Threshold and Rate (U_electrolysis, k_electrolysis):
  //    - Perform constant voltage charging at a voltage above the expected electrolysis threshold.
  //    - Measure the current and temperature change.
  //    - Adjust U_electrolysis and k_electrolysis to match the observed behavior.

  // 5. Recombination Heating Coefficient (k_recombination):
  //    - Analyze data from overcharging tests, particularly the temperature rise after reaching full charge.
  //    - Adjust k_recombination to match the observed heating.

  // --- Example Implementation Snippet (Illustrative for Internal Resistance) ---
  // std::vector<float> resistances;
  // std::vector<float> temps;
  // std::vector<float> socs;
  // for (int i = 0; i < numDataPoints; ++i) {
  //   if (/* Condition for a discharge pulse */) {
  //     float resistance = (testData[i].voltage - testData[i+1].voltage) / testData[i].current;
  //     resistances.push_back(resistance);
  //     temps.push_back(testData[i].temperature);
  //     // Estimate SOC based on voltage (you might need a more accurate method)
  //     float soc = 0.0;
  //     if (testData[i].voltage <= 1.0) soc = 0.0;
  //     // ... SOC estimation logic ...
  //     socs.push_back(soc);
  //   }
  // }
  // // Now, fit a function to resistances based on temps and socs to find baseline resistance
  // // and the coefficients for tempFactor and chargeFactor.

  // --- End of Calibration Procedure Outline ---

  Serial.println(F("Calibration function called (implementation needed)"));
  return params;
}

float predictBatteryTemp(
  float initialTemp,
  float ambientTemp,
  float current,
  float internalRes,
  float duration,
  float initialVoltage,
  float timestep = 1.0,
  float logInterval = 60.0,
  float alpha_a = 0.5,
  float alpha_c = 0.5,
  float j0 = 0.1,
  float R_conc = 0.005,
  float U_electrolysis = 1.5,
  float k_electrolysis = 0.1,
  float k_recombination = 0.05
) {
  float cellTemp = initialTemp;
  float timeElapsed = 0;
  float openCircuitVoltage = initialVoltage;
  float loadedVoltage = openCircuitVoltage - (current * internalRes);
  float lastLogTime = 0;

  float stateOfCharge;
  if (openCircuitVoltage <= 1.0) stateOfCharge = 0.0;
  else if (openCircuitVoltage >= 1.45) stateOfCharge = 1.0;
  else if (openCircuitVoltage <= 1.15) stateOfCharge = (openCircuitVoltage - 1.0) / 0.15 * 0.3;
  else if (openCircuitVoltage >= 1.35) stateOfCharge = 0.7 + (openCircuitVoltage - 1.35) / 0.1 * 0.3;
  else stateOfCharge = 0.3 + (openCircuitVoltage - 1.15) / 0.2 * 0.4;
  stateOfCharge = constrain(stateOfCharge, 0.0, 1.0);

  float capacity = 800.0;

  Serial.println(F("Time(s),Tem
