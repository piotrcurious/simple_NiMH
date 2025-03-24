#include <cmath>
#include <Eigen/Dense>

// Define constants for Butler-Volmer equation and other electrochemical parameters
const float R_GAS = 8.314; // J/(mol*K)
const float F_FARADAY = 96485.0; // C/mol
const int N_ELECTRONS = 1; // Number of electrons transferred (approximate for NiMH)

// Helper function to estimate SOC from voltage
float estimateSOC(float voltage) {
  if (voltage <= 1.0) return 0.0;
  else if (voltage >= 1.45) return 1.0;
  else if (voltage <= 1.15) return (voltage - 1.0) / 0.15 * 0.3;
  else if (voltage >= 1.35) return 0.7 + (voltage - 1.35) / 0.1 * 0.3;
  else return 0.3 + (voltage - 1.15) / 0.2 * 0.4;
}

// Helper function for temperature factor of internal resistance
float tempFactor(float soc, float temp) {
  return 1.0 + (0.005 + (1.0 - soc) * 0.01) * (25.0 - temp);
}

// Helper function for charge factor of internal resistance
float chargeFactor(float soc) {
  if (soc < 0.1) return 1.0 + 3.0 * (0.1 - soc);
  else if (soc < 0.2) return 1.0 + 1.0 * (0.2 - soc);
  else if (soc > 0.9) return 1.0 + 0.8 * (soc - 0.9) * 10.0;
  else return 1.0;
}

/**
 * Estimates temperature change of a AAA NiMH cell due to internal heating and cooling effects.
 * ... (rest of the estimateBatteryTempDelta function remains the same) ...
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
  float j0 = 0.1,
  float R_conc = 0.005,
  float U_electrolysis = 1.5,
  float k_electrolysis = 0.1,
  float k_recombination = 0.05
) {
  // ... (function body as before) ...
  const float STEFAN_BOLTZMANN = 5.67e-8;
  const float EMISSIVITY = 0.9;
  const float AIR_THERMAL_CONDUCTIVITY = 0.026;
  const float AIR_KINEMATIC_VISCOSITY = 1.568e-5;
  const float AIR_PRANDTL_NUMBER = 0.707;
  const float SPECIFIC_HEAT_CAPACITY_NOMINAL = 1100.0;
  const float DENSITY_NOMINAL = 3000.0;

  float stateOfCharge = estimateSOC(voltageUnloaded);

  float tempFactorVal = tempFactor(stateOfCharge, cellTemp);
  float chargeFactorVal = chargeFactor(stateOfCharge);
  float adjustedInternalRes = internalRes * tempFactorVal * chargeFactorVal;

  float radius = cellDiameter / 2.0;
  float surfaceArea = 2.0 * PI * radius * radius + 2.0 * PI * radius * cellLength;
  float volume = PI * radius * radius * cellLength;
  float density = DENSITY_NOMINAL;
  float specificHeatCapacity = SPECIFIC_HEAT_CAPACITY_NOMINAL;
  float mass = volume * density;
  float thermalCapacity = mass * specificHeatCapacity;
  float effectiveSurfaceArea = surfaceArea * (1.0 - contactAreaFrac);

  float cellTempK = cellTemp + 273.15;
  float ambientTempK = ambientTemp + 273.15;

  float powerFromResistance = current * current * adjustedInternalRes;

  float overpotential_act = 0.0;
  if (abs(current) > 1e-6 && surfaceArea > 0) {
    float currentDensity = current / surfaceArea;
    float RT_nF = R_GAS * cellTempK / (N_ELECTRONS * F_FARADAY);
    overpotential_act = RT_nF * asinh(currentDensity / (2 * j0));
  }
  float powerFromActivation = abs(current) * abs(overpotential_act);

  float overpotential_conc = abs(current) * R_conc;
  float powerFromConcentration = abs(current) * abs(overpotential_conc);

  float powerFromElectrolysis = 0.0;
  if (voltageUnloaded > U_electrolysis && current < 0) {
    powerFromElectrolysis = (voltageUnloaded - U_electrolysis) * abs(current) * k_electrolysis;
  }

  float powerFromRecombination = 0.0;
  if (stateOfCharge > 0.95 && current < 0) {
    powerFromRecombination = abs(current) * k_recombination * (stateOfCharge - 0.95) * 20.0;
  }

  float electrochemicalHeating = powerFromActivation + powerFromConcentration + powerFromElectrolysis + powerFromRecombination;
  float powerGenerated = powerFromResistance + electrochemicalHeating;

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
  float convectionLoss = convectionCoeff * effectiveSurfaceArea * (cellTemp - ambientTemp);

  float radiationLoss = EMISSIVITY * STEFAN_BOLTZMANN * effectiveSurfaceArea *
                          (pow(cellTempK, 4) - pow(ambientTempK, 4));

  float conductionLoss = 0.0;
  if (contactAreaFrac > 0.0) {
    conductionLoss = (cellTemp - ambientTemp) / contactThermalRes;
  }

  float netHeatFlow = powerGenerated - convectionLoss - radiationLoss - conductionLoss;
  float tempChange = (netHeatFlow * timeInterval) / thermalCapacity;

  if (!isfinite(tempChange) || abs(tempChange) > 100.0) {
    tempChange = (tempChange > 0) ? 5.0 : -5.0;
  }

  return tempChange;
}

/**
 * Returns detailed battery heating components for analysis with increased electrochemical model complexity
 * ... (rest of the analyzeBatteryHeating function remains the same) ...
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
  // ... (function body as before) ...
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

  result.stateOfCharge = estimateSOC(voltageUnloaded);

  float tempFactorVal = tempFactor(result.stateOfCharge, cellTemp);
  float chargeFactorVal = chargeFactor(result.stateOfCharge);
  float adjustedInternalRes = internalRes * tempFactorVal * chargeFactorVal;

  result.resistiveHeating = current * current * adjustedInternalRes;

  result.activationPolarizationHeating = 0.0;
  if (abs(current) > 1e-6 && surfaceArea > 0) {
    float cellTempK = cellTemp + 273.15;
    float currentDensity = current / surfaceArea;
    float RT_nF = R_GAS * cellTempK / (N_ELECTRONS * F_FARADAY);
    float overpotential_act = RT_nF * asinh(currentDensity / (2 * j0));
    result.activationPolarizationHeating = abs(current) * abs(overpotential_act);
  }

  result.concentrationPolarizationHeating = abs(current) * abs(current) * R_conc;

  result.electrolysisHeating = 0.0;
  if (voltageUnloaded > U_electrolysis && current < 0) {
    result.electrolysisHeating = (voltageUnloaded - U_electrolysis) * abs(current) * k_electrolysis;
  }

  result.recombinationHeating = 0.0;
  if (result.stateOfCharge > 0.95 && current < 0) {
    result.recombinationHeating = abs(current) * k_recombination * (result.stateOfCharge - 0.95) * 20.0;
  }

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

  result.netHeating = result.resistiveHeating + result.activationPolarizationHeating + result.concentrationPolarizationHeating + result.electrolysisHeating + result.recombinationHeating -
                      result.convectionCooling - result.radiationCooling - result.conductionCooling;

  result.tempChangeRate = result.netHeating / thermalCapacity;

  return result;
}

/**
 * Function to perform various tests and calibrate the electrochemical model parameters.
 * This implementation provides a basic least-squares fit for the baseline internal resistance
 * and a conceptual iterative approach for polarization parameters.
 * Calibrating other parameters requires more advanced techniques.
 *
 * @param initialTemp       Initial cell temperature (°C)
 * @param ambientTemp       Ambient temperature (°C)
 * @param cellDiameter      Diameter of AAA cell (m)
 * @param cellLength        Length of AAA cell (m)
 * @param capacity_mAh      Nominal capacity of the cell in mAh
 * @param testData          Structure to hold measured voltage and temperature data for different tests
 * @param numDataPoints     Number of data points in the testData
 * @param timestep          Time interval for measurements (s)
 * @return                  A structure containing the calibrated parameters
 */
struct TestDataPoint {
  float time;
  float voltage_no_load;
  float voltage_load;
  float temperature;
  float current; // Applied current during the test
};

struct CalibrationParameters {
  float baselineInternalResistance;
  float alpha_a;
  float alpha_c;
  float j0;
  float R_conc;
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
  const TestDataPoint* testData,
  int numDataPoints,
  float timestep
) {
  CalibrationParameters params;
  params.baselineInternalResistance = 0.05; // Initial guess
  params.alpha_a = 0.5;
  params.alpha_c = 0.5;
  params.j0 = 0.1;
  params.R_conc = 0.001;
  params.U_electrolysis = 1.5;
  params.k_electrolysis = 0.1;
  params.k_recombination = 0.05;

  // --- 1. Internal Resistance Calibration (Basic Least Squares using Eigen) ---
  Eigen::MatrixXf A_ir(numDataPoints, 1);
  Eigen::VectorXf b_ir(numDataPoints);

  int ir_data_points = 0;
  for (int i = 0; i < numDataPoints; ++i) {
    if (abs(testData[i].current) > 0.01 && testData[i].voltage_no_load > testData[i].voltage_load) {
      float measured_resistance_raw = (testData[i].voltage_no_load - testData[i].voltage_load) / testData[i].current;
      float soc = estimateSOC(testData[i].voltage_no_load);
      float tempFactorVal = tempFactor(soc, testData[i].temperature);
      float chargeFactorVal = chargeFactor(soc);
      if (tempFactorVal > 0 && chargeFactorVal > 0) {
        A_ir(ir_data_points, 0) = tempFactorVal * chargeFactorVal;
        b_ir(ir_data_points) = measured_resistance_raw;
        ir_data_points++;
      }
    }
  }

  if (ir_data_points > 0) {
    A_ir.conservativeResize(ir_data_points, 1);
    b_ir.conservativeResize(ir_data_points);
    Eigen::VectorXf x_ir = (A_ir.transpose() * A_ir).ldlt().solve(A_ir.transpose() * b_ir);
    params.baselineInternalResistance = x_ir(0);
    Serial.print(F("Calibrated Baseline Internal Resistance: "));
    Serial.println(params.baselineInternalResistance);
  } else {
    Serial.println(F("Insufficient data for internal resistance calibration."));
  }

  // --- 2. Polarization Parameter Calibration (Simplified Iterative Approach) ---
  Serial.println(F("Starting Polarization Parameter Calibration (Simplified Iterative)."));
  float best_error = 1e10; // Initialize with a large error
  CalibrationParameters current_params = params;

  // Iterate through a small range of parameter values (this is a very basic search)
  for (float alpha = 0.3; alpha <= 0.7; alpha += 0.2) {
    for (float j_log = -3; j_log <= 1; j_log += 2) { // Iterate through orders of magnitude for j0
      float current_j0 = pow(10.0, j_log);
      current_params.alpha_a = alpha;
      current_params.alpha_c = alpha; // Assuming symmetric for simplicity
      current_params.j0 = current_j0;

      float total_error = 0;
      int polarization_data_points = 0;

      // Iterate through test data that might be suitable for polarization analysis
      for (int i = 0; i < numDataPoints; ++i) {
        if (abs(testData[i].current) > 0.1) { // Consider data with significant current
          float ocv = testData[i].voltage_no_load;
          float current_val = testData[i].current;
          float temp_val = testData[i].temperature;
          float surfaceArea = 2.0 * PI * (cellDiameter / 2.0) * (cellDiameter / 2.0) + 2.0 * PI * (cellDiameter / 2.0) * cellLength;
          if (surfaceArea > 0) {
            float currentDensity = current_val / surfaceArea;
            float RT_nF = R_GAS * (temp_val + 273.15) / (N_ELECTRONS * F_FARADAY);
            float overpotential_act = RT_nF * asinh(currentDensity / (2 * current_params.j0));
            float predicted_voltage_load = ocv - (current_val * params.baselineInternalResistance * tempFactor(estimateSOC(ocv), temp_val) * chargeFactor(estimateSOC(ocv))) - overpotential_act;
            total_error += pow(testData[i].voltage_load - predicted_voltage_load, 2);
            polarization_data_points++;
          }
        }
      }

      if (polarization_data_points > 0) {
        if (total_error < best_error) {
          best_error = total_error;
          params.alpha_a = current_params.alpha_a;
          params.alpha_c = current_params.alpha_c;
          params.j0 = current_params.j0;
          Serial.print(F("Improved Polarization Parameters - alpha: "));
          Serial.print(params.alpha_a);
          Serial.print(F(", j0: "));
          Serial.println(params.j0);
        }
      }
    }
  }
  Serial.println(F("Finished Polarization Parameter Calibration (Simplified)."));

  // --- 3. Concentration Polarization Resistance (Conceptual Outline) ---
  Serial.println(F("Concentration polarization resistance calibration (conceptual - needs implementation)."));
  // Analyze high current discharge data and fit R_conc.

  // --- 4. Electrolysis Threshold and Rate (Conceptual Outline) ---
  Serial.println(F("Electrolysis parameter calibration (conceptual - needs implementation)."));
  // Analyze constant voltage charging data above threshold.

  // --- 5. Recombination Heating Coefficient (Conceptual Outline) ---
  Serial.println(F("Recombination heating coefficient calibration (conceptual - needs implementation)."));
  // Analyze temperature data during overcharging.

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
  // ... (rest of the predictBatteryTemp function remains the same) ...
  float cellTemp = initialTemp;
  float timeElapsed = 0;
  float openCircuitVoltage = initialVoltage;
  float loadedVoltage = openCircuitVoltage - (current * internalRes);
  float lastLogTime = 0;

  float stateOfCharge = estimateSOC(openCircuitVoltage);

  float capacity = 800.0;

  Serial.println(F("Time(s),Temp(C),OCV(V),LoadedV(V),SOC,ResistiveHeat(W),ActivationHeat(W),ConcentrationHeat(W),ElectrolysisHeat(W),RecombinationHeat(W),NetHeat(W)"));

  while (timeElapsed < duration) {
    float ampHours = (current / 3600.0) * timestep;
    float socChange = ampHours / (capacity / 1000.0);

    if (current > 0) stateOfCharge -= socChange;
    else {
      float chargingEfficiency = 0.9;
      if (stateOfCharge > 0.8) chargingEfficiency = 0.9 - 0.4 * (stateOfCharge - 0.8) * 5.0;
      stateOfCharge += socChange * chargingEfficiency;
    }
    stateOfCharge = constrain(stateOfCharge, 0.0, 1.0);

    if (stateOfCharge <= 0.3) openCircuitVoltage = 1.0 + (stateOfCharge / 0.3) * 0.15;
    else if (stateOfCharge >= 0.7) openCircuitVoltage = 1.35 + ((stateOfCharge - 0.7) / 0.3) * 0.1;
    else openCircuitVoltage = 1.15 + ((stateOfCharge - 0.3) / 0.4) * 0.2;
    openCircuitVoltage = constrain(openCircuitVoltage, 1.0, 1.45);

    BatteryHeatingComponents heating = analyzeBatteryHeating(
      loadedVoltage,
      openCircuitVoltage,
      current,
      internalRes,
      ambientTemp,
      cellTemp,
      0.0,
      0.1,
      10.0,
      alpha_a,
      alpha_c,
      j0,
      R_conc,
      U_electrolysis,
      k_electrolysis,
      k_recombination
    );

    if (timeElapsed - lastLogTime >= logInterval || timeElapsed == 0) {
      Serial.print(timeElapsed);
      Serial.print(F(","));
      Serial.print(cellTemp);
      Serial.print(F(","));
      Serial.print(openCircuitVoltage);
      Serial.print(F(","));
      Serial.print(loadedVoltage);
      Serial.print(F(","));
      Serial.print(heating.stateOfCharge);
      Serial.print(F(","));
      Serial.print(heating.resistiveHeating);
      Serial.print(F(","));
      Serial.print(heating.activationPolarizationHeating);
      Serial.print(F(","));
      Serial.print(heating.concentrationPolarizationHeating);
      Serial.print(F(","));
      Serial.print(heating.electrolysisHeating);
      Serial.print(F(","));
      Serial.print(heating.recombinationHeating);
      Serial.print(F(","));
      Serial.println(heating.netHeating);

      lastLogTime = timeElapsed;
    }

    float deltaT = estimateBatteryTempDelta(
      loadedVoltage,
      openCircuitVoltage,
      current,
      internalRes,
      ambientTemp,
      cellTemp,
      0.0,
      0.1,
      10.0,
      0.0105,
      0.0445,
      timestep,
      alpha_a,
      alpha_c,
      j0,
      R_conc,
      U_electrolysis,
      k_electrolysis,
      k_recombination
    );

    cellTemp += deltaT;
    loadedVoltage = openCircuitVoltage - (current * internalRes);

    if (cellTemp > 80.0) {
      Serial.println(F("WARNING: Battery temperature exceeds 80°C - simulation terminated"));
      return 80.0;
    }
    if (stateOfCharge <= 0.0 && current > 0) {
      Serial.println(F("Battery depleted - simulation terminated"));
      break;
    }
    if (stateOfCharge >= 1.0 && current < 0) {
      Serial.println(F("Battery fully charged - simulation terminated"));
      break;
    }

    timeElapsed += timestep;
  }

  return cellTemp;
}

float calibrateInternalResistance(
  float voltageNoLoad,
  float voltageUnderLoad,
  float testCurrent,
  float cellTemp = 25.0
) {
  float voltageDrop = voltageNoLoad - voltageUnderLoad;
  float rawResistance = voltageDrop / testCurrent;
  float stateOfCharge = estimateSOC(voltageNoLoad);
  float tempFactorVal = tempFactor(stateOfCharge, cellTemp);
  float chargeFactorVal = chargeFactor(stateOfCharge);
  float baselineResistance = rawResistance / (tempFactorVal * chargeFactorVal);
  return baselineResistance;
}
