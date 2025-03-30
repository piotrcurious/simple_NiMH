#include <Arduino.h>

// Pin definitions
#define PWM_PIN  5  
#define ANALOG_VOLT_PIN A0  // Analog pin for direct voltage measurement
#define ANALOG_CURR_PIN A1  // Analog pin for current measurement

// Moving average filter size for measurements
#define FILTER_SIZE 8

// Global measured battery values (updated by a separate system task every 500ms)
extern float batteryVoltage;  // Battery voltage (V)
extern float batteryCurrent;  // Battery current (A)

// Electrochemical constants 
const float F = 96485.33212;   // Faraday's constant (C/mol)
const float R = 8.3145;        // Universal gas constant (J/(mol*K))
const float V_H2_EVOL = -0.83; // Standard hydrogen evolution potential (V)
const float V_O2_EVOL = 0.40;  // Standard oxygen evolution potential (V)
const float alpha = 0.5;       // Charge transfer coefficient
const float n_H2 = 2.0;        // Number of electrons for hydrogen evolution
const float n_O2 = 4.0;        // Number of electrons for oxygen evolution
const float E_overpot = 0.1;   // Overpotential threshold for electrolysis onset (V)

// Calibratable parameters with default values
struct ElectrolysisParams {
  float A_CATHODE;       // Effective cathode surface area (m^2)
  float A_NODE;          // Effective anode surface area (m^2)
  float i0_H2;           // Exchange current density for H2 evolution (A/m^2)
  float i0_O2;           // Exchange current density for O2 evolution (A/m^2)
  float cellResistance;  // Internal cell resistance (ohms)
  float timeConstant;    // RC time constant of the cell (ms)
  float temperatureCoef; // Temperature coefficient for electrolysis voltage (V/°C)
};

// Initialize with reasonable defaults
ElectrolysisParams params = {
  .A_CATHODE = 1.5e-4,      // m^2
  .A_NODE = 1.2e-4,         // m^2
  .i0_H2 = 1e-6,            // A/m^2
  .i0_O2 = 2e-6,            // A/m^2
  .cellResistance = 0.05,   // ohms
  .timeConstant = 250.0,    // ms
  .temperatureCoef = 0.001  // V/°C
};

// Circular buffer for voltage/current moving average
float voltageBuffer[FILTER_SIZE] = {0};
float currentBuffer[FILTER_SIZE] = {0};
int bufferIndex = 0;

// EEPROM addresses for parameter storage
#define EEPROM_VALID_FLAG 0
#define EEPROM_PARAMS_ADDR 4
#define EEPROM_VALID_VALUE 0xABCD1234

//--------------------------------------------------------------
// getFilteredMeasurements()
// 
// Take multiple measurements and apply filtering to reduce noise
//--------------------------------------------------------------
void getFilteredMeasurements(float &voltage, float &current) {
  // Add latest measurements to circular buffer
  voltageBuffer[bufferIndex] = batteryVoltage;
  currentBuffer[bufferIndex] = batteryCurrent;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
  
  // Calculate moving average
  voltage = 0;
  current = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    voltage += voltageBuffer[i];
    current += currentBuffer[i];
  }
  voltage /= FILTER_SIZE;
  current /= FILTER_SIZE;
}

//--------------------------------------------------------------
// applyTemperatureCompensation()
// 
// Adjust thresholds based on temperature effects on electrochemical potentials
//--------------------------------------------------------------
void applyTemperatureCompensation(float &V_H2_thresh, float &V_O2_thresh, float T_kelvin) {
  // Nernst equation temperature compensation
  float T_std = 298.15; // Standard temperature (K)
  
  // Temperature factor for Nernst equation
  float T_factor = (T_kelvin / T_std);
  
  // Apply temperature compensation to threshold voltages
  V_H2_thresh = V_H2_EVOL * T_factor + E_overpot;
  V_O2_thresh = V_O2_EVOL * T_factor + E_overpot;
  
  // Apply additional linear temperature coefficient
  float T_celsius = T_kelvin - 273.15;
  float T_diff = T_celsius - 25.0; // Difference from 25°C
  V_H2_thresh += params.temperatureCoef * T_diff;
  V_O2_thresh += params.temperatureCoef * T_diff;
}

//--------------------------------------------------------------
// computeMaxElectrolysisCurrent()
// 
// Uses an electrochemical model (Nernst and Butler-Volmer equations)
// with temperature compensation and dynamic response adjustment
//--------------------------------------------------------------
float computeMaxElectrolysisCurrent(float internalResistance, float idleVoltage, float batteryTemperature) {
  float T = batteryTemperature + 273.15;  // Convert to Kelvin

  // Get temperature-compensated threshold potentials
  float V_H2_thresh, V_O2_thresh;
  applyTemperatureCompensation(V_H2_thresh, V_O2_thresh, T);

  // Predicted cell voltage for simultaneous H2 and O2 evolution
  float electrolysisVoltage = fabs(V_H2_thresh) + fabs(V_O2_thresh);

  // Get filtered measurements for more stable readings
  float V_measured, I_measured;
  getFilteredMeasurements(V_measured, I_measured);

  // Apply the Butler-Volmer equation with more accurate implementation
  // Using hyperbolic sine for better numerical stability
  float eta_H2 = V_H2_thresh;
  float eta_O2 = V_O2_thresh;
  
  float RT_F = R * T / F;
  
  // More stable implementation of Butler-Volmer using sinh
  float i_H2 = 2 * params.i0_H2 * sinh(n_H2 * alpha * eta_H2 / RT_F);
  float i_O2 = 2 * params.i0_O2 * sinh(n_O2 * alpha * eta_O2 / RT_F);

  // Total electrolysis current contributions from both electrodes
  float I_H2 = i_H2 * params.A_CATHODE;
  float I_O2 = i_O2 * params.A_NODE;
  float I_electrolysis = I_H2 + I_O2;

  // Incorporate the cell's RC time constant for dynamic response adjustment
  float dynamicFactor = 1.0;
  if (params.timeConstant > 0) {
    // Adjust based on the ratio of measurement time to time constant
    // This accounts for the fact that short pulses may not trigger electrolysis
    // as effectively as sustained current flow
    float measurementTime = 1000.0; // Typical measurement time in ms
    dynamicFactor = 1.0 - exp(-measurementTime / params.timeConstant);
    I_electrolysis /= dynamicFactor;
  }

  // Also compute maximum current based on internal resistance limitations
  float V_drop = electrolysisVoltage - idleVoltage;
  float I_max_resistance = V_drop / internalResistance;
  
  // Choose the lower of the two limits for safety
  float I_max = min(I_max_resistance, I_electrolysis);

  // Ensure we don't return negative values due to numerical issues
  return max(I_max, 0.0);
}

//--------------------------------------------------------------
// performStepResponse()
// 
// Improved dynamic response measurement using PWM pulse
//--------------------------------------------------------------
struct DynamicResponse {
  float timeConstant;      // Inferred time constant (ms)
  float voltageStep;       // Average voltage step response (V)
  float currentStep;       // Average current step response (A)
  float steadyStateR;      // Steady-state resistance (ohms)
};

DynamicResponse performStepResponse() {
  DynamicResponse resp;
  const int maxSamples = 20;
  const int sampleInterval = 50; // ms
  float voltageData[maxSamples];
  float currentData[maxSamples];
  unsigned long times[maxSamples];
  
  // Ensure PWM pin is configured as output
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  
  // Measure baseline idle values with sufficient settling time
  delay(500);
  float baselineVoltage, baselineCurrent;
  getFilteredMeasurements(baselineVoltage, baselineCurrent);
  
  // Apply step input (full PWM)
  analogWrite(PWM_PIN, 255);
  
  // Sample response over time
  unsigned long startTime = millis();
  for (int i = 0; i < maxSamples; i++) {
    // Record precise timing
    times[i] = millis() - startTime;
    
    // Sample voltage and current
    voltageData[i] = batteryVoltage;
    currentData[i] = batteryCurrent;
    
    // Wait for next sample
    delay(sampleInterval);
  }
  
  // End the pulse
  analogWrite(PWM_PIN, 0);
  
  // Calculate steady-state values (average of last few samples)
  float steadyVoltage = 0;
  float steadyCurrent = 0;
  int steadySamples = 3;
  for (int i = maxSamples - steadySamples; i < maxSamples; i++) {
    steadyVoltage += voltageData[i];
    steadyCurrent += currentData[i];
  }
  steadyVoltage /= steadySamples;
  steadyCurrent /= steadySamples;
  
  // Calculate delta V and delta I
  resp.voltageStep = steadyVoltage - baselineVoltage;
  resp.currentStep = steadyCurrent - baselineCurrent;
  
  // Calculate steady state resistance
  resp.steadyStateR = fabs(resp.voltageStep) / fabs(resp.currentStep);
  
  // Find time constant using curve fitting
  // Time at which response reaches ~63% of final value
  float targetResponse = baselineVoltage + 0.63 * resp.voltageStep;
  float timeConstant = 0;
  
  // Search for the time when voltage first crosses 63% threshold
  for (int i = 1; i < maxSamples; i++) {
    if ((resp.voltageStep > 0 && voltageData[i] >= targetResponse) ||
        (resp.voltageStep < 0 && voltageData[i] <= targetResponse)) {
      // Linear interpolation between samples for more accuracy
      float t1 = times[i-1];
      float t2 = times[i];
      float v1 = voltageData[i-1];
      float v2 = voltageData[i];
      timeConstant = t1 + (t2 - t1) * (targetResponse - v1) / (v2 - v1);
      break;
    }
  }
  
  resp.timeConstant = timeConstant;
  return resp;
}

//--------------------------------------------------------------
// calibrateBatteryModel()
// 
// Improved model calibration with state estimation and parameter optimization
//--------------------------------------------------------------
void calibrateBatteryModel(float idleVoltage, float batteryTemperature) {
  Serial.println("Beginning battery model calibration...");
  
  // First, measure dynamic response
  DynamicResponse dynResp = performStepResponse();
  
  // Update time constant and resistance parameters
  params.timeConstant = dynResp.timeConstant;
  params.cellResistance = dynResp.steadyStateR;
  
  // Next, perform a steady-state test for electrolysis parameters
  float T = batteryTemperature + 273.15;  // in Kelvin
  
  // Get temperature-compensated threshold potentials
  float V_H2_thresh, V_O2_thresh;
  applyTemperatureCompensation(V_H2_thresh, V_O2_thresh, T);
  
  // Apply a longer calibration pulse with 50% PWM
  analogWrite(PWM_PIN, 128);
  delay(3000);  // Longer stabilization period
  
  // Get measurements
  float V_cal, I_cal;
  getFilteredMeasurements(V_cal, I_cal);
  analogWrite(PWM_PIN, 0);
  
  // Calculate the expected voltage drop based on IR
  float V_IR = I_cal * params.cellResistance;
  
  // Estimate the actual electrode potential difference
  float V_electrodes = idleVoltage - V_cal - V_IR;
  
  // If electrolysis is happening, use measurements to update model parameters
  if (V_electrodes > fabs(V_H2_thresh) + fabs(V_O2_thresh)) {
    // Compute the steady-state model prediction
    float RT_F = R * T / F;
    
    float i_H2 = 2 * params.i0_H2 * sinh(n_H2 * alpha * V_H2_thresh / RT_F);
    float i_O2 = 2 * params.i0_O2 * sinh(n_O2 * alpha * V_O2_thresh / RT_F);
    float I_model = i_H2 * params.A_CATHODE + i_O2 * params.A_NODE;
    
    // Calculate a correction factor based on the measured vs. predicted current
    float correctionFactor = I_cal / I_model;
    
    // Limit the correction factor to avoid extreme adjustments
    correctionFactor = constrain(correctionFactor, 0.5, 2.0);
    
    // Apply the correction factor to the effective electrode areas
    params.A_CATHODE *= correctionFactor;
    params.A_NODE *= correctionFactor;
    
    Serial.print("Applied area correction factor: ");
    Serial.println(correctionFactor, 4);
  }
  else {
    Serial.println("No significant electrolysis detected during calibration");
  }
  
  // Apply a temperature calibration if temperature differs from reference
  if (fabs(batteryTemperature - 25.0) > 5.0) {
    // Perform the test at a higher current to estimate temperature coefficient
    analogWrite(PWM_PIN, 200);
    delay(2000);
    
    float V_temp, I_temp;
    getFilteredMeasurements(V_temp, I_temp);
    analogWrite(PWM_PIN, 0);
    
    // Calculate temperature coefficient
    float V_drop_measured = idleVoltage - V_temp;
    float V_drop_IR = I_temp * params.cellResistance;
    float V_drop_electrochem = V_drop_measured - V_drop_IR;
    
    // Adjust temperature coefficient (simple linear model)
    float T_diff = batteryTemperature - 25.0;
    if (fabs(T_diff) > 0.1) {  // Avoid division by zero
      params.temperatureCoef = V_drop_electrochem / T_diff;
      
      // Limit to reasonable values
      params.temperatureCoef = constrain(params.temperatureCoef, 0.0005, 0.005);
    }
  }
  
  // Output calibration results
  Serial.println("Calibration complete with results:");
  Serial.print("Time constant (ms): ");
  Serial.println(params.timeConstant, 2);
  Serial.print("Cell resistance (ohms): ");
  Serial.println(params.cellResistance, 4);
  Serial.print("A_CATHODE (m^2): ");
  Serial.println(params.A_CATHODE, 6);
  Serial.print("A_NODE (m^2): ");
  Serial.println(params.A_NODE, 6);
  Serial.print("Temperature coefficient (V/°C): ");
  Serial.println(params.temperatureCoef, 6);
  
  // Save parameters to EEPROM
  saveParamsToEEPROM();
}

//--------------------------------------------------------------
// EEPROM Parameter Management
//--------------------------------------------------------------
#include <EEPROM.h>

void saveParamsToEEPROM() {
  // Write validation flag
  uint32_t validFlag = EEPROM_VALID_VALUE;
  EEPROM.put(EEPROM_VALID_FLAG, validFlag);
  
  // Write parameters
  EEPROM.put(EEPROM_PARAMS_ADDR, params);
  
  Serial.println("Parameters saved to EEPROM");
}

boolean loadParamsFromEEPROM() {
  // Check validation flag
  uint32_t validFlag;
  EEPROM.get(EEPROM_VALID_FLAG, validFlag);
  
  if (validFlag != EEPROM_VALID_VALUE) {
    Serial.println("No valid parameters in EEPROM");
    return false;
  }
  
  // Read parameters
  EEPROM.get(EEPROM_PARAMS_ADDR, params);
  
  Serial.println("Parameters loaded from EEPROM");
  return true;
}

//--------------------------------------------------------------
// Example setup and loop routines
//--------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ANALOG_VOLT_PIN, INPUT);
  pinMode(ANALOG_CURR_PIN, INPUT);
  
  // Initialize PWM frequency for smoother control
  // On most Arduino boards, pin 5 is on timer0
  // Set to 31372.55 Hz (no prescaling)
  // TCCR0B = (TCCR0B & 0b11111000) | 0x01;
  
  // Try to load parameters from EEPROM
  if (!loadParamsFromEEPROM()) {
    // If no saved parameters, run calibration
    // Use the current battery voltage as idle voltage
    calibrateBatteryModel(batteryVoltage, 25.0);
  }
  
  Serial.println("Battery electrolysis monitor initialized");
}

// State machine states for better control flow
enum SystemState {
  STATE_IDLE,
  STATE_MEASURING,
  STATE_CALIBRATING
};

SystemState currentState = STATE_IDLE;
unsigned long lastMeasurementTime = 0;
unsigned long lastCalibrationTime = 0;
const unsigned long MEASUREMENT_INTERVAL = 5000;    // 5 seconds
const unsigned long CALIBRATION_INTERVAL = 3600000; // 1 hour

void loop() {
  unsigned long currentTime = millis();
  
  // State machine for system operation
  switch (currentState) {
    case STATE_IDLE:
      // Check if it's time for a measurement
      if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
        currentState = STATE_MEASURING;
      }
      // Check if it's time for calibration
      else if (currentTime - lastCalibrationTime >= CALIBRATION_INTERVAL) {
        currentState = STATE_CALIBRATING;
      }
      break;
      
    case STATE_MEASURING:
      {
        // Get latest filtered measurements
        float V_measured, I_measured;
        getFilteredMeasurements(V_measured, I_measured);
        
        // Compute the maximum electrolysis current
        float batteryTempC = 25.0;  // Replace with actual temperature sensor reading
        float I_max = computeMaxElectrolysisCurrent(params.cellResistance, V_measured, batteryTempC);
        
        Serial.print("Battery Voltage: ");
        Serial.print(V_measured, 3);
        Serial.print("V, Current: ");
        Serial.print(I_measured, 3);
        Serial.print("A, Max safe current: ");
        Serial.print(I_max, 3);
        Serial.println("A");
        
        // If we're approaching the electrolysis threshold, issue a warning
        if (I_measured > I_max * 0.9) {
          Serial.println("WARNING: Current approaching electrolysis threshold!");
        }
        
        lastMeasurementTime = currentTime;
        currentState = STATE_IDLE;
      }
      break;
      
    case STATE_CALIBRATING:
      {
        // Get a stable idle voltage reading
        float V_idle, I_idle;
        getFilteredMeasurements(V_idle, I_idle);
        
        // Only calibrate if the system is truly idle
        if (I_idle < 0.01) {  // Less than 10mA
          Serial.println("Starting scheduled calibration...");
          float batteryTempC = 25.0;  // Replace with actual temperature sensor
          calibrateBatteryModel(V_idle, batteryTempC);
        } else {
          Serial.println("Skipping calibration - system not idle");
        }
        
        lastCalibrationTime = currentTime;
        currentState = STATE_IDLE;
      }
      break;
  }
  
  // Process any serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      Serial.println("Manual calibration requested");
      currentState = STATE_CALIBRATING;
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}
