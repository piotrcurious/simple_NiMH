// Constants for battery charging
const float MAX_TEMP_DIFF_THRESHOLD = 0.8f;        // Maximum temperature difference in Celsius before stopping charge
const float MH_ELECTRODE_RATIO = 0.5f;             // Target ratio for MH electrode voltage (half value)
const int CHARGE_EVALUATION_INTERVAL_MS = 60000;   // Re-evaluate charging parameters every minute
const int CHARGE_CURRENT_STEP = 5;                 // Step size for PWM duty cycle adjustment
const int MAX_CHARGE_DUTY_CYCLE = 230;             // Maximum duty cycle for charging
const int MIN_CHARGE_DUTY_CYCLE = 20;              // Minimum duty cycle for charging

// Global variables for charging state
bool isCharging = false;
unsigned long lastChargeEvaluationTime = 0;
float mhElectrodeVoltage = 0.0f;
int chargingDutyCycle = 0;

// Structure to hold MH electrode voltage measurement data
struct MHElectrodeData {
  float unloadedVoltage;
  float loadedVoltage;
  float mhElectrodeRatio;
  float current;
};

/**
 * Measures the MH electrode voltage by comparing unloaded vs loaded voltages
 * Returns the data structure with measurements
 */
MHElectrodeData measureMHElectrodeVoltage(int testDutyCycle) {
  MHElectrodeData data;
  
  // Measure unloaded voltage
  MeasurementData unloadedData = getUnloadedVoltageMeasurement();
  data.unloadedVoltage = unloadedData.voltage;
  
  // Measure loaded voltage with the specified duty cycle
  MeasurementData loadedData = takeMeasurement(testDutyCycle, STABILIZATION_DELAY_MS);
  data.loadedVoltage = loadedData.voltage;
  data.current = loadedData.current;
  
  // Calculate the MH electrode voltage ratio (loaded/unloaded)
  data.mhElectrodeRatio = data.loadedVoltage / data.unloadedVoltage;
  
  return data;
}

/**
 * Finds the optimal charging duty cycle that maintains the MH electrode voltage
 * at the target ratio (half value)
 */
int findOptimalChargingDutyCycle() {
  Serial.println("Finding optimal charging duty cycle...");
  
  int optimalDutyCycle = MIN_CHARGE_DUTY_CYCLE;
  float closestRatio = 1.0f;  // Start with unloaded ratio (1.0)
  float targetRatio = MH_ELECTRODE_RATIO;
  
  // Start with a binary search approach to narrow down the range
  int lowDC = MIN_CHARGE_DUTY_CYCLE;
  int highDC = MAX_CHARGE_DUTY_CYCLE;
  
  while (highDC - lowDC > CHARGE_CURRENT_STEP * 2) {
    int midDC = (lowDC + highDC) / 2;
    
    MHElectrodeData data = measureMHElectrodeVoltage(midDC);
    Serial.printf("Duty Cycle: %d, Unloaded: %.3fV, Loaded: %.3fV, Ratio: %.3f, Current: %.3fA\n", 
                  midDC, data.unloadedVoltage, data.loadedVoltage, data.mhElectrodeRatio, data.current);
    
    if (data.mhElectrodeRatio < targetRatio) {
      // Too much load, reduce duty cycle
      highDC = midDC;
    } else {
      // Too little load, increase duty cycle
      lowDC = midDC;
    }
    
    float ratioDiff = abs(data.mhElectrodeRatio - targetRatio);
    if (ratioDiff < abs(closestRatio - targetRatio)) {
      closestRatio = data.mhElectrodeRatio;
      optimalDutyCycle = midDC;
    }
  }
  
  // Fine tune with small steps around the approximate value
  for (int dc = max(MIN_CHARGE_DUTY_CYCLE, optimalDutyCycle - CHARGE_CURRENT_STEP * 3); 
       dc <= min(MAX_CHARGE_DUTY_CYCLE, optimalDutyCycle + CHARGE_CURRENT_STEP * 3); 
       dc += CHARGE_CURRENT_STEP) {
    
    MHElectrodeData data = measureMHElectrodeVoltage(dc);
    Serial.printf("Fine-tuning - Duty Cycle: %d, Ratio: %.3f, Current: %.3fA\n", 
                  dc, data.mhElectrodeRatio, data.current);
    
    float ratioDiff = abs(data.mhElectrodeRatio - targetRatio);
    if (ratioDiff < abs(closestRatio - targetRatio)) {
      closestRatio = data.mhElectrodeRatio;
      optimalDutyCycle = dc;
    }
  }
  
  Serial.printf("Optimal charging duty cycle found: %d (ratio: %.3f)\n", 
                optimalDutyCycle, closestRatio);
  
  return optimalDutyCycle;
}

/**
 * Main charging function - starts, monitors, and stops battery charging
 * Returns true if charging is in progress, false if charging has stopped
 */
bool chargeBattery() {
  // If not currently charging, start the charging process
  if (!isCharging) {
    Serial.println("Starting battery charging process...");
    isCharging = true;
    lastChargeEvaluationTime = millis();
    
    // Find the initial optimal charging duty cycle
    chargingDutyCycle = findOptimalChargingDutyCycle();
    
    // Apply the charging current
    dutyCycle = chargingDutyCycle;
    analogWrite(pwmPin, chargingDutyCycle);
    
    // Store initial data for monitoring
    MeasurementData initialData;
    getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff, 
                         initialData.t1_millivolts, initialData.voltage, initialData.current);
    
    Serial.printf("Charging started - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                  chargingDutyCycle, initialData.current, initialData.temp1, initialData.temp2, initialData.tempDiff);
    
    return true;
  }
  
  // Get current temperature and voltage readings
  float temp1, temp2, tempDiff, t1_millivolts, voltage, current;
  getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
  
  // Check if temperature difference exceeds threshold - stop charging if it does
  if (tempDiff > MAX_TEMP_DIFF_THRESHOLD) {
    Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C), stopping charging\n", 
                  tempDiff, MAX_TEMP_DIFF_THRESHOLD);
    
    // Stop charging
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    isCharging = false;
    
    return false;
  }
  
  // Periodically re-evaluate the MH electrode voltage and adjust charging current
  unsigned long currentTime = millis();
  if (currentTime - lastChargeEvaluationTime > CHARGE_EVALUATION_INTERVAL_MS) {
    Serial.println("Re-evaluating charging parameters...");
    
    // Temporarily pause charging to measure unloaded voltage
    analogWrite(pwmPin, 0);
    delay(UNLOADED_VOLTAGE_DELAY_MS);
    
    // Find the optimal charging duty cycle again
    chargingDutyCycle = findOptimalChargingDutyCycle();
    
    // Apply the updated charging current
    dutyCycle = chargingDutyCycle;
    analogWrite(pwmPin, chargingDutyCycle);
    
    lastChargeEvaluationTime = currentTime;
    
    Serial.printf("Charging parameters updated - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                  chargingDutyCycle, current, temp1, temp2, tempDiff);
  }
  
  // Display charging status on TFT
  tft.setCursor(5, PLOT_Y_START + PLOT_HEIGHT + 20);
  tft.printf("CHARGING - DC: %d, I: %.2fA", chargingDutyCycle, current);
  tft.setCursor(5, PLOT_Y_START + PLOT_HEIGHT + 40);
  tft.printf("T1: %.2fC, T2: %.2fC, dT: %.2fC", temp1, temp2, tempDiff);
  
  return true; // Charging is still in progress
}

/**
 * Start charging with a single call
 */
void startCharging() {
  if (!isCharging) {
    Serial.println("Initiating battery charging...");
    isCharging = true;
    chargeBattery();
  } else {
    Serial.println("Charging already in progress");
  }
}

/**
 * Stop charging with a single call
 */
void stopCharging() {
  if (isCharging) {
    Serial.println("Manually stopping charging");
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    isCharging = false;
    
    tft.setCursor(5, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("CHARGING STOPPED                  ");
  }
}

/**
 * Loop function to be called in the main loop to monitor and maintain charging
 */
void handleBatteryCharging() {
  if (isCharging) {
    // Update charging status and check if it's still charging
    if (!chargeBattery()) {
      // Charging has stopped
      tft.setCursor(5, PLOT_Y_START + PLOT_HEIGHT + 20);
      tft.printf("CHARGING COMPLETE                ");
    }
  }
}
