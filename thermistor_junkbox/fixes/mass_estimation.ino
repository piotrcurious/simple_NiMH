#include <Arduino.h>
#include <Eigen.h>       // Make sure to include your Arduino Eigen library header
#include <Eigen/Dense>   // Depending on your library installation

// Helper function to perform a 3rd order polynomial fit and return the coefficients.
// It assumes time and temperature arrays of length 'n'.
// The coefficients are returned in an Eigen::Vector4f, where
// coeffs[0] = a0, coeffs[1] = a1, coeffs[2] = a2, coeffs[3] = a3.
Eigen::Vector4f polyFit3rdOrder(const float* tData, const float* tempData, int n) {
  // Create an n x 4 design matrix X and n x 1 vector Y.
  Eigen::MatrixXf X(n, 4);
  Eigen::VectorXf Y(n);
  
  for (int i = 0; i < n; i++) {
    float t = tData[i];
    X(i, 0) = 1.0;
    X(i, 1) = t;
    X(i, 2) = t * t;
    X(i, 3) = t * t * t;
    Y(i) = tempData[i];
  }
  
  // Compute the least-squares solution: coeffs = (X^T*X)^{-1} * X^T * Y.
  Eigen::Vector4f coeffs = (X.transpose() * X).inverse() * (X.transpose() * Y);
  return coeffs;
}

// Main helper function to estimate the battery weight (mass in kg)
// using measured temperature vs. time data, current, and internal resistance.
// 'tData' and 'tempData' are arrays of measurements of time (in seconds) and temperature (in °C).
// 'n' is the number of samples. 'current' (in A) and 'internalResistance' (in ohms) are used to compute power.
// 'specificHeat' is the assumed specific heat capacity of the battery (J/kg·K).
// The function returns the estimated mass in kg, or -1 if the temperature rise has essentially stopped.
float estimateBatteryMass(const float* tData, const float* tempData, int n,
                          float current, float internalResistance, float specificHeat) {
  // Fit a 3rd order polynomial to the temperature data.
  Eigen::Vector4f coeffs = polyFit3rdOrder(tData, tempData, n);
  
  // Evaluate the derivative dT/dt at the latest time sample (you might choose a different evaluation time).
  float t_eval = tData[n - 1];
  float dTdt = coeffs[1] + 2.0f * coeffs[2] * t_eval + 3.0f * coeffs[3] * t_eval * t_eval;
  
  // Determine if temperature rise has stopped by checking if the derivative is very small.
  const float derivativeThreshold = 0.01; // °C/s; adjust as needed
  if (fabs(dTdt) < derivativeThreshold) {
    // Heating has essentially stopped, so a dynamic estimation is no longer possible.
    return -1.0f;
  }
  
  // Calculate the power dissipated (W) by the battery internal resistance.
  float power = current * current * internalResistance;
  
  // Estimate the battery mass (in kg) using: m = P / (c * (dT/dt))
  float mass = power / (specificHeat * dTdt);
  
  return mass;
}

/*
Example usage:

Suppose you have collected temperature data at different time intervals in arrays tData[] and tempData[].
Assume you apply a current of 1.0 A through a battery with an internal resistance of 0.1 ohm,
and you assume a specific heat capacity of 1000 J/(kg·K) (this value may need calibration).

The function will return the battery mass (in kg) if the temperature is still rising,
or -1 if the temperature rise has effectively stopped.
*/
void setup() {
  Serial.begin(9600);
  
  // Example data (for demonstration):
  const int numSamples = 10;
  float tData[numSamples] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};          // in seconds
  float tempData[numSamples] = {25, 25.5, 26.1, 26.6, 27.0, 27.3, 27.5, 27.6, 27.65, 27.66}; // in °C
  
  // Battery test parameters:
  float current = 1.0;             // in A
  float internalResistance = 0.1;  // in ohms
  float specificHeat = 1000.0;       // J/(kg·K) - assumed value
  
  // Estimate battery mass:
  float batteryMass = estimateBatteryMass(tData, tempData, numSamples, current, internalResistance, specificHeat);
  
  if (batteryMass < 0) {
    Serial.println("Temperature rise has stopped; unable to estimate battery mass dynamically.");
  } else {
    Serial.print("Estimated battery mass: ");
    Serial.print(batteryMass, 4);
    Serial.println(" kg");
  }
}

void loop() {
  // The estimation can be updated continuously as new data is collected.
}
