#include <Arduino.h>
#include <TFT_eSPI.h>       // Graphics library for the TFT display
#include <math.h>
#include <ArduinoEigen.h>   // Include the ArduinoEigen library for matrix computations

using namespace Eigen;

// ===== Circuit/Hardware Configuration =====
// Transistor base is driven by tonePin (using a LEDC channel for variable-frequency output)
const int tonePin = 25;       // Output pin to control the transistor base

// ADC channels for voltage measurements
const int adcBattPin = 34;    // ADC pin measuring battery negative terminal voltage
const int adcResPin  = 35;    // ADC pin measuring voltage across the 2.5Ω resistor

// Sampling parameters for frequency sweep
const int numSweepPoints = 50;   // Number of frequency points in the sweep
float freqPoints[numSweepPoints];  // Frequencies (Hz) for the sweep
float Vbatt[numSweepPoints];       // Measured battery negative terminal voltage at each frequency
float Vres[numSweepPoints];        // Measured voltage across resistor at each frequency
float Zmeasured[numSweepPoints];   // "Impedance" computed as V_batt/(I), with I = V_res/2.5

// TFT display instance (using TFT_eSPI library)
TFT_eSPI tft = TFT_eSPI();

// ===== Setup Functions =====
void setup() {
  Serial.begin(115200);
  
  // Initialize TFT display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  
  // Configure LEDC (ESP32 PWM) for tone generation on tonePin.
  // Channel 0 is used. (Frequency will be updated during the sweep.)
  ledcSetup(0, 1000, 8);  // initial frequency 1kHz, 8-bit resolution
  ledcAttachPin(tonePin, 0);
  
  // Set ADC resolution – ESP32 ADC by default is 12-bit.
  analogReadResolution(12);
  
  // Prepare sweep frequencies – logarithmically spaced between 100Hz and 40kHz.
  float startFreq = 100.0;
  float endFreq   = 40000.0;
  for (int i = 0; i < numSweepPoints; i++) {
    float ratio = (float)i / (numSweepPoints - 1);
    // logarithmic spacing: f = f_start * (f_end/f_start)^ratio
    freqPoints[i] = startFreq * pow((endFreq / startFreq), ratio);
  }
}

void loop() {
  performFrequencySweep();
  plotTransferFunction();
  inferRLParametersCurveFit();
  
  // Stop further execution after one sweep.
  while (true) {
    delay(1000);
  }
}

// ===== Sweep and Measurement Functions =====
void performFrequencySweep() {
  const int numSamples = 50;    // Number of ADC samples to average at each frequency point
  for (int i = 0; i < numSweepPoints; i++) {
    // Update LEDC tone frequency to excite the circuit.
    uint32_t currentFreq = (uint32_t)freqPoints[i];
    ledcWriteTone(0, currentFreq);
    
    // Wait for the signal to stabilize.
    delay(100);
    
    // Take multiple ADC readings on both channels.
    long sumBatt = 0;
    long sumRes  = 0;
    for (int j = 0; j < numSamples; j++) {
      sumBatt += analogRead(adcBattPin);
      sumRes  += analogRead(adcResPin);
      delay(2);
    }
    float avgBatt = sumBatt / (float)numSamples;
    float avgRes  = sumRes  / (float)numSamples;
    
    // Convert ADC values (assuming 3.3V reference and 12-bit ADC)
    float voltageBatt = avgBatt * (3.3 / 4095.0);
    float voltageRes  = avgRes * (3.3 / 4095.0);
    
    Vbatt[i] = voltageBatt;
    Vres[i]  = voltageRes;
    
    // Compute the DC current through the resistor: I = V / R, where R = 2.5Ω.
    float current = voltageRes / 2.5;
    
    // Compute an effective battery impedance as V_batt / I (avoid division by very small currents)
    if (fabs(current) > 1e-4)
      Zmeasured[i] = voltageBatt / current;
    else
      Zmeasured[i] = 0;
      
    // Log the measurement for this frequency point.
    Serial.print("Freq: ");
    Serial.print(freqPoints[i]);
    Serial.print(" Hz, V_batt: ");
    Serial.print(voltageBatt, 3);
    Serial.print(" V, V_res: ");
    Serial.print(voltageRes, 3);
    Serial.print(" V, Z: ");
    Serial.print(Zmeasured[i], 3);
    Serial.println(" Ohm");
  }
  // Turn off tone generation after sweep.
  ledcWriteTone(0, 0);
}

// ===== Display Function =====
void plotTransferFunction() {
  // Clear screen and draw axes.
  tft.fillScreen(TFT_BLACK);
  tft.drawLine(20, 220, 300, 220, TFT_WHITE);   // x-axis
  tft.drawLine(20, 20, 20, 220, TFT_WHITE);       // y-axis
  
  // Autoscale: find the maximum measured impedance.
  float maxZ = 0;
  for (int i = 0; i < numSweepPoints; i++) {
    if (Zmeasured[i] > maxZ)
      maxZ = Zmeasured[i];
  }
  
  // Plot the transfer function (impedance vs. frequency).
  // x-axis: logarithmic scale from 100 Hz to 40 kHz; y-axis: scaled to maxZ.
  for (int i = 0; i < numSweepPoints; i++) {
    // Map logarithm of frequency from log10(100) to log10(40000) to x = 20 to 300.
    float logFreq  = log10(freqPoints[i]);
    float x        = 20 + (logFreq - log10(100)) / (log10(40000) - log10(100)) * (300 - 20);
    // Map impedance magnitude to y (inverted: larger Z → higher on graph).
    float y        = 220 - (Zmeasured[i] / maxZ) * (220 - 20);
    tft.fillCircle(x, y, 2, TFT_GREEN);
    
    // Draw connecting lines between consecutive points.
    if (i > 0) {
      float prevLogFreq = log10(freqPoints[i - 1]);
      float x0          = 20 + (prevLogFreq - log10(100)) / (log10(40000) - log10(100)) * (300 - 20);
      float y0          = 220 - (Zmeasured[i - 1] / maxZ) * (220 - 20);
      tft.drawLine(x0, y0, x, y, TFT_GREEN);
    }
  }
  
  // Display title.
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Transfer Function", 40, 5, 2);
}

// ===== RL Parameter Inference via Curve Fitting =====
// This function uses ArduinoEigen to perform a least-squares fit of the squared impedance data
// to the linearized model:  Z^2 = R^2 + (2πL)^2 * f^2.
// The design matrix has two columns: a constant term and f^2.
// From the fitted coefficients, we extract:
//    R_est = sqrt( intercept )
//    L_est = sqrt( slope ) / (2π)
void inferRLParametersCurveFit() {
  int N = numSweepPoints;
  
  // Create a design matrix X (N x 2) and measurement vector Y (N x 1)
  MatrixXf X(N, 2);
  VectorXf Y(N);
  
  for (int i = 0; i < N; i++) {
    float f = freqPoints[i];
    X(i, 0) = 1.0;       // constant term
    X(i, 1) = f * f;     // f^2 term
    Y(i) = Zmeasured[i] * Zmeasured[i];  // squared measured impedance
  }
  
  // Compute the least-squares solution: beta = (X^T * X)^{-1} * X^T * Y.
  MatrixXf Xt = X.transpose();
  MatrixXf beta = (Xt * X).inverse() * Xt * Y;
  
  // beta(0) is the intercept = R^2, beta(1) is the slope = (2πL)^2.
  float R_est = sqrt(beta(0));
  float L_est = sqrt(beta(1)) / (2 * PI);
  
  Serial.println("----- Curve Fitting Inferred RL Parameters -----");
  Serial.print("Estimated Internal Resistance: ");
  Serial.print(R_est, 2);
  Serial.println(" Ohm");
  Serial.print("Estimated Internal Inductance: ");
  Serial.print(L_est * 1e3, 2);
  Serial.println(" mH");
  
  // Display results on the TFT.
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Curve Fit RL Params:", 10, 10, 2);
  tft.drawString("R: " + String(R_est, 2) + " Ohm", 10, 40, 2);
  tft.drawString("L: " + String(L_est * 1e3, 2) + " mH", 10, 70, 2);
}
