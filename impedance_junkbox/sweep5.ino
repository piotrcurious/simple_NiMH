#include <Arduino.h>
#include <TFT_eSPI.h>       // TFT display library
#include <Eigen/Dense>      // ArduinoEigen library for matrix computations
#include <math.h>

using namespace Eigen;

// ===== Hardware Configuration =====
const int tonePin     = 25;   // PWM output for transistor base
const int adcBattPin  = 34;   // ADC for battery negative terminal voltage
const int adcResPin   = 35;   // ADC for voltage across 2.5Ω resistor

// Sweep settings
const int numSweepPoints = 50;         // Number of frequency points (logarithmically spaced)
float freqPoints[numSweepPoints];        // Frequency array (Hz)
float sweepZ[numSweepPoints];            // Averaged impedance from full sweeps

// Iterative measurement settings
const int fullSweepIterations = 5;       // Number of full sweeps over all frequencies
const int extraSweepCount = 3;           // Extra measurements for critical frequencies
// Critical frequency indices: low-frequency indices (for R estimation) and highest index (for L estimation)
const int lowFreqCount = 3;              // First three points considered low frequency
const int highFreqIndex = numSweepPoints - 1; 

// For accumulating measurements over iterations
float accumZ[numSweepPoints];            // Accumulated impedance for each frequency
int countZ[numSweepPoints];              // Count of measurements per frequency

// TFT Display instance
TFT_eSPI tft = TFT_eSPI();

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Configure LEDC PWM for tone generation
  ledcSetup(0, 1000, 8);   // initial frequency 1kHz, 8-bit resolution
  ledcAttachPin(tonePin, 0);

  analogReadResolution(12);  // ESP32 ADC: 12-bit

  // Prepare logarithmically spaced frequency sweep array between 100 Hz and 40 kHz
  float startFreq = 100.0, endFreq = 40000.0;
  for (int i = 0; i < numSweepPoints; i++) {
    float ratio = (float)i / (numSweepPoints - 1);
    freqPoints[i] = startFreq * pow((endFreq / startFreq), ratio);
    accumZ[i] = 0;
    countZ[i] = 0;
  }
}

void loop() {
  // Iteratively perform full frequency sweeps and accumulate measurements.
  for (int iter = 0; iter < fullSweepIterations; iter++) {
    performFullSweep();
  }
  
  // Take extra measurements for the critical frequencies.
  for (int iter = 0; iter < extraSweepCount; iter++) {
    for (int i = 0; i < lowFreqCount; i++) {
      performSingleMeasurement(i);
    }
    performSingleMeasurement(highFreqIndex);
  }
  
  // Compute average impedance for each frequency point.
  for (int i = 0; i < numSweepPoints; i++) {
    if (countZ[i] > 0)
      sweepZ[i] = accumZ[i] / countZ[i];
    else
      sweepZ[i] = 0;
  }
  
  // Plot the transfer function on the TFT.
  plotTransferFunction();
  
  // Use curve fitting (least squares via ArduinoEigen) to estimate R and L.
  inferRLParameters();
  
  // Stop execution after one overall cycle.
  while (true) {
    delay(1000);
  }
}

// ===== Measurement Functions =====

// Perform a full frequency sweep (averaging several ADC samples at each frequency).
void performFullSweep() {
  const int numSamples = 50;
  for (int i = 0; i < numSweepPoints; i++) {
    uint32_t currentFreq = (uint32_t)freqPoints[i];
    ledcWriteTone(0, currentFreq);
    delay(100);  // allow stabilization
    
    long sumBatt = 0, sumRes = 0;
    for (int j = 0; j < numSamples; j++) {
      sumBatt += analogRead(adcBattPin);
      sumRes  += analogRead(adcResPin);
      delay(2);
    }
    float voltageBatt = (sumBatt / (float)numSamples) * (3.3 / 4095.0);
    float voltageRes  = (sumRes / (float)numSamples) * (3.3 / 4095.0);
    float current = voltageRes / 2.5; // using 2.5Ω resistor
    float Zpoint = (fabs(current) > 1e-4) ? (voltageBatt / current) : 0;

    // Accumulate measurement for this frequency.
    accumZ[i] += Zpoint;
    countZ[i]++;

    // Also log the value via Serial.
    Serial.print("Iter Sweep - Freq: ");
    Serial.print(freqPoints[i]);
    Serial.print(" Hz, Z: ");
    Serial.print(Zpoint, 3);
    Serial.println(" Ohm");
  }
  ledcWriteTone(0, 0);  // Stop tone generation after the sweep
}

// Perform a single measurement at a specified frequency index.
void performSingleMeasurement(int idx) {
  const int numSamples = 50;
  uint32_t currentFreq = (uint32_t)freqPoints[idx];
  ledcWriteTone(0, currentFreq);
  delay(100);
  
  long sumBatt = 0, sumRes = 0;
  for (int j = 0; j < numSamples; j++) {
    sumBatt += analogRead(adcBattPin);
    sumRes  += analogRead(adcResPin);
    delay(2);
  }
  float voltageBatt = (sumBatt / (float)numSamples) * (3.3 / 4095.0);
  float voltageRes  = (sumRes / (float)numSamples) * (3.3 / 4095.0);
  float current = voltageRes / 2.5;
  float Zpoint = (fabs(current) > 1e-4) ? (voltageBatt / current) : 0;
  
  accumZ[idx] += Zpoint;
  countZ[idx]++;
  
  Serial.print("Extra Sweep - Freq: ");
  Serial.print(freqPoints[idx]);
  Serial.print(" Hz, Z: ");
  Serial.print(Zpoint, 3);
  Serial.println(" Ohm");
  
  ledcWriteTone(0, 0);  // Stop tone generation
}

// ===== Plot Transfer Function on TFT =====
void plotTransferFunction() {
  tft.fillScreen(TFT_BLACK);
  // Draw axes (x-axis: frequency, y-axis: impedance)
  tft.drawLine(20, 220, 300, 220, TFT_WHITE);
  tft.drawLine(20, 20, 20, 220, TFT_WHITE);

  // Find maximum impedance for autoscaling
  float maxZ = 0;
  for (int i = 0; i < numSweepPoints; i++) {
    if (sweepZ[i] > maxZ)
      maxZ = sweepZ[i];
  }

  // Plot the data (logarithmic x-scale)
  for (int i = 0; i < numSweepPoints; i++) {
    float logFreq = log10(freqPoints[i]);
    float x = 20 + (logFreq - log10(100)) / (log10(40000) - log10(100)) * (300 - 20);
    float y = 220 - (sweepZ[i] / maxZ) * (220 - 20);
    tft.fillCircle(x, y, 2, TFT_GREEN);

    if (i > 0) {
      float prevLogFreq = log10(freqPoints[i - 1]);
      float x0 = 20 + (prevLogFreq - log10(100)) / (log10(40000) - log10(100)) * (300 - 20);
      float y0 = 220 - (sweepZ[i - 1] / maxZ) * (220 - 20);
      tft.drawLine(x0, y0, x, y, TFT_GREEN);
    }
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Transfer Function", 40, 5, 2);
}

// ===== Curve Fitting: Least Squares for RL Parameters =====
// The theoretical model is: Z(f)^2 = R^2 + (2π f L)^2
// We set up a linear regression with:
//    y = R^2 + (2π f)^2 L^2
// where y = Z^2. We fit for parameters: a = R^2 and b = L^2.
void inferRLParameters() {
  // Create vectors for measured Z^2 and frequencies.
  VectorXf Y(numSweepPoints);
  VectorXf f(numSweepPoints);
  for (int i = 0; i < numSweepPoints; i++) {
    Y[i] = sweepZ[i] * sweepZ[i];
    f[i] = freqPoints[i];
  }
  
  // Build the design matrix A with two columns:
  // Column 0: constant 1 (for R^2) and Column 1: (2πf)^2 (for L^2)
  MatrixXf A(numSweepPoints, 2);
  for (int i = 0; i < numSweepPoints; i++) {
    A(i, 0) = 1;
    A(i, 1) = pow(2 * PI * f[i], 2);
  }
  
  // Solve for x = [R^2, L^2] via least squares (using QR decomposition)
  VectorXf x = A.colPivHouseholderQr().solve(Y);
  float R_batt = sqrt(x[0]);
  float L_batt = sqrt(x[1]);
  
  // Print fitted parameters to Serial
  Serial.println("----- Fitted RL Parameters (Iterative) -----");
  Serial.print("Estimated R: "); Serial.print(R_batt, 3); Serial.println(" Ohm");
  Serial.print("Estimated L: "); Serial.print(L_batt * 1e3, 3); Serial.println(" mH");

  // Display results on the TFT screen.
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Battery RL Params:", 10, 10, 2);
  tft.drawString("R: " + String(R_batt, 2) + " Ohm", 10, 40, 2);
  tft.drawString("L: " + String(L_batt * 1e3, 2) + " mH", 10, 70, 2);
}
