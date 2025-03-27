#include <Arduino.h>
#include <TFT_eSPI.h>       // TFT display library
#include <Eigen/Dense>      // ArduinoEigen library for matrix computations
#include <math.h>

using namespace Eigen;

// ===== Hardware Configuration =====
const int tonePin   = 25;   // PWM output for transistor base
const int adcBattPin = 34;  // ADC measuring battery negative voltage
const int adcResPin  = 35;  // ADC measuring voltage across 2.5Ω resistor

const int numSweepPoints = 50;
float freqPoints[numSweepPoints];  // Sweep frequencies
float Vbatt[numSweepPoints];       // Battery voltage
float Vres[numSweepPoints];        // Resistor voltage
float Zmeasured[numSweepPoints];   // Measured impedance

TFT_eSPI tft = TFT_eSPI();

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  ledcSetup(0, 1000, 8);  // Initial frequency 1kHz, 8-bit resolution
  ledcAttachPin(tonePin, 0);

  analogReadResolution(12);  // ESP32 ADC is 12-bit

  // Logarithmic frequency spacing
  float startFreq = 100.0, endFreq = 40000.0;
  for (int i = 0; i < numSweepPoints; i++) {
    float ratio = (float)i / (numSweepPoints - 1);
    freqPoints[i] = startFreq * pow((endFreq / startFreq), ratio);
  }
}

void loop() {
  performFrequencySweep();
  plotTransferFunction();
  inferRLParameters();  

  while (true) delay(1000);  // Stop execution after one run
}

// ===== Frequency Sweep =====
void performFrequencySweep() {
  const int numSamples = 50;
  for (int i = 0; i < numSweepPoints; i++) {
    uint32_t currentFreq = (uint32_t)freqPoints[i];
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

    Vbatt[i] = voltageBatt;
    Vres[i]  = voltageRes;

    float current = voltageRes / 2.5;
    Zmeasured[i] = (fabs(current) > 1e-4) ? (voltageBatt / current) : 0;
  }
  ledcWriteTone(0, 0);  // Stop PWM after sweep
}

// ===== Transfer Function Plotting =====
void plotTransferFunction() {
  tft.fillScreen(TFT_BLACK);
  tft.drawLine(20, 220, 300, 220, TFT_WHITE);
  tft.drawLine(20, 20, 20, 220, TFT_WHITE);

  float maxZ = 0;
  for (int i = 0; i < numSweepPoints; i++)
    if (Zmeasured[i] > maxZ) maxZ = Zmeasured[i];

  for (int i = 0; i < numSweepPoints; i++) {
    float logFreq = log10(freqPoints[i]);
    float x = 20 + (logFreq - log10(100)) / (log10(40000) - log10(100)) * (300 - 20);
    float y = 220 - (Zmeasured[i] / maxZ) * (220 - 20);
    tft.fillCircle(x, y, 2, TFT_GREEN);

    if (i > 0) {
      float prevLogFreq = log10(freqPoints[i - 1]);
      float x0 = 20 + (prevLogFreq - log10(100)) / (log10(40000) - log10(100)) * (300 - 20);
      float y0 = 220 - (Zmeasured[i - 1] / maxZ) * (220 - 20);
      tft.drawLine(x0, y0, x, y, TFT_GREEN);
    }
  }
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Transfer Function", 40, 5, 2);
}

// ===== Curve Fitting to Extract R and L =====
void inferRLParameters() {
  VectorXf Z(numSweepPoints), freq(numSweepPoints);
  for (int i = 0; i < numSweepPoints; i++) {
    Z[i] = Zmeasured[i];
    freq[i] = freqPoints[i];
  }

  // Design matrix A: [ 1, (2πf)^2 ]
  MatrixXf A(numSweepPoints, 2);
  VectorXf b(numSweepPoints);
  for (int i = 0; i < numSweepPoints; i++) {
    A(i, 0) = 1;
    A(i, 1) = pow(2 * PI * freq[i], 2);
    b[i] = pow(Z[i], 2);
  }

  // Solve Ax = b using least squares (x = [R^2, L^2])
  VectorXf x = A.colPivHouseholderQr().solve(b);
  float R_batt = sqrt(x[0]);
  float L_batt = sqrt(x[1]);

  // Print and display results
  Serial.println("----- Fitted RL Parameters -----");
  Serial.print("R: "); Serial.print(R_batt, 3); Serial.println(" Ohm");
  Serial.print("L: "); Serial.print(L_batt * 1e3, 3); Serial.println(" mH");

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Battery RL Params:", 10, 10, 2);
  tft.drawString("R: " + String(R_batt, 2) + " Ohm", 10, 40, 2);
  tft.drawString("L: " + String(L_batt * 1e3, 2) + " mH", 10, 70, 2);
}
