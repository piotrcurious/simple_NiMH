#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <algorithm>

using namespace Eigen;

// ===== Hardware Configuration =====
const int tonePin    = 25;   // PWM output for transistor base
const int adcBattPin = 34;   // ADC for battery negative voltage
const int adcResPin  = 35;   // ADC for voltage across 2.5Ω resistor

TFT_eSPI tft = TFT_eSPI();

// ===== Frequency Sweep and Iteration Parameters =====
const float f_min = 100.0;
const float f_max = 40000.0;
const int initialPoints = 20;  // Initial frequency sweep points
const int maxIterations = 7;   // Maximum allowed iterations
const int numSamples = 50;     // ADC samples per frequency point

// Convergence thresholds:
const float errorThreshold = 0.05;    // 5% relative error allowed
const float paramChangeThreshold = 0.01; // 1% change in R & L between iterations

std::vector<float> freqPoints;
std::vector<float> Vbatt;
std::vector<float> Vres;
std::vector<float> Zmeasured;

// ===== Global Variables for Tracking Parameters =====
float last_R = 0, last_L = 0;

// ===== Function Prototypes =====
void initializeFrequencyPoints();
void performFrequencySweep();
void plotTransferFunction();
void inferRLParameters(float &R_batt, float &L_batt);
bool refineFrequencySelection(float R_batt, float L_batt, float &avgError, float &maxError);

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Configure LEDC for tone generation
  ledcSetup(0, 1000, 8);
  ledcAttachPin(tonePin, 0);
  analogReadResolution(12);

  initializeFrequencyPoints();
}

void loop() {
  int iter = 0;
  bool needMoreIterations = true;
  float R_batt = 0, L_batt = 0;
  
  while (needMoreIterations && iter < maxIterations) {
    Serial.print("Iteration ");
    Serial.println(iter + 1);
    
    performFrequencySweep();
    plotTransferFunction();
    inferRLParameters(R_batt, L_batt);
    
    float avgError = 0, maxError = 0;
    needMoreIterations = refineFrequencySelection(R_batt, L_batt, avgError, maxError);
    
    // Check for convergence in parameter values:
    if (iter > 0) {
      float dR = fabs(R_batt - last_R) / last_R;
      float dL = fabs(L_batt - last_L) / last_L;
      Serial.print("Parameter change: dR = ");
      Serial.print(dR, 3);
      Serial.print(", dL = ");
      Serial.println(dL, 3);
      if (dR < paramChangeThreshold && dL < paramChangeThreshold) {
        needMoreIterations = false;
        Serial.println("Parameters converged based on relative change.");
      }
    }
    
    last_R = R_batt;
    last_L = L_batt;
    
    Serial.print("Average Relative Error: ");
    Serial.print(avgError * 100, 1);
    Serial.print("%, Maximum Relative Error: ");
    Serial.print(maxError * 100, 1);
    Serial.println("%");
    
    iter++;
    delay(500);
  }
  
  Serial.println("Final Fitted Parameters:");
  Serial.print("R: "); Serial.print(R_batt, 3); Serial.println(" Ohm");
  Serial.print("L: "); Serial.print(L_batt * 1e3, 3); Serial.println(" mH");

  // Display final results on TFT:
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Final RL Params:", 10, 10, 2);
  tft.drawString("R: " + String(R_batt, 2) + " Ohm", 10, 40, 2);
  tft.drawString("L: " + String(L_batt * 1e3, 2) + " mH", 10, 70, 2);
  
  while (true) delay(1000);
}

// ===== Initialize Frequency Points (Logarithmic Spacing) =====
void initializeFrequencyPoints() {
  freqPoints.clear();
  for (int i = 0; i < initialPoints; i++) {
    float ratio = (float)i / (initialPoints - 1);
    freqPoints.push_back(f_min * pow((f_max / f_min), ratio));
  }
}

// ===== Perform Frequency Sweep =====
void performFrequencySweep() {
  Vbatt.clear();
  Vres.clear();
  Zmeasured.clear();
  
  for (float freq : freqPoints) {
    ledcWriteTone(0, (uint32_t)freq);
    delay(100);  // Allow settling
    
    long sumBatt = 0, sumRes = 0;
    for (int j = 0; j < numSamples; j++) {
      sumBatt += analogRead(adcBattPin);
      sumRes  += analogRead(adcResPin);
      delay(2);
    }
    float voltageBatt = (sumBatt / (float)numSamples) * (3.3 / 4095.0);
    float voltageRes  = (sumRes / (float)numSamples) * (3.3 / 4095.0);
    float current = voltageRes / 2.5;
    
    Vbatt.push_back(voltageBatt);
    Vres.push_back(voltageRes);
    Zmeasured.push_back((fabs(current) > 1e-4) ? (voltageBatt / current) : 0);
  }
  
  ledcWriteTone(0, 0);  // Stop tone generation
}

// ===== Plot Transfer Function on TFT =====
void plotTransferFunction() {
  tft.fillScreen(TFT_BLACK);
  tft.drawLine(20, 220, 300, 220, TFT_WHITE);
  tft.drawLine(20, 20, 20, 220, TFT_WHITE);
  
  float maxZ = *std::max_element(Zmeasured.begin(), Zmeasured.end());
  for (size_t i = 0; i < freqPoints.size(); i++) {
    float logFreq = log10(freqPoints[i]);
    float x = 20 + (logFreq - log10(f_min)) / (log10(f_max) - log10(f_min)) * (300 - 20);
    float y = 220 - (Zmeasured[i] / maxZ) * (220 - 20);
    tft.fillCircle(x, y, 2, TFT_GREEN);
  }
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Transfer Function", 40, 5, 2);
}

// ===== Fit the RL Model Using Least Squares =====
// The model is: Z^2 = R^2 + (2πf L)^2
void inferRLParameters(float &R_batt, float &L_batt) {
  int n = freqPoints.size();
  VectorXf Z(n), freq(n);
  for (int i = 0; i < n; i++) {
    Z[i] = Zmeasured[i];
    freq[i] = freqPoints[i];
  }
  
  MatrixXf A(n, 2);
  VectorXf b(n);
  for (int i = 0; i < n; i++) {
    A(i, 0) = 1;
    A(i, 1) = pow(2 * PI * freq[i], 2);
    b[i] = pow(Z[i], 2);
  }
  
  VectorXf x = A.colPivHouseholderQr().solve(b);
  R_batt = sqrt(x[0]);
  L_batt = sqrt(x[1]);
}

// ===== Refine Frequency Selection Based on Error Analysis =====
// Computes the average and maximum relative errors between measured Z and the model.
// Adds new frequencies in regions where error exceeds threshold.
bool refineFrequencySelection(float R_batt, float L_batt, float &avgError, float &maxError) {
  avgError = 0;
  maxError = 0;
  std::vector<float> newFreqs;
  int count = 0;
  
  for (size_t i = 0; i < freqPoints.size(); i++) {
    float f = freqPoints[i];
    float predictedZ = sqrt(R_batt * R_batt + pow(2 * PI * f * L_batt, 2));
    float relError = fabs(Zmeasured[i] - predictedZ) / predictedZ;
    avgError += relError;
    count++;
    if (relError > maxError)
      maxError = relError;
    
    // If error is high, add intermediate frequencies
    if (relError > errorThreshold) {
      if (i > 0) {
        float f_new = sqrt(freqPoints[i - 1] * f);
        newFreqs.push_back(f_new);
      }
      if (i < freqPoints.size() - 1) {
        float f_new = sqrt(f * freqPoints[i + 1]);
        newFreqs.push_back(f_new);
      }
    }
  }
  avgError /= count;
  
  if (newFreqs.empty()) {
    Serial.println("No additional frequencies required.");
    return false;  // Convergence achieved
  }
  
  // Insert new frequencies, ensuring uniqueness and bounds
  for (float f : newFreqs) {
    if (f > f_min && f < f_max)
      freqPoints.push_back(f);
  }
  std::sort(freqPoints.begin(), freqPoints.end());
  freqPoints.erase(std::unique(freqPoints.begin(), freqPoints.end()), freqPoints.end());
  
  Serial.print("Added "); Serial.print(newFreqs.size());
  Serial.println(" new frequency points.");
  
  return true;  // Further iterations required
}
