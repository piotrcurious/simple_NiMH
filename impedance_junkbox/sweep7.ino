#include <Arduino.h>
#include <TFT_eSPI.h>       
#include <Eigen/Dense>      
#include <math.h>

using namespace Eigen;

// ===== Hardware Configuration =====
const int tonePin    = 25;   // PWM output for transistor base
const int adcBattPin = 34;   // ADC measuring battery negative voltage
const int adcResPin  = 35;   // ADC measuring voltage across 2.5Ω resistor

TFT_eSPI tft = TFT_eSPI();

// ===== Frequency Sweep Parameters =====
const float f_min = 100.0;
const float f_max = 40000.0;
const int initialPoints = 20;
const int maxIterations = 5;
const int numSamples = 50;
const float errorThreshold = 0.05;  // Convergence threshold (5% error)

std::vector<float> freqPoints;
std::vector<float> Vbatt;
std::vector<float> Vres;
std::vector<float> Zmeasured;

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  ledcSetup(0, 1000, 8);
  ledcAttachPin(tonePin, 0);
  analogReadResolution(12);

  initializeFrequencyPoints();
}

void loop() {
  for (int iter = 0; iter < maxIterations; iter++) {
    performFrequencySweep();
    plotTransferFunction();
    
    float R_batt, L_batt;
    inferRLParameters(R_batt, L_batt);

    if (!refineFrequencySelection(R_batt, L_batt))
      break;  // Converged, exit loop
  }

  while (true) delay(1000);
}

// ===== Initialize Frequency Points (Log Scale) =====
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
    
    Vbatt.push_back(voltageBatt);
    Vres.push_back(voltageRes);
    Zmeasured.push_back((fabs(current) > 1e-4) ? (voltageBatt / current) : 0);
  }

  ledcWriteTone(0, 0);
}

// ===== Plot Transfer Function =====
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

// ===== Fit RL Model Using Least Squares =====
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

  Serial.println("----- Iteration Results -----");
  Serial.print("R: "); Serial.print(R_batt, 3); Serial.println(" Ohm");
  Serial.print("L: "); Serial.print(L_batt * 1e3, 3); Serial.println(" mH");

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Battery RL Params:", 10, 10, 2);
  tft.drawString("R: " + String(R_batt, 2) + " Ohm", 10, 40, 2);
  tft.drawString("L: " + String(L_batt * 1e3, 2) + " mH", 10, 70, 2);
}

// ===== Refine Frequency Selection Based on Error Analysis =====
bool refineFrequencySelection(float R_batt, float L_batt) {
  std::vector<float> newFreqs;
  float maxError = 0;

  for (size_t i = 0; i < freqPoints.size(); i++) {
    float f = freqPoints[i];
    float predictedZ = sqrt(R_batt * R_batt + pow(2 * PI * f * L_batt, 2));
    float error = fabs((Zmeasured[i] - predictedZ) / predictedZ);
    
    if (error > errorThreshold) {
      maxError = fmax(maxError, error);

      if (i > 0)
        newFreqs.push_back(sqrt(freqPoints[i - 1] * f));
      if (i < freqPoints.size() - 1)
        newFreqs.push_back(sqrt(f * freqPoints[i + 1]));
    }
  }

  if (newFreqs.empty()) 
    return false;  // Converged

  freqPoints.insert(freqPoints.end(), newFreqs.begin(), newFreqs.end());
  std::sort(freqPoints.begin(), freqPoints.end());
  freqPoints.erase(std::unique(freqPoints.begin(), freqPoints.end()), freqPoints.end());

  return true;  // Need more iterations
}
