#include <WiFi.h>
#include <TFT_eSPI>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

// ... (TFT, Pin, Shunt Resistance, Rint definitions)
// ... (Measurement Parameters, Burst Parameters, Data Storage, Plotting Functions, linspace)

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData);
float measureAverageBatteryVoltage(int numReadings, int delayMs);
float measureAverageShuntVoltage(int numReadings, int delayMs);
void generateSquareWave(float frequency, int durationMs, bool inverted); // Added inverted parameter
float impedanceModel(float frequency, float inductance, float capacitance);
std::pair<float, float> initialRLCInference(const std::vector<float>& frequencies, const std::vector<float>& impedances);
std::pair<float, float> analyzePhaseSensitivity(float frequency, int durationMs);

void setup() {
  // ...
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Inferring RLC (Phase Reversal for Resonance)...");

  measuredFrequencies.clear();
  measuredImpedances.clear();

  // --- Initial RLC Inference ---
  // ... (Perform initial sweep as before)
  std::vector<float> initialFrequencies = linspace(100, 10000, 5);
  for (float freq : initialFrequencies) {
    generateSquareWave(freq, 50, false);
    float avgV = measureAverageBatteryVoltage(numAverageReadings, measurementDelayMs);
    float avgI = measureAverageShuntVoltage(numAverageReadings, measurementDelayMs) / shuntResistance;
    if (avgI > 1e-6) {
      measuredFrequencies.push_back(freq);
      measuredImpedances.push_back(avgV / avgI);
    }
    delay(10);
  }
  std::pair<float, float> initialEstimates = initialRLCInference(measuredFrequencies, measuredImpedances);
  float estimatedL = initialEstimates.first;
  float estimatedC = initialEstimates.second;
  float resonantFrequency = 1.0 / (2 * PI * sqrt(estimatedL * estimatedC));
  tft.printf("Estimated Resonant Frequency: %.0f Hz\n", resonantFrequency);
  delay(2000);

  // --- Analyze Phase Sensitivity Around Resonance ---
  tft.println("Analyzing Phase Sensitivity Near Resonance...");

  std::vector<float> testFrequencies = linspace(resonantFrequency * 0.8, resonantFrequency * 1.2, 10);
  std::vector<std::pair<float, float>> phaseSensitivityData;

  for (float freq : testFrequencies) {
    tft.printf("Testing Frequency: %.0f Hz for Phase Sensitivity...\n", freq);
    std::pair<float, float> sensitivity = analyzePhaseSensitivity(freq, 200);
    phaseSensitivityData.push_back({freq, sensitivity.first}); // Store frequency and some measure of sensitivity
    tft.printf("Phase Sensitivity at %.0f Hz: Delta Voltage %.3f V, Delta Current %.3f A\n", freq, sensitivity.first, sensitivity.second);
    delay(1000);
  }

  // --- Infer RLC from Phase Sensitivity Data ---
  // This requires a model of how the phase reversal affects the average current/voltage
  // near resonance as a function of R, L, and C.

  // Placeholder for inference logic
  float refinedL = estimatedL;
  float refinedC = estimatedC;
  float refinedR = 0.1; // Placeholder

  tft.printf("Refined L: %.3e H, C: %.3e F, R: %.3f Ohm\n", refinedL, refinedC, refinedR);

  delay(10000);
}

void setupTFT() {
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
}

// ... (plotGraph, plotBode, measureAverageBatteryVoltage, measureAverageShuntVoltage, impedanceModel, linspace, initialRLCInference)

void generateSquareWave(float frequency, int durationMs, bool inverted) {
  long periodUs = 1000000.0 / frequency;
  long highTimeUs = periodUs / 2;
  long startTime = millis();
  int highLevel = inverted ? LOW : HIGH;
  int lowLevel = inverted ? HIGH : LOW;

  while (millis() - startTime < durationMs) {
    digitalWrite(transistorBasePin, highLevel);
    delayMicroseconds(highTimeUs);
    digitalWrite(transistorBasePin, lowLevel);
    delayMicroseconds(periodUs - highTimeUs);
  }
  digitalWrite(transistorBasePin, LOW); // Ensure off
}

std::pair<float, float> analyzePhaseSensitivity(float frequency, int durationMs) {
  // Measure response to normal square wave
  generateSquareWave(frequency, durationMs, false);
  float avgVoltageNormal = measureAverageBatteryVoltage(numAverageReadings * 2, measurementDelayMs);
  float avgCurrentNormal = measureAverageShuntVoltage(numAverageReadings * 2, measurementDelayMs) / shuntResistance;
  delay(50);

  // Measure response to phase-reversed square wave
  generateSquareWave(frequency, durationMs, true);
  float avgVoltageInverted = measureAverageBatteryVoltage(numAverageReadings * 2, measurementDelayMs);
  float avgCurrentInverted = measureAverageShuntVoltage(numAverageReadings * 2, measurementDelayMs) / shuntResistance;

  // Calculate the difference in average voltage and current
  float deltaVoltage = avgVoltageNormal - avgVoltageInverted;
  float deltaCurrent = avgCurrentNormal - avgCurrentInverted;

  return {deltaVoltage, deltaCurrent};
}
