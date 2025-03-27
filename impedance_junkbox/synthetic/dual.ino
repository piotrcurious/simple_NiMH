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
void generateTwoFrequencyWave(float freq1, float freq2, int durationMs, bool invertFreq1);
float impedanceModel(float frequency, float inductance, float capacitance);
std::pair<float, float> initialRLCInference(const std::vector<float>& frequencies, const std::vector<float>& impedances);
float analyzeTargetedPhaseReversal(float freq1, float freq2, int durationMs);

void setup() {
  // ...
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Inferring RLC (Sweeping Two Frequencies)...");

  measuredFrequencies.clear();
  measuredImpedances.clear();

  // --- Initial RLC Inference ---
  tft.println("Performing Initial Frequency Sweep...");
  std::vector<float> initialFrequencies = linspace(100, 20000, 10);
  for (float freq : initialFrequencies) {
    generateSquareWave(freq, 50, false);
    float avgV = measureAverageBatteryVoltage(numAverageReadings, measurementDelayMs);
    float avgI = measureAverageShuntVoltage(numAverageReadings, measurementDelayMs) / shuntResistance;
    if (avgI > 1e-6) {
      measuredFrequencies.push_back(freq);
      measuredImpedances.push_back(avgV / avgI);
      Serial.printf("Sweep Freq: %.0f Hz, |Z|: %.3f Ohm\n", freq, avgV / avgI);
    }
    delay(10);
  }
  std::pair<float, float> initialEstimates = initialRLCInference(measuredFrequencies, initialImpedances);
  float initialResonantFrequency = 1.0 / (2 * PI * sqrt(initialEstimates.first * initialEstimates.second));
  tft.printf("Estimated Resonant Frequency: %.0f Hz\n", initialResonantFrequency);
  delay(2000);

  // --- Analyze Targeted Phase Reversal with Sweeping Frequencies ---
  tft.println("Analyzing Targeted Phase Reversal with Sweeping Frequencies...");

  float freq1Start = initialResonantFrequency * 0.5;
  float freq1End = initialResonantFrequency * 1.5;
  std::vector<float> freq1Sweep = linspace(freq1Start, freq1End, 15);

  float freq2Start = 500.0; // Example range for the second frequency
  float freq2End = 2000.0;
  std::vector<float> freq2Sweep = linspace(freq2Start, freq2End, 10);

  std::vector<std::tuple<float, float, float>> responseData; // freq1, freq2, deltaCurrent

  for (float freq1 : freq1Sweep) {
    for (float freq2 : freq2Sweep) {
      tft.printf("Testing Frequencies: %.0f Hz (f1), %.0f Hz (f2)...\n", freq1, freq2);
      float responseDelta = analyzeTargetedPhaseReversal(freq1, freq2, 150);
      responseData.emplace_back(freq1, freq2, responseDelta);
      tft.printf("Phase Reversal Response: Delta Current %.3f A\n", responseDelta);
      delay(20);
    }
  }

  // --- Infer Resonant Frequency from Response Data ---
  float bestFreq1 = initialResonantFrequency;
  float bestFreq2 = 1000.0;
  float maxResponseDelta = -1.0;

  for (const auto& dataPoint : responseData) {
    float f1, f2, deltaI;
    std::tie(f1, f2, deltaI) = dataPoint;
    if (fabs(deltaI) > fabs(maxResponseDelta)) {
      maxResponseDelta = deltaI;
      bestFreq1 = f1;
      bestFreq2 = f2;
    }
  }
  tft.printf("Best Frequencies for Phase Reversal: %.0f Hz (f1), %.0f Hz (f2)\n", bestFreq1, bestFreq2);

  // --- Infer RLC Parameters (More Complex with Two Frequencies) ---
  // This step requires a deeper understanding of how the RLC circuit responds to two frequencies and the phase reversal of one.
  // A simple approach might be to assume that the best f1 is close to the resonant frequency.

  float refinedResonantFrequency = bestFreq1;
  float impedanceAtResonance = 0.0;
  for (size_t i = 0; i < measuredFrequencies.size(); ++i) {
    if (abs(measuredFrequencies[i] - refinedResonantFrequency) < refinedResonantFrequency / 10.0) {
      impedanceAtResonance = measuredImpedances[i];
      break;
    }
  }
  float refinedL = 1.0 / (pow(2 * PI * refinedResonantFrequency, 2) * initialEstimates.second);
  float refinedC = 1.0 / (pow(2 * PI * refinedResonantFrequency, 2) * refinedL);
  float refinedR = impedanceAtResonance - Rint;
  if (refinedR < 0) refinedR = 0.01;

  tft.printf("Refined L (Two Freq Sweep): %.3e H\n", refinedL);
  tft.printf("Refined C (Two Freq Sweep): %.3e F\n", refinedC);
  tft.printf("Refined R (Two Freq Sweep): %.3f Ohm\n", refinedR);

  delay(10000);
}

void setupTFT() {
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
}

// ... (plotGraph, plotBode, measureAverageBatteryVoltage, measureAverageShuntVoltage, impedanceModel, linspace, initialRLCInference, generateSquareWave)

void generateTwoFrequencyWave(float freq1, float freq2, int durationMs, bool invertFreq1) {
  long startTime = millis();
  while (millis() - startTime < durationMs) {
    float timeSec = (millis() - startTime) / 1000.0;
    float signal1 = sin(2 * PI * freq1 * timeSec);
    float signal2 = sin(2 * PI * freq2 * timeSec);
    float combinedSignal = signal1 + signal2;
    digitalWrite(transistorBasePin, ((combinedSignal > 0) ^ invertFreq1) ? HIGH : LOW);
    delayMicroseconds(1000000.0 / (std::max(freq1, freq2) * 10));
  }
  digitalWrite(transistorBasePin, LOW);
}

float analyzeTargetedPhaseReversal(float freq1, float freq2, int durationMs) {
  // Measure average current with normal phase for freq1
  generateTwoFrequencyWave(freq1, freq2, durationMs, false);
  float avgCurrentNormal = measureAverageShuntVoltage(numAverageReadings * 2, measurementDelayMs) / shuntResistance;
  delay(20);

  // Measure average current with reversed phase for freq1
  generateTwoFrequencyWave(freq1, freq2, durationMs, true);
  float avgCurrentInverted = measureAverageShuntVoltage(numAverageReadings * 2, measurementDelayMs) / shuntResistance;

  // Return the difference in average current
  return avgCurrentNormal - avgCurrentInverted;
}

std::pair<float, float> initialRLCInference(const std::vector<float>& frequencies, const std::vector<float>& impedances) {
  if (frequencies.empty() || impedances.empty() || frequencies.size() != impedances.size()) {
    return {1e-6, 1e-3}; // Default values
  }

  auto minIt = std::min_element(impedances.begin(), impedances.end());
  int minIndex = std::distance(impedances.begin(), minIt);
  float resonantFrequency = frequencies[minIndex];
  float impedanceAtResonance = impedances[minIndex];

  float inductance = 1.0 / (pow(2 * PI * resonantFrequency, 2) * 1e-3); // Assume C = 1mF initially
  float capacitance = 1e-3;
  if (inductance > 0 && resonantFrequency > 0) {
    capacitance = 1.0 / (pow(2 * PI * resonantFrequency, 2) * inductance);
  }
  return {inductance, capacitance};
}
