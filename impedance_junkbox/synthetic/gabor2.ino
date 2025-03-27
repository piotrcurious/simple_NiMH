#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

// TFT Display Configuration
TFT_eSPI tft = TFT_eSPI();

// Pin Definitions
const int transistorBasePin = 2;   // ESP32 GPIO pin to control the transistor base
const int batteryVoltagePin = 34; // ESP32 GPIO pin to measure battery voltage (ADC1)
const int resistorVoltagePin = 35; // ESP32 GPIO pin to measure voltage across the shunt resistor (ADC1)

// Shunt Resistor Value
const float shuntResistance = 2.5; // Ohms

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Measurement Parameters
const int numAverageReadings = 15;
const int measurementDelayMs = 15;

// Initial Sweep Parameters
const int initialStartFrequency = 100;
const int initialEndFrequency = 20000;
const int initialFrequencySteps = 20;

// Burst Pulse Parameters
const int burstDelayMs = 50;

struct BurstConfig {
    float frequency;
    int durationUs;
};

// Data Storage
std::vector<float> measuredFrequencies;
std::vector<float> measuredImpedances;

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData);
float measureAverageBatteryVoltage();
float measureAverageShuntVoltage();
void generateBurstPulse(float frequency, int durationUs);
float impedanceModel(float frequency, float inductance, float capacitance);
void inferRLC(const std::vector<float>& frequencies, const std::vector<float>& impedances);
std::vector<float> linspace(float start, float end, int num);

void setup() {
  Serial.begin(115200);
  setupTFT();
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW);
  analogReadResolution(12);
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Inferring RLC (Iterative Sweep + Bursts)...");

  measuredFrequencies.clear();
  measuredImpedances.clear();

  // --- Initial Classic Frequency Sweep ---
  tft.println("Performing Initial Frequency Sweep...");
  std::vector<float> initialFrequencies = linspace(initialStartFrequency, initialEndFrequency, initialFrequencySteps);
  for (float freq : initialFrequencies) {
    generateBurstPulse(freq, 1000000.0 / freq * 10); // Use longer burst for sweep
    float avgVoltageRaw = measureAverageBatteryVoltage();
    float avgCurrentVoltageRaw = measureAverageShuntVoltage();
    float avgVoltage = avgVoltageRaw * (3.3 / 4095.0);
    float avgCurrent = (avgCurrentVoltageRaw * (3.3 / 4095.0)) / shuntResistance;
    if (avgCurrent > 1e-6) {
      measuredFrequencies.push_back(freq);
      measuredImpedances.push_back(avgVoltage / avgCurrent);
      Serial.printf("Sweep Freq: %.0f Hz, |Z|: %.3f Ohm\n", freq, avgVoltage / avgCurrent);
    }
    delay(10);
  }

  // --- Iterative Refinement with Bursts ---
  for (int iteration = 0; iteration < 2; ++iteration) {
    tft.printf("\nIteration: %d - Analyzing Data...\n", iteration + 1);
    if (measuredFrequencies.size() < 3) break;

    // Find approximate resonance from the current data
    auto minIt = std::min_element(measuredImpedances.begin(), measuredImpedances.end());
    int minIndex = std::distance(measuredImpedances.begin(), minIt);
    float resonantFrequency = measuredFrequencies[minIndex];
    Serial.printf("Approximate Resonant Frequency: %.0f Hz\n", resonantFrequency);

    // Define burst configurations around resonance and edges
    std::vector<BurstConfig> burstConfigurations;
    burstConfigurations.push_back({resonantFrequency / 2.0, 2000});
    burstConfigurations.push_back({resonantFrequency, 1000});
    burstConfigurations.push_back({resonantFrequency * 2.0, 500});
    burstConfigurations.push_back({initialStartFrequency, 5000});
    burstConfigurations.push_back({initialEndFrequency, 200});

    tft.println("Performing Burst Measurements...");
    for (const auto& config : burstConfigurations) {
      if (std::find(measuredFrequencies.begin(), measuredFrequencies.end(), config.frequency) == measuredFrequencies.end()) {
        tft.printf("Burst Freq: %.0f Hz, Dur: %d us...\n", config.frequency, config.durationUs);
        generateBurstPulse(config.frequency, config.durationUs);
        float avgVoltageRaw = measureAverageBatteryVoltage();
        float avgCurrentVoltageRaw = measureAverageShuntVoltage();
        float avgVoltage = avgVoltageRaw * (3.3 / 4095.0);
        float avgCurrent = (avgCurrentVoltageRaw * (3.3 / 4095.0)) / shuntResistance;
        if (avgCurrent > 1e-6) {
          measuredFrequencies.push_back(config.frequency);
          measuredImpedances.push_back(avgVoltage / avgCurrent);
          Serial.printf("Burst Freq: %.0f Hz, |Z|: %.3f Ohm\n", config.frequency, avgVoltage / avgCurrent);
        }
        delay(10);
      }
    }

    // Sort data for better fitting
    std::vector<std::pair<float, float>> combinedData(measuredFrequencies.size());
    for (size_t i = 0; i < measuredFrequencies.size(); ++i) {
      combinedData[i] = std::make_pair(measuredFrequencies[i], measuredImpedances[i]);
    }
    std::sort(combinedData.begin(), combinedData.end());
    measuredFrequencies.clear();
    measuredImpedances.clear();
    for (const auto& pair : combinedData) {
      measuredFrequencies.push_back(pair.first);
      measuredImpedances.push_back(pair.second);
    }

    inferRLC(measuredFrequencies, measuredImpedances);
    delay(2000);
  }

  plotGraph("Impedance vs. Frequency", measuredFrequencies, measuredImpedances, "Frequency (Hz)", "Impedance (Ohm)");
  delay(5000);
  plotBode(measuredFrequencies, measuredImpedances);
  delay(5000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2 - 10);
  tft.println("Inference Completed!");
  delay(5000);
}

void setupTFT() {
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
}

void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel) {
  // ... (plotting function from previous versions)
}

void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData) {
  plotGraph("Bode Plot (Magnitude)", freqData, magData, "Frequency (Hz)", "Impedance (Ohm)");
}

float measureAverageBatteryVoltage() {
  // ... (measurement function from previous versions)
}

float measureAverageShuntVoltage() {
  // ... (measurement function from previous versions)
}

void generateBurstPulse(float frequency, int durationUs) {
  // ... (burst generation function from previous versions)
}

float impedanceModel(float frequency, float inductance, float capacitance) {
  // ... (impedance model function from previous versions)
}

void inferRLC(const std::vector<float>& frequencies, const std::vector<float>& impedances) {
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 50);
  tft.println("Inferring RLC Parameters (Fitting):");
  tft.printf("Known Internal Resistance (Rint): %.3f Ohm\n", Rint);

  if (frequencies.size() < 3) {
    tft.println("Not enough data points to perform fitting.");
    return;
  }

  Eigen::VectorXd measuredFreqEigen(frequencies.size());
  Eigen::VectorXd measuredImpEigen(impedances.size());
  for (size_t i = 0; i < frequencies.size(); ++i) {
    measuredFreqEigen(i) = frequencies[i];
    measuredImpEigen(i) = impedances[i];
  }

  auto costFunction = [&](const Eigen::VectorXd& params) {
    float inductance = params(0);
    float capacitance = params(1);
    double sumSquaredErrors = 0.0;
    for (size_t i = 0; i < measuredFreqEigen.size(); ++i) {
      float modelImpedance = impedanceModel(measuredFreqEigen(i), inductance, capacitance);
      sumSquaredErrors += pow(modelImpedance - measuredImpEigen(i), 2);
    }
    return sumSquaredErrors;
  };

  Eigen::VectorXd initialParams(2);
  initialParams(0) = 1e-6;
  initialParams(1) = 1e-3;

  double learningRate = 1e-10;
  int iterations = 2000;
  Eigen::VectorXd currentParams = initialParams;

  for (int iter = 0; iter < iterations; ++iter) {
    float inductance = currentParams(0);
    float capacitance = currentParams(1);

    double delta = 1e-10;
    Eigen::VectorXd gradient(2);
    Eigen::VectorXd paramsPlusDeltaL = currentParams;
    paramsPlusDeltaL(0) += delta;
    gradient(0) = (costFunction(paramsPlusDeltaL) - costFunction(currentParams)) / delta;
    Eigen::VectorXd paramsPlusDeltaC = currentParams;
    paramsPlusDeltaC(1) += delta;
    gradient(1) = (costFunction(paramsPlusDeltaC) - costFunction(currentParams)) / delta;
    currentParams -= learningRate * gradient;
    currentParams(0) = fmax(currentParams(0), 0.0f);
    currentParams(1) = fmax(currentParams(1), 1e-12f);
  }

  float inferredInductance = currentParams(0);
  float inferredCapacitance = currentParams(1);

  tft.printf("Inferred Inductance (L): %.3e H\n", inferredInductance);
  tft.printf("Inferred Capacitance (C): %.3e F\n", inferredCapacitance);
}

std::vector<float> linspace(float start, float end, int num) {
  std::vector<float> result;
  if (num <= 1) {
    result.push_back(start);
  } else {
    float step = (end - start) / (num - 1);
    for (int i = 0; i < num; ++i) {
      result.push_back(start + i * step);
    }
  }
  return result;
}
