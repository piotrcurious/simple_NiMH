#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

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

// Burst Pulse Parameters
// const int numCyclesPerBurst = 5; // Removed - now using duration
const int burstDelayMs = 50;    // Delay between bursts

// Synthetic Frequencies and Durations to Sweep
struct BurstConfig {
    float frequency;
    int durationUs; // Duration of the burst in microseconds
};

const std::vector<BurstConfig> burstConfigurations = {
    {100, 20000},   // Lower frequency, longer duration (narrower bandwidth)
    {500, 4000},    // Medium frequency, medium duration
    {2000, 1000},   // Higher frequency, shorter duration (wider bandwidth)
    {10000, 200}
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
void inferRLC();

void setup() {
  Serial.begin(115200);
  setupTFT();
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW); // Initially turn off the transistor
  analogReadResolution(12);
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Inferring RLC (Wavelet Inspired Bursts)...");

  inferRLC();

  delay(10000);
}

void setupTFT() {
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1); // Adjust rotation as needed
}

void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel) {
  // Placeholder for future use if needed
}

void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData) {
  // Placeholder for future use if needed
}

float measureAverageBatteryVoltage() {
  float totalVoltage = 0;
  for (int i = 0; i < numAverageReadings; ++i) {
    totalVoltage += analogRead(batteryVoltagePin);
    delay(measurementDelayMs);
  }
  return totalVoltage / numAverageReadings;
}

float measureAverageShuntVoltage() {
  float totalVoltage = 0;
  for (int i = 0; i < numAverageReadings; ++i) {
    totalVoltage += analogRead(resistorVoltagePin);
    delay(measurementDelayMs);
  }
  return totalVoltage / numAverageReadings;
}

void generateBurstPulse(float frequency, int durationUs) {
  long periodUs = 1000000.0 / frequency;
  long highTimeUs = periodUs / 2; // 50% duty cycle
  long startTime = micros();

  while (micros() - startTime < durationUs) {
    digitalWrite(transistorBasePin, HIGH);
    delayMicroseconds(highTimeUs);
    digitalWrite(transistorBasePin, LOW);
    delayMicroseconds(periodUs - highTimeUs);
  }
  digitalWrite(transistorBasePin, LOW); // Ensure off
  delay(burstDelayMs);
}

float impedanceModel(float frequency, float inductance, float capacitance) {
  float omega = 2 * PI * frequency;
  float z_imag = omega * inductance - 1.0 / (omega * capacitance);
  return sqrt(pow(Rint, 2) + pow(z_imag, 2));
}

void inferRLC() {
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 50);
  tft.println("Inferring RLC Parameters:");
  tft.printf("Known Internal Resistance (Rint): %.3f Ohm\n", Rint);

  measuredFrequencies.clear();
  measuredImpedances.clear();

  // Perform synthetic frequency sweep with controlled burst durations
  for (const auto& config : burstConfigurations) {
    float frequency = config.frequency;
    int durationUs = config.durationUs;
    tft.printf("Testing Freq: %.0f Hz, Dur: %d us...\n", frequency, durationUs);
    generateBurstPulse(frequency, durationUs);
    float avgVoltageRaw = measureAverageBatteryVoltage();
    float avgCurrentVoltageRaw = measureAverageShuntVoltage();

    float avgVoltage = avgVoltageRaw * (3.3 / 4095.0); // Convert to Volts
    float avgCurrent = (avgCurrentVoltageRaw * (3.3 / 4095.0)) / shuntResistance; // Convert to Amps

    if (avgCurrent > 1e-6) {
      measuredFrequencies.push_back(frequency);
      measuredImpedances.push_back(avgVoltage / avgCurrent);
      Serial.printf("Freq: %.0f Hz, Dur: %d us, Avg V: %.3f V, Avg I: %.3f A, |Z|: %.3f Ohm\n", frequency, durationUs, avgVoltage, avgCurrent, avgVoltage / avgCurrent);
    } else {
      Serial.printf("Freq: %.0f Hz, Dur: %d us, Current too low.\n", frequency, durationUs);
    }
    delay(100);
  }

  if (measuredFrequencies.size() < 3) {
    tft.println("Not enough impedance data for fitting.");
    return;
  }

  // Prepare data for Eigen
  Eigen::VectorXd measuredFreqEigen(measuredFrequencies.size());
  Eigen::VectorXd measuredImpEigen(measuredImpedances.size());
  for (size_t i = 0; i < measuredFrequencies.size(); ++i) {
    measuredFreqEigen(i) = measuredFrequencies[i];
    measuredImpEigen(i) = measuredImpedances[i];
  }

  // Define the cost function
  auto costFunction = [&](const Eigen::VectorXd& params) {
    float inductance = params(0);
    float capacitance = params(1);
    double sumSquaredErrors = 0.0;
    for (int i = 0; i < measuredFreqEigen.size(); ++i) {
      float modelImpedance = impedanceModel(measuredFreqEigen(i), inductance, capacitance);
      sumSquaredErrors += pow(modelImpedance - measuredImpEigen(i), 2);
    }
    return sumSquaredErrors;
  };

  // Initial guesses
  Eigen::VectorXd initialParams(2);
  initialParams(0) = 1e-6;
  initialParams(1) = 1e-3;

  // Optimization (Gradient Descent)
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

  tft.printf("Estimated Inductance (L): %.3e H\n", inferredInductance);
  tft.printf("Estimated Capacitance (C): %.3e F\n", inferredCapacitance);
}
