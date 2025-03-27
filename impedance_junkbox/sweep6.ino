#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <arduinoFFT.h>
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

// Capacitor Value (for reference)
const float shuntCapacitance = 2000e-6; // Farads

// Initial Frequency Sweep Parameters
const int initialStartFrequency = 100;   // Hz
const int initialEndFrequency = 40000; // Hz
const int initialFrequencySteps = 30;  // Fewer steps for the initial sweep

// FFT Parameters
const int sampleRate = 80000; // Samples per second (adjust as needed)
const int numSamples = 1024;  // Number of samples for FFT (power of 2)
arduinoFFT FFT(numSamples);
double vRealBattery[numSamples];
double vImagBattery[numSamples];
double vRealResistor[numSamples];
double vImagResistor[numSamples];

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Data Storage for Measurements
std::vector<float> measuredFrequencies;
std::vector<float> measuredImpedances;

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData);
float measureImpedance(float frequency);
float impedanceModel(float frequency, float inductance, float capacitance);
void inferRLC();
std::vector<float> linspace(float start, float end, int num);

void setup() {
  Serial.begin(115200);
  setupTFT();
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW); // Initially turn off the transistor

  // Configure ADC resolution
  analogReadResolution(12);
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Starting Initial Frequency Sweep...");

  measuredFrequencies.clear();
  measuredImpedances.clear();

  // Perform Initial Frequency Sweep
  std::vector<float> initialFrequencies = linspace(initialStartFrequency, initialEndFrequency, initialFrequencySteps);
  for (float freq : initialFrequencies) {
    float impedance = measureImpedance(freq);
    if (impedance > 0) {
      measuredFrequencies.push_back(freq);
      measuredImpedances.push_back(impedance);
    }
    tft.setCursor(0, 20 + measuredFrequencies.size() * 5);
    tft.printf("Initial Freq: %.0f Hz, Z: %.2f Ohm", freq, impedance);
    Serial.printf("Initial Freq: %.0f Hz, Z: %.2f Ohm\n", freq, impedance);
    delay(10);
  }

  tft.println("\nAnalyzing Initial Data...");
  Serial.println("Analyzing Initial Data...");

  // Identify Critical Frequencies for Refinement
  std::vector<float> refinementFrequencies;

  // 1. Around the frequency of minimum impedance
  if (!measuredImpedances.empty()) {
    auto minIt = std::min_element(measuredImpedances.begin(), measuredImpedances.end());
    int minIndex = std::distance(measuredImpedances.begin(), minIt);
    float minFreq = measuredFrequencies[minIndex];
    refinementFrequencies.push_back(minFreq / 2.0); // Below resonance
    refinementFrequencies.push_back(minFreq);       // At resonance
    refinementFrequencies.push_back(minFreq * 2.0); // Above resonance
  }

  // 2. At the edges of the frequency range (if not already measured)
  if (!measuredFrequencies.empty()) {
    if (std::find(measuredFrequencies.begin(), measuredFrequencies.end(), initialStartFrequency) == measuredFrequencies.end()) {
      refinementFrequencies.push_back(initialStartFrequency);
    }
    if (std::find(measuredFrequencies.begin(), measuredFrequencies.end(), initialEndFrequency) == measuredFrequencies.end()) {
      refinementFrequencies.push_back(initialEndFrequency);
    }
  }

  // Remove duplicates and sort
  std::sort(refinementFrequencies.begin(), refinementFrequencies.end());
  refinementFrequencies.erase(std::unique(refinementFrequencies.begin(), refinementFrequencies.end()), refinementFrequencies.end());

  tft.println("\nPerforming Refinement Measurements...");
  Serial.println("Performing Refinement Measurements...");

  // Perform Refinement Measurements
  for (float freq : refinementFrequencies) {
    if (std::find(measuredFrequencies.begin(), measuredFrequencies.end(), freq) == measuredFrequencies.end()) {
      float impedance = measureImpedance(freq);
      if (impedance > 0) {
        measuredFrequencies.push_back(freq);
        measuredImpedances.push_back(impedance);
      }
      tft.printf("Refined Freq: %.0f Hz, Z: %.2f Ohm", freq, impedance);
      Serial.printf("Refined Freq: %.0f Hz, Z: %.2f Ohm\n", freq, impedance);
      delay(10);
    }
  }

  // Sort the combined data by frequency
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

  // Plot Transfer Function with all data
  plotGraph("Impedance vs. Frequency", measuredFrequencies, measuredImpedances, "Frequency (Hz)", "Impedance (Ohm)");
  delay(5000);

  // Infer RLC parameters using fitting with all data
  inferRLC();
  delay(5000);

  // Plot Bode Plot (Magnitude) with all data
  plotBode(measuredFrequencies, measuredImpedances);
  delay(10000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2 - 10);
  tft.println("Sweep Completed!");
  delay(5000);
}

void setupTFT() {
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1); // Adjust rotation as needed
}

void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println(title);

  if (xData.size() <= 1) return;

  // Find min and max values for autoscaling
  float minX = xData[0], maxX = xData[0];
  float minY = yData[0], maxY = yData[0];
  for (float val : xData) {
    minX = min(minX, val);
    maxX = max(maxX, val);
  }
  for (float val : yData) {
    minY = min(minY, val);
    maxY = max(maxY, val);
  }

  // Add some padding
  float xRange = maxX - minX;
  float yRange = maxY - minY;
  if (xRange == 0) xRange = 1;
  if (yRange == 0) yRange = 1;
  minX -= xRange * 0.1;
  maxX += xRange * 0.1;
  minY -= yRange * 0.1;
  maxY += yRange * 0.1;

  // Calculate scaling factors
  float xScale = (float)(tft.width() - 40) / (maxX - minX);
  float yScale = (float)(tft.height() - 60) / (maxY - minY);

  // Draw axes
  tft.drawLine(20, tft.height() - 30, tft.width() - 20, tft.height() - 30, TFT_WHITE); // X-axis
  tft.drawLine(20, tft.height() - 30, 20, 30, TFT_WHITE);                            // Y-axis

  // Plot data points
  tft.setTextColor(TFT_GREEN);
  for (size_t i = 0; i < xData.size() - 1; i++) {
    int x1 = 20 + (xData[i] - minX) * xScale;
    int y1 = tft.height() - 30 - (yData[i] - minY) * yScale;
    int x2 = 20 + (xData[i + 1] - minX) * xScale;
    int y2 = tft.height() - 30 - (yData[i + 1] - minY) * yScale;
    tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
  }

  // Draw axis labels
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(tft.width() / 2 - strlen(xAxisLabel) * 3, tft.height() - 20);
  tft.println(xAxisLabel);
  tft.setCursor(5, tft.height() / 2 - strlen(yAxisLabel) * 3);
  tft.println(yAxisLabel);
}

void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData) {
  plotGraph("Bode Plot (Magnitude)", freqData, magData, "Frequency (Hz)", "Impedance (Ohm)");
}

// Function to measure impedance at a given frequency
float measureImpedance(float frequency) {
  long halfPeriodUs = 500000 / (long)frequency; // Half period in microseconds

  // Collect data for FFT
  for (int j = 0; j < numSamples; j++) {
    digitalWrite(transistorBasePin, HIGH);
    delayMicroseconds(halfPeriodUs);
    vRealBattery[j] = analogRead(batteryVoltagePin);
    vRealResistor[j] = analogRead(resistorVoltagePin);
    vImagBattery[j] = 0;
    vImagResistor[j] = 0;
    digitalWrite(transistorBasePin, LOW);
    delayMicroseconds(halfPeriodUs);
  }

  // Perform FFT analysis
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vRealBattery, vImagBattery, numSamples);
  FFT.ComplexToMagnitude(vRealBattery, vImagBattery, numSamples);

  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vRealResistor, vImagResistor, numSamples);
  FFT.ComplexToMagnitude(vRealResistor, vImagResistor, numSamples);

  // Find the magnitude at the fundamental frequency
  int fundamentalBin = round((float)frequency * numSamples / sampleRate);
  float batteryVoltageRMS = vRealBattery[fundamentalBin] / (numSamples / 2); // Approximate RMS for fundamental
  float resistorVoltageRMS = vRealResistor[fundamentalBin] / (numSamples / 2); // Approximate RMS for fundamental

  // Calculate current and impedance
  float currentRMS = resistorVoltageRMS * (3.3 / 4095.0) / shuntResistance; // Convert to Volts
  float batteryVoltageRMS_V = batteryVoltageRMS * (3.3 / 4095.0);         // Convert to Volts
  float impedance = 0;
  if (currentRMS > 1e-6) {
    impedance = batteryVoltageRMS_V / currentRMS;
  }
  return impedance;
}

// Function to calculate the impedance magnitude based on the series RLC model
float impedanceModel(float frequency, float inductance, float capacitance) {
  float omega = 2 * PI * frequency;
  float z_imag = omega * inductance - 1.0 / (omega * capacitance);
  return sqrt(Rint * Rint + z_imag * z_imag);
}

void inferRLC() {
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 50);
  tft.println("Inferring RLC Parameters (using fitting):");
  tft.printf("Known Internal Resistance (Rint): %.3f Ohm\n", Rint);

  if (measuredFrequencies.size() < 3) {
    tft.println("Not enough data points to perform fitting.");
    return;
  }

  // Prepare data for Eigen
  Eigen::VectorXd measuredFreqEigen(measuredFrequencies.size());
  Eigen::VectorXd measuredImpEigen(measuredImpedances.size());
  for (size_t i = 0; i < measuredFrequencies.size(); ++i) {
    measuredFreqEigen(i) = measuredFrequencies[i];
    measuredImpEigen(i) = measuredImpedances[i];
  }

  // Define a cost function (sum of squared errors)
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

  // Initial guesses for inductance and capacitance
  Eigen::VectorXd initialParams(2);
  initialParams(0) = 1e-6;  // Initial guess for Inductance
  initialParams(1) = 1e-3;  // Initial guess for Capacitance

  // Use a simple iterative optimization (gradient descent)
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

// Helper function to create a linearly spaced vector
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
