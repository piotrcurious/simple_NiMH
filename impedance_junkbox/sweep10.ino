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

// FFT Parameters
const int sampleRate = 80000; // Samples per second (adjust as needed)
const int numSamples = 2048;  // Increase number of samples for better frequency resolution
arduinoFFT FFT_battery(numSamples);
arduinoFFT FFT_resistor(numSamples);
double vRealBattery[numSamples];
double vImagBattery[numSamples];
double vRealResistor[numSamples];
double vImagResistor[numSamples];

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Data Storage for Measurements (Frequency, Impedance Magnitude)
std::vector<float> measuredFrequencies;
std::vector<float> measuredImpedances;

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, const std::vector<float>& xData, const std::vector<float>& yData, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(const std::vector<float>& freqData, const std::vector<float>& magData);
std::map<float, float> measureHarmonicImpedance(float fundamentalFrequency, float dutyCycle);
float impedanceModel(float frequency, float inductance, float capacitance);
void inferRLC();

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
  tft.println("Starting Harmonic Frequency Sweep...");

  measuredFrequencies.clear();
  measuredImpedances.clear();

  // Sweep through different fundamental frequencies and duty cycles
  std::vector<float> fundamentalFrequencies = {100, 500, 2000, 10000}; // Example frequencies
  std::vector<float> dutyCycles = {0.25, 0.5, 0.75};                   // Example duty cycles

  for (float fundamentalFreq : fundamentalFrequencies) {
    for (float dutyCycle : dutyCycles) {
      Serial.printf("Fundamental Freq: %.0f Hz, Duty Cycle: %.2f\n", fundamentalFreq, dutyCycle);
      tft.printf("Freq: %.0f Hz, DC: %.2f...", fundamentalFreq, dutyCycle);

      std::map<float, float> harmonicImpedances = measureHarmonicImpedance(fundamentalFreq, dutyCycle);

      for (const auto& pair : harmonicImpedances) {
        float frequency = pair.first;
        float impedance = pair.second;
        measuredFrequencies.push_back(frequency);
        measuredImpedances.push_back(impedance);
        Serial.printf("  Harmonic Freq: %.0f Hz, Z: %.2f Ohm\n", frequency, impedance);
      }
      delay(100);
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
  plotGraph("Impedance vs. Frequency (Harmonics)", measuredFrequencies, measuredImpedances, "Frequency (Hz)", "Impedance (Ohm)");
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

// Function to measure impedance at multiple harmonics of a given fundamental frequency and duty cycle
std::map<float, float> measureHarmonicImpedance(float fundamentalFrequency, float dutyCycle) {
  std::map<float, float> harmonicImpedances;
  long periodUs = 1000000 / (long)fundamentalFrequency;
  long highTimeUs = periodUs * dutyCycle;

  // Collect time-domain data
  for (int i = 0; i < numSamples; i++) {
    digitalWrite(transistorBasePin, HIGH);
    delayMicroseconds(highTimeUs);
    vRealBattery[i] = analogRead(batteryVoltagePin);
    vRealResistor[i] = analogRead(resistorVoltagePin);
    vImagBattery[i] = 0;
    vImagResistor[i] = 0;
    digitalWrite(transistorBasePin, LOW);
    delayMicroseconds(periodUs - highTimeUs);
  }

  // Perform FFT analysis on battery voltage
  FFT_battery.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT_battery.Compute(vRealBattery, vImagBattery, numSamples);
  FFT_battery.ComplexToMagnitude(vRealBattery, vImagBattery, numSamples);

  // Perform FFT analysis on resistor voltage
  FFT_resistor.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT_resistor.Compute(vRealResistor, vImagResistor, numSamples);
  FFT_resistor.ComplexToMagnitude(vRealResistor, vImagResistor, numSamples);

  // Calculate impedance for the fundamental and a few harmonics
  for (int harmonic = 1; harmonic <= 10; ++harmonic) { // Analyze up to 10th harmonic
    float frequency = fundamentalFrequency * harmonic;
    int bin = round(frequency * numSamples / sampleRate);

    if (bin > 0 && bin < numSamples / 2) {
      float batteryVoltageRMS = vRealBattery[bin] / (numSamples / 2); // Approximate RMS
      float resistorVoltageRMS = vRealResistor[bin] / (numSamples / 2); // Approximate RMS

      float currentRMS = resistorVoltageRMS * (3.3 / 4095.0) / shuntResistance; // Convert to Amps
      float batteryVoltageRMS_V = batteryVoltageRMS * (3.3 / 4095.0);         // Convert to Volts

      if (currentRMS > 1e-6) {
        harmonicImpedances[frequency] = batteryVoltageRMS_V / currentRMS;
      }
    }
  }
  return harmonicImpedances;
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
  tft.println("Inferring RLC Parameters (using harmonic fitting):");
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
    for (int i = 0; i < measuredFreqEigen.size(); ++i) {
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
  double learningRate = 1e-11;
  int iterations = 3000;

  Eigen::VectorXd currentParams = initialParams;
  for (int iter = 0; iter < iterations; ++iter) {
    float inductance = currentParams(0);
    float capacitance = currentParams(1);

    double delta = 1e-11;
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
