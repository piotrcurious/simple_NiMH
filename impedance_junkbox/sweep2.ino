#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <arduinoFFT.h>
#include <Eigen/Dense>

// TFT Display Configuration
TFT_eSPI tft = TFT_eSPI();

// Pin Definitions
const int transistorBasePin = 2;   // ESP32 GPIO pin to control the transistor base
const int batteryVoltagePin = 34; // ESP32 GPIO pin to measure battery voltage (ADC1)
const int resistorVoltagePin = 35; // ESP32 GPIO pin to measure voltage across the shunt resistor (ADC1)

// Shunt Resistor Value
const float shuntResistance = 2.5; // Ohms

// Capacitor Value (for reference, though we'll be measuring AC components)
const float shuntCapacitance = 2000e-6; // Farads

// Frequency Sweep Parameters
const int startFrequency = 100;     // Hz
const int endFrequency = 40000;   // Hz
const int frequencySteps = 50;     // Number of frequency points to sweep

// Data Logging Arrays
const int maxDataPoints = frequencySteps;
float frequencies[maxDataPoints];
float batteryVoltages[maxDataPoints]; // RMS voltage across the battery
float currents[maxDataPoints];        // RMS current through the battery
float impedances[maxDataPoints];      // Impedance magnitude

// FFT Parameters (for measuring AC components)
const int sampleRate = 80000; // Samples per second (adjust as needed)
const int numSamples = 1024;  // Number of samples for FFT (power of 2)
arduinoFFT FFT(numSamples);
double vRealBattery[numSamples];
double vImagBattery[numSamples];
double vRealResistor[numSamples];
double vImagResistor[numSamples];

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(float* freqData, float* magData, int numPoints);
float calculateRMS(int pin, double* vReal, double* vImag);
void inferRLC();
float impedanceModel(float frequency, float inductance, float capacitance);

void setup() {
  Serial.begin(115200);
  setupTFT();
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW); // Initially turn off the transistor

  // Configure ADC resolution (optional, but can improve accuracy)
  analogReadResolution(12); // 12-bit resolution (0-4095)
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Starting Frequency Sweep...");

  // Perform Frequency Sweep
  float frequency = startFrequency;
  for (int i = 0; i < frequencySteps; i++) {
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
    tft.setCursor(0, 20 + i * 5);
    tft.printf("Freq: %.0f Hz...", frequency);

    // Generate a square wave at the current frequency
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
      // No need to read again immediately, the low state completes the cycle
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
    float currentRMS = resistorVoltageRMS / shuntResistance;
    float impedance = 0;
    if (currentRMS > 1e-6) { // Avoid division by zero
      impedance = batteryVoltageRMS / currentRMS;
    }

    // Store the data
    frequencies[i] = frequency;
    batteryVoltages[i] = batteryVoltageRMS;
    currents[i] = currentRMS;
    impedances[i] = impedance;

    Serial.printf("  Battery Voltage (RMS): %.3f V\n", batteryVoltageRMS * (3.3 / 4095.0)); // Assuming 3.3V reference
    Serial.printf("  Current (RMS): %.3f A\n", currentRMS * (3.3 / 4095.0) / shuntResistance);
    Serial.printf("  Impedance: %.3f Ohm\n", impedance * (3.3 / 4095.0) / (currentRMS > 1e-6 ? (3.3 / 4095.0) / shuntResistance : 1)); // Scale back to voltage and current units

    frequency = startFrequency + (endFrequency - startFrequency) * (float)(i + 1) / frequencySteps;
    delay(10); // Small delay
  }

  // Plot Transfer Function (Impedance vs. Frequency)
  plotGraph("Impedance vs. Frequency", frequencies, impedances, maxDataPoints, "Frequency (Hz)", "Impedance (Ohm)");
  delay(5000);

  // Infer RLC parameters using fitting
  inferRLC();
  delay(5000);

  // Plot Bode Plot (Magnitude)
  plotBode(frequencies, impedances, maxDataPoints);
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

void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println(title);

  if (numPoints <= 1) return;

  // Find min and max values for autoscaling
  float minX = xData[0], maxX = xData[0];
  float minY = yData[0], maxY = yData[0];
  for (int i = 1; i < numPoints; i++) {
    minX = min(minX, xData[i]);
    maxX = max(maxX, xData[i]);
    minY = min(minY, yData[i]);
    maxY = max(maxY, yData[i]);
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
  for (int i = 0; i < numPoints - 1; i++) {
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

void plotBode(float* freqData, float* magData, int numPoints) {
  plotGraph("Bode Plot (Magnitude)", freqData, magData, numPoints, "Frequency (Hz)", "Impedance (Ohm)");
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

  if (maxDataPoints < 3) {
    tft.println("Not enough data points to perform fitting.");
    return;
  }

  // Prepare data for Eigen
  Eigen::VectorXd measuredFrequencies(maxDataPoints);
  Eigen::VectorXd measuredImpedances(maxDataPoints);
  for (int i = 0; i < maxDataPoints; ++i) {
    measuredFrequencies(i) = frequencies[i];
    measuredImpedances(i) = impedances[i];
  }

  // Define a cost function (sum of squared errors)
  auto costFunction = [&](const Eigen::VectorXd& params) {
    float inductance = params(0);
    float capacitance = params(1);
    double sumSquaredErrors = 0.0;
    for (int i = 0; i < maxDataPoints; ++i) {
      float modelImpedance = impedanceModel(measuredFrequencies(i), inductance, capacitance);
      sumSquaredErrors += pow(modelImpedance - measuredImpedances(i), 2);
    }
    return sumSquaredErrors;
  };

  // Initial guesses for inductance and capacitance
  Eigen::VectorXd initialParams(2);
  initialParams(0) = 1e-6;  // Initial guess for Inductance (1 uH)
  initialParams(1) = 1e-3;  // Initial guess for Capacitance (1 mF)

  // Use a simple iterative optimization (gradient descent - can be improved)
  double learningRate = 1e-9;
  int iterations = 1000;

  Eigen::VectorXd currentParams = initialParams;
  for (int iter = 0; iter < iterations; ++iter) {
    float inductance = currentParams(0);
    float capacitance = currentParams(1);

    // Calculate gradients numerically (can be done analytically for better performance)
    double delta = 1e-9;
    Eigen::VectorXd gradient(2);

    Eigen::VectorXd paramsPlusDeltaL = currentParams;
    paramsPlusDeltaL(0) += delta;
    gradient(0) = (costFunction(paramsPlusDeltaL) - costFunction(currentParams)) / delta;

    Eigen::VectorXd paramsPlusDeltaC = currentParams;
    paramsPlusDeltaC(1) += delta;
    gradient(1) = (costFunction(paramsPlusDeltaC) - costFunction(currentParams)) / delta;

    // Update parameters
    currentParams -= learningRate * gradient;

    // Keep inductance and capacitance non-negative
    currentParams(0) = fmax(currentParams(0), 0.0f);
    currentParams(1) = fmax(currentParams(1), 1e-12f); // Avoid division by zero
  }

  float inferredInductance = currentParams(0);
  float inferredCapacitance = currentParams(1);

  tft.printf("Inferred Inductance (L): %.3e H\n", inferredInductance);
  tft.printf("Inferred Capacitance (C): %.3e F\n", inferredCapacitance);
}
