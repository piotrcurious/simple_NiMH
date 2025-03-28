#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <arduinoFFT.h>
#include <ArduinoEigenDense.h>

// TFT Display Configuration
TFT_eSPI tft = TFT_eSPI();

// Pin Definitions
const int transistorBasePin = 19;     // ESP32 GPIO pin to control the transistor base
const int batteryVoltagePin = 39; // ESP32 GPIO pin to measure VCC - battery voltage (ADC1)
const int resistorVoltagePin = 34; // ESP32 GPIO pin to measure voltage across the shunt resistor (ADC1)
const int vccVoltagePin = 35;    // ESP32 GPIO pin to measure VCC voltage via 1:1 divider (ADC1)

// Shunt Resistor Value
const float shuntResistance = 2.5; // Ohms

// Capacitor Value (shunting the resistor)
const float shuntCapacitance = 2000e-6; // Farads

// Frequency Sweep Parameters
const int startFrequency = 1;         // Hz
const int endFrequency = 500;         // Hz
const int frequencySteps = 50;          // Number of frequency points to sweep

// Data Logging Arrays
const int maxDataPoints = frequencySteps;
float frequencies[maxDataPoints];
float impedanceMagnitudes[maxDataPoints]; // Impedance magnitude
float impedancePhases[maxDataPoints];   // Impedance phase (degrees)

// FFT Parameters (for measuring AC components during pulse)
const double sampleRate = 2000; // Samples per second (adjust as needed)
const int samplesPerPulse = 256; // Number of samples during one pulse
double vRealBatteryDiffPulse[samplesPerPulse];
double vImagBatteryDiffPulse[samplesPerPulse];
double vRealResistorPulse[samplesPerPulse];
double vImagResistorPulse[samplesPerPulse];
double vRealVCCPulse[samplesPerPulse];
double vImagVCCPulse[samplesPerPulse];
ArduinoFFT FFT_pulse_battery(vRealBatteryDiffPulse, vImagBatteryDiffPulse, samplesPerPulse, sampleRate);
ArduinoFFT FFT_pulse_resistor(vRealResistorPulse, vImagResistorPulse, samplesPerPulse, sampleRate);
ArduinoFFT FFT_pulse_vcc(vRealVCCPulse, vImagVCCPulse, samplesPerPulse, sampleRate);

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(float* freqData, float* magData, int numPoints);
void inferRLC();
float impedanceModel(float frequency, float inductance, float capacitance);

void setup() {
  Serial.begin(115200);
  setupTFT();
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW); // Initially turn off the transistor

  // Configure ADC resolution (optional, but can improve accuracy)
  analogReadResolution(12); // 12-bit resolution (0-4095)
  pinMode(batteryVoltagePin, INPUT);
  pinMode(resistorVoltagePin, INPUT);
  pinMode(vccVoltagePin, INPUT); // Initialize VCC voltage pin as input
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Starting Frequency Sweep (Pulse Data)...");

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
    long samplingIntervalUs = halfPeriodUs / samplesPerPulse;

    // Collect data during the HIGH pulse
    digitalWrite(transistorBasePin, HIGH);
    for (int j = 0; j < samplesPerPulse; j++) {
      delayMicroseconds(samplingIntervalUs);
      vRealBatteryDiffPulse[j] = analogRead(batteryVoltagePin);
      vRealResistorPulse[j] = analogRead(resistorVoltagePin);
      vRealVCCPulse[j] = analogRead(vccVoltagePin);
      vImagBatteryDiffPulse[j] = 0;
      vImagResistorPulse[j] = 0;
      vImagVCCPulse[j] = 0;
    }
    digitalWrite(transistorBasePin, LOW);
    delayMicroseconds(halfPeriodUs); // Wait for the LOW pulse

    // Perform FFT analysis for battery voltage difference during the pulse
    FFT_pulse_battery.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_pulse_battery.compute(vRealBatteryDiffPulse, vImagBatteryDiffPulse, samplesPerPulse, FFT_FORWARD);

    // Perform FFT analysis for resistor voltage during the pulse
    FFT_pulse_resistor.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_pulse_resistor.compute(vRealResistorPulse, vImagResistorPulse, samplesPerPulse, FFT_FORWARD);

    // Perform FFT analysis for VCC voltage during the pulse
    FFT_pulse_vcc.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_pulse_vcc.compute(vRealVCCPulse, vImagVCCPulse, samplesPerPulse, FFT_FORWARD);

    // Find the bin corresponding to the fundamental frequency
    int fundamentalBin = round((float)frequency * samplesPerPulse / sampleRate);
    if (fundamentalBin > samplesPerPulse / 2 - 1) fundamentalBin = samplesPerPulse / 2 - 1; // Limit bin

    // Get complex values at the fundamental frequency
    std::complex<double> vcc_complex(vRealVCCPulse[fundamentalBin], vImagVCCPulse[fundamentalBin]);
    std::complex<double> diff_complex(vRealBatteryDiffPulse[fundamentalBin], vImagBatteryDiffPulse[fundamentalBin]);
    std::complex<double> resistor_complex(vRealResistorPulse[fundamentalBin], vImagResistorPulse[fundamentalBin]);

    // Calculate complex amplitudes (scaling factor: 2/N * Vref / 4095)
    double scalingFactor = 2.0 / samplesPerPulse * (3.3 / 4095.0);
    std::complex<double> vcc_amplitude = vcc_complex * scalingFactor * 2.0; // VCC has divider
    std::complex<double> diff_amplitude = diff_complex * scalingFactor;
    std::complex<double> resistor_amplitude = resistor_complex * scalingFactor;
    std::complex<double> battery_amplitude = vcc_amplitude - diff_amplitude;

    // Calculate complex current through the shunt (considering capacitor)
    double omega = 2.0 * PI * frequency;
    std::complex<double> shunt_impedance(shuntResistance, -1.0 / (omega * shuntCapacitance));
    std::complex<double> current_amplitude = resistor_amplitude / shunt_impedance;

    // Calculate complex impedance of the battery
    std::complex<double> battery_impedance(0, 0);
    if (std::abs(current_amplitude) > 1e-9) {
      battery_impedance = battery_amplitude / current_amplitude;
    }

    // Store magnitude and phase
    impedanceMagnitudes[i] = std::abs(battery_impedance);
    impedancePhases[i] = std::arg(battery_impedance) * 180.0 / PI; // Phase in degrees

    frequencies[i] = frequency;

    Serial.printf("Frequency: %.2f Hz\n", frequency);
    Serial.printf("  VCC Voltage (Complex): %.3f + j%.3f V\n", vcc_amplitude.real(), vcc_amplitude.imag());
    Serial.printf("  Battery Diff Voltage (Complex): %.3f + j%.3f V\n", diff_amplitude.real(), diff_amplitude.imag());
    Serial.printf("  Battery Voltage (Complex): %.3f + j%.3f V\n", battery_amplitude.real(), battery_amplitude.imag());
    Serial.printf("  Current (Complex): %.3f + j%.3f A\n", current_amplitude.real(), current_amplitude.imag());
    Serial.printf("  Impedance (Complex): %.3f + j%.3f Ohm\n", battery_impedance.real(), battery_impedance.imag());
    Serial.printf("  Impedance Magnitude: %.3f Ohm\n", impedanceMagnitudes[i]);
    Serial.printf("  Impedance Phase: %.3f degrees\n", impedancePhases[i]);

    frequency = startFrequency + (endFrequency - startFrequency) * (float)(i + 1) / frequencySteps;
    delay(10); // Small delay
  }

  // Plot Bode Plot (Magnitude)
  plotBode(frequencies, impedanceMagnitudes, maxDataPoints);
  delay(5000);

  // Plot Phase Plot
  plotGraph("Phase vs. Frequency", frequencies, impedancePhases, maxDataPoints, "Frequency (Hz)", "Phase (Degrees)");
  delay(5000);

  // Infer RLC parameters using fitting (can be extended to use phase)
  inferRLC();
  delay(5000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2 - 10);
  tft.println("Sweep Completed (Pulse Data)!");
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
  tft.drawLine(20, tft.height() - 30, 20, 30, TFT_WHITE);                                     // Y-axis

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
    measuredImpedances(i) = impedanceMagnitudes[i]; // Use magnitude for fitting
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
