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
const int startFrequency = 10;         // Hz
const int endFrequency = 1000;         // Hz
const int frequencySteps = 50;          // Number of frequency points to sweep

// Data Logging Arrays
const int maxDataPoints = frequencySteps;
float frequencies[maxDataPoints];
float impedanceMagnitudes[maxDataPoints]; // Impedance magnitude
float impedancePhases[maxDataPoints];   // Impedance phase (degrees)

// FFT Parameters (for measuring AC components during pulse)
const double sampleRate = 2000; // Samples per second (adjust as needed)
const int samplesPerPulse = 256; // Number of samples during one pulse
const int numAverages = 5;       // Number of FFT averages

// Arrays for single pulse capture
double vRealBatteryDiffPulseSingle[samplesPerPulse];
double vImagBatteryDiffPulseSingle[samplesPerPulse];
double vRealResistorPulseSingle[samplesPerPulse];
double vImagResistorPulseSingle[samplesPerPulse];
double vRealVCCPulseSingle[samplesPerPulse];
double vImagVCCPulseSingle[samplesPerPulse];

// FFT Objects
ArduinoFFT FFT_pulse_battery(vRealBatteryDiffPulseSingle, vImagBatteryDiffPulseSingle, samplesPerPulse, sampleRate);
ArduinoFFT FFT_pulse_resistor(vRealResistorPulseSingle, vImagResistorPulseSingle, samplesPerPulse, sampleRate);
ArduinoFFT FFT_pulse_vcc(vRealVCCPulseSingle, vImagVCCPulseSingle, samplesPerPulse, sampleRate);

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
  tft.println("Starting Frequency Sweep (Pulse Data with Averaging)...");

  // Perform Frequency Sweep
  float frequency = startFrequency;
  for (int i = 0; i < frequencySteps; i++) {
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
    tft.setCursor(0, 20 + i * 5);
    tft.printf("Freq: %.0f Hz...", frequency);

    long halfPeriodUs = 500000 / (long)frequency;
    long samplingIntervalUs = halfPeriodUs / samplesPerPulse;

    std::complex<double> vcc_amplitude_avg(0, 0);
    std::complex<double> diff_amplitude_avg(0, 0);
    std::complex<double> resistor_amplitude_avg(0, 0);

    for (int avg = 0; avg < numAverages; avg++) {
      // Collect data during the HIGH pulse
      digitalWrite(transistorBasePin, HIGH);
      for (int j = 0; j < samplesPerPulse; j++) {
        delayMicroseconds(samplingIntervalUs);
        vRealBatteryDiffPulseSingle[j] = analogRead(batteryVoltagePin);
        vRealResistorPulseSingle[j] = analogRead(resistorVoltagePin);
        vRealVCCPulseSingle[j] = analogRead(vccVoltagePin);
        vImagBatteryDiffPulseSingle[j] = 0;
        vImagResistorPulseSingle[j] = 0;
        vImagVCCPulseSingle[j] = 0;
      }
      digitalWrite(transistorBasePin, LOW);
      delayMicroseconds(halfPeriodUs);

      // Perform FFT analysis
      FFT_pulse_battery.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT_pulse_battery.compute(vRealBatteryDiffPulseSingle, vImagBatteryDiffPulseSingle, samplesPerPulse, FFT_FORWARD);
      FFT_pulse_resistor.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT_pulse_resistor.compute(vRealResistorPulseSingle, vImagResistorPulseSingle, samplesPerPulse, FFT_FORWARD);
      FFT_pulse_vcc.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT_pulse_vcc.compute(vRealVCCPulseSingle, vImagVCCPulseSingle, samplesPerPulse, FFT_FORWARD);

      int fundamentalBin = round((float)frequency * samplesPerPulse / sampleRate);
      if (fundamentalBin > samplesPerPulse / 2 - 1) fundamentalBin = samplesPerPulse / 2 - 1;

      double scalingFactor = 2.0 / samplesPerPulse * (3.3 / 4095.0);
      std::complex<double> vcc_complex(vRealVCCPulseSingle[fundamentalBin], vImagVCCPulseSingle[fundamentalBin]);
      std::complex<double> diff_complex(vRealBatteryDiffPulseSingle[fundamentalBin], vImagBatteryDiffPulseSingle[fundamentalBin]);
      std::complex<double> resistor_complex(vRealResistorPulseSingle[fundamentalBin], vImagResistorPulseSingle[fundamentalBin]);

      vcc_amplitude_avg += vcc_complex * scalingFactor * 2.0;
      diff_amplitude_avg += diff_complex * scalingFactor;
      resistor_amplitude_avg += resistor_complex * scalingFactor;
      delay(5); // Small delay between averages
    }

    // Calculate the average complex amplitudes
    vcc_amplitude_avg /= numAverages;
    diff_amplitude_avg /= numAverages;
    resistor_amplitude_avg /= numAverages;

    std::complex<double> battery_amplitude = vcc_amplitude_avg - diff_amplitude_avg;

    double omega = 2.0 * PI * frequency;
    std::complex<double> shunt_impedance(shuntResistance, -1.0 / (omega * shuntCapacitance));
    std::complex<double> current_amplitude = resistor_amplitude_avg / shunt_impedance;

    std::complex<double> battery_impedance(0, 0);
    if (std::abs(current_amplitude) > 1e-9) {
      battery_impedance = battery_amplitude / current_amplitude;
    }

    impedanceMagnitudes[i] = std::abs(battery_impedance);
    impedancePhases[i] = std::arg(battery_impedance) * 180.0 / PI;

    frequencies[i] = frequency;

    Serial.printf("Frequency: %.2f Hz\n", frequency);
    Serial.printf("  VCC Voltage (Complex): %.3f + j%.3f V\n", vcc_amplitude_avg.real(), vcc_amplitude_avg.imag());
    Serial.printf("  Battery Diff Voltage (Complex): %.3f + j%.3f V\n", diff_amplitude_avg.real(), diff_amplitude_avg.imag());
    Serial.printf("  Battery Voltage (Complex): %.3f + j%.3f V\n", battery_amplitude.real(), battery_amplitude.imag());
    Serial.printf("  Current (Complex): %.3f + j%.3f A\n", current_amplitude.real(), current_amplitude.imag());
    Serial.printf("  Impedance (Complex): %.3f + j%.3f Ohm\n", battery_impedance.real(), battery_impedance.imag());
    Serial.printf("  Impedance Magnitude: %.3f Ohm\n", impedanceMagnitudes[i]);
    Serial.printf("  Impedance Phase: %.3f degrees\n", impedancePhases[i]);

    frequency = startFrequency + (endFrequency - startFrequency) * (float)(i + 1) / frequencySteps;
    delay(10);
  }

  plotBode(frequencies, impedanceMagnitudes, maxDataPoints);
  delay(5000);
  plotGraph("Phase vs. Frequency", frequencies, impedancePhases, maxDataPoints, "Frequency (Hz)", "Phase (Degrees)");
  delay(5000);
  inferRLC();
  delay(5000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2 - 10);
  tft.println("Sweep Completed (Pulse Data with Averaging)!");
  delay(5000);
}

void setupTFT() {
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
}

void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println(title);
  // ... (rest of the plotGraph function remains the same)
}

void plotBode(float* freqData, float* magData, int numPoints) {
  plotGraph("Bode Plot (Magnitude)", freqData, magData, numPoints, "Frequency (Hz)", "Impedance (Ohm)");
}

float impedanceModel(float frequency, float inductance, float capacitance) {
  float omega = 2 * PI * frequency;
  float z_imag = omega * inductance - 1.0 / (omega * capacitance);
  return sqrt(Rint * Rint + z_imag * z_imag);
}

void inferRLC() {
  // ... (rest of the inferRLC function remains the same)
}
