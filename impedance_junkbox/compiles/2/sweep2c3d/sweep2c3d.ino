#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <arduinoFFT.h>
#include <ArduinoEigenDense.h>

// TFT Display Configuration
TFT_eSPI tft = TFT_eSPI();

// Pin Definitions
const int transistorBasePin = 19;
const int batteryVoltagePin = 39;
const int resistorVoltagePin = 34;
const int vccVoltagePin = 35;

// Shunt Resistor Value
const float shuntResistance = 2.5; // Ohms

// Capacitor Value (shunting the resistor)
const float shuntCapacitance = 2000e-6; // Farads

// Frequency Sweep Parameters
const int startFrequency = 10;
const int endFrequency = 1000; // Limiting for now to manage data size
const int frequencySteps = 100;    // Reduced for testing

// Duty Cycle Sweep Parameters
const float dutyCycles[] = {0.1, 0.3, 0.4, 0.5, 0.7};
const int numDutyCycles = sizeof(dutyCycles) / sizeof(dutyCycles[0]);

// Sampling Parameters
const double sampleRate = 2000; // Samples per second
const int samplesPerCycle = 128; // Samples per cycle

// Data Logging Arrays (Time Domain) - REMOVED TO SAVE MEMORY
const int maxFrequencies = frequencySteps;
float sweptFrequencies[maxFrequencies];

// Inferred Parameters
float inferredResistance[maxFrequencies][numDutyCycles];
float inferredCapacitance[maxFrequencies][numDutyCycles];

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(float* freqData, float* magData, int numPoints);
void inferRLC();
float impedanceModel(float frequency, float inductance, float capacitance);
void analyzeTimeDomainData(int freqIndex, int dutyIndex, float vcc[], float batteryDiff[], float resistorVoltage[]);

void setup() {
  Serial.begin(115200);
  setupTFT();
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW);

  analogReadResolution(12);
  pinMode(batteryVoltagePin, INPUT);
  pinMode(resistorVoltagePin, INPUT);
  pinMode(vccVoltagePin, INPUT);
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Starting Frequency and Duty Cycle Sweep...");

  float frequency = startFrequency;
  for (int freqIndex = 0; freqIndex < frequencySteps; freqIndex++) {
    sweptFrequencies[freqIndex] = frequency;
    Serial.printf("Frequency: %.2f Hz\n", frequency);
    tft.setCursor(0, 20 + freqIndex * 5);
    tft.printf("Freq: %.0f Hz...\n", frequency);

    for (int dutyIndex = 0; dutyIndex < numDutyCycles; dutyIndex++) {
      float dutyCycle = dutyCycles[dutyIndex];
      Serial.printf("  Duty Cycle: %.2f%%\n", dutyCycle * 100);

      long periodUs = 1000000 / (long)frequency;
      long onTimeUs = periodUs * dutyCycle;
      long offTimeUs = periodUs - onTimeUs;
      long samplingIntervalUs = periodUs / samplesPerCycle;

      float vccPulse[samplesPerCycle];
      float batteryDiffPulse[samplesPerCycle];
      float resistorVoltagePulse[samplesPerCycle];

      for (int sampleIndex = 0; sampleIndex < samplesPerCycle; sampleIndex++) {
        if ((float)sampleIndex / samplesPerCycle < dutyCycle) {
          digitalWrite(transistorBasePin, HIGH);
          delayMicroseconds(max(1L, min((long)samplingIntervalUs, onTimeUs))); // Ensure at least 1 us delay
        } else {
          digitalWrite(transistorBasePin, LOW);
          delayMicroseconds(max(1L, min((long)samplingIntervalUs, offTimeUs))); // Ensure at least 1 us delay
        }

        vccPulse[sampleIndex] = analogRead(vccVoltagePin);
        batteryDiffPulse[sampleIndex] = analogRead(batteryVoltagePin);
        resistorVoltagePulse[sampleIndex] = analogRead(resistorVoltagePin);
        delayMicroseconds(1); // Small delay to ensure sampling occurs
      }
      digitalWrite(transistorBasePin, LOW); // Ensure transistor is off at the end of cycle
      delay(1);

      // Analyze the captured time-domain data
      analyzeTimeDomainData(freqIndex, dutyIndex, vccPulse, batteryDiffPulse, resistorVoltagePulse);
    }

    frequency = startFrequency + (endFrequency - startFrequency) * (float)(freqIndex + 1) / frequencySteps;
    delay(10);
  }

  // Plot Inferred Resistance vs. Frequency for a specific duty cycle
  int dutyCycleToPlotR = 1; // Example: 30% duty cycle
  float resistanceToPlot[maxFrequencies];
  for (int i = 0; i < maxFrequencies; i++) {
    resistanceToPlot[i] = inferredResistance[i][dutyCycleToPlotR];
  }
  plotGraph("Inferred Resistance vs. Frequency (30% Duty)", sweptFrequencies, resistanceToPlot, frequencySteps, "Frequency (Hz)", "Resistance (Ohm)");
  delay(5000);

  // Plot Inferred Capacitance vs. Frequency for a specific duty cycle
  int dutyCycleToPlotC = 1; // Example: 30% duty cycle
  float capacitanceToPlot[maxFrequencies];
  for (int i = 0; i < maxFrequencies; i++) {
    capacitanceToPlot[i] = inferredCapacitance[i][dutyCycleToPlotC];
  }
  plotGraph("Inferred Capacitance vs. Frequency (30% Duty)", sweptFrequencies, capacitanceToPlot, frequencySteps, "Frequency (Hz)", "Capacitance (F)");
  delay(5000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2 - 10);
  tft.println("Sweep Completed (Time Domain Analysis)!");
  delay(5000);
}

void analyzeTimeDomainData(int freqIndex, int dutyIndex, float vccADC[], float batteryDiffADC[], float resistorVoltageADC[]) {
  float vcc[samplesPerCycle];
  float batteryDiff[samplesPerCycle];
  float resistorVoltage[samplesPerCycle];
  float current[samplesPerCycle];
  float batteryVoltage[samplesPerCycle];

  for (int i = 0; i < samplesPerCycle; i++) {
    vcc[i] = vccADC[i] * (3.3 / 4095.0) * 2.0;
    batteryDiff[i] = batteryDiffADC[i] * (3.3 / 4095.0);
    resistorVoltage[i] = resistorVoltageADC[i] * (3.3 / 4095.0);
    current[i] = resistorVoltage[i] / shuntResistance;
    batteryVoltage[i] = vcc[i] - batteryDiff[i];
  }

  int onSamples = samplesPerCycle * dutyCycles[dutyIndex];
  int offSamples = samplesPerCycle - onSamples;

  // Estimate DC Resistance
  float avgBatteryVoltageOn = 0;
  float avgCurrentOn = 0;
  for (int i = (int)(onSamples * 0.1); i < (int)(onSamples * 0.9); i++) { // Use middle 80% for steady state
    avgBatteryVoltageOn += batteryVoltage[i];
    avgCurrentOn += current[i];
  }
  if ((int)(onSamples * 0.8) > 0) {
    avgBatteryVoltageOn /= (onSamples * 0.8);
    avgCurrentOn /= (onSamples * 0.8);
  }

  float avgBatteryVoltageOff = 0;
  float avgCurrentOff = 0;
  for (int i = onSamples + (int)(offSamples * 0.1); i < onSamples + (int)(offSamples * 0.9); i++) { // Use middle 80% for steady state
    avgBatteryVoltageOff += batteryVoltage[i];
    avgCurrentOff += current[i];
  }
  if ((int)(offSamples * 0.8) > 0) {
    avgBatteryVoltageOff /= (offSamples * 0.8);
    avgCurrentOff /= (offSamples * 0.8);
  }

  if (abs(avgCurrentOn - avgCurrentOff) > 1e-6) {
    inferredResistance[freqIndex][dutyIndex] = (avgBatteryVoltageOn - avgBatteryVoltageOff) / (avgCurrentOn - avgCurrentOff);
  } else {
    inferredResistance[freqIndex][dutyIndex] = NAN;
  }

  // Estimate Capacitance (looking at the voltage slope during ON)
  int capacitanceStartIndex = 10; // Adjust as needed
  int capacitanceEndIndex = min(capacitanceStartIndex + 20, onSamples - 1);
  if (capacitanceStartIndex < capacitanceEndIndex) {
    float deltaV = batteryVoltage[capacitanceEndIndex] - batteryVoltage[capacitanceStartIndex];
    float deltaTime = (capacitanceEndIndex - capacitanceStartIndex) * (1.0 / sampleRate);
    float avgCurrent = 0;
    for (int i = capacitanceStartIndex; i <= capacitanceEndIndex; i++) {
      avgCurrent += current[i];
    }
    avgCurrent /= (capacitanceEndIndex - capacitanceStartIndex + 1);
    if (abs(deltaTime) > 1e-9 && abs(deltaV) > 1e-6) {
      inferredCapacitance[freqIndex][dutyIndex] = avgCurrent / (deltaV / deltaTime);
    } else {
      inferredCapacitance[freqIndex][dutyIndex] = NAN;
    }
  } else {
    inferredCapacitance[freqIndex][dutyIndex] = NAN;
  }

  Serial.printf("    Inferred Resistance (DC Approx): %.3f Ohm\n", inferredResistance[freqIndex][dutyIndex]);
  Serial.printf("    Inferred Capacitance (Approx): %.3e F\n", inferredCapacitance[freqIndex][dutyIndex]);
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

  if (numPoints <= 1) return;

  float minX = xData[0], maxX = xData[0];
  float minY = yData[0], maxY = yData[0];
  for (int i = 1; i < numPoints; i++) {
    minX = min(minX, xData[i]);
    maxX = max(maxX, xData[i]);
    minY = min(minY, yData[i]);
    maxY = max(maxY, yData[i]);
  }

  float xRange = maxX - minX;
  float yRange = maxY - minY;
  if (xRange == 0) xRange = 1;
  if (yRange == 0) yRange = 1;
  minX -= xRange * 0.1;
  maxX += xRange * 0.1;
  minY -= yRange * 0.1;
  maxY += yRange * 0.1;

  float xScale = (float)(tft.width() - 40) / (maxX - minX);
  float yScale = (float)(tft.height() - 60) / (maxY - minY);

  tft.drawLine(20, tft.height() - 30, tft.width() - 20, tft.height() - 30, TFT_WHITE);
  tft.drawLine(20, tft.height() - 30, 20, 30, TFT_WHITE);

  tft.setTextColor(TFT_GREEN);
  for (int i = 0; i < numPoints - 1; i++) {
    int x1 = 20 + (xData[i] - minX) * xScale;
    int y1 = tft.height() - 30 - (yData[i] - minY) * yScale;
    int x2 = 20 + (xData[i + 1] - minX) * xScale;
    int y2 = tft.height() - 30 - (yData[i + 1] - minY) * yScale;
    if (!isnan(y1) && !isnan(y2)) {
      tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
    }
  }

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

float impedanceModel(float frequency, float inductance, float capacitance) {
  float omega = 2 * PI * frequency;
  float z_imag = omega * inductance - 1.0 / (omega * capacitance);
  return sqrt(Rint * Rint + z_imag * z_imag);
}

void inferRLC() {
  // Placeholder function - can be implemented later if needed
}
