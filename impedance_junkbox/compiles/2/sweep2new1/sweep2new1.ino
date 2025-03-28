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
const int endFrequency = 1000;
const int frequencySteps = 100;

// Duty Cycle Sweep Parameters
const float dutyCycles[] = {0.1, 0.3, 0.4, 0.5, 0.7};
const int numDutyCycles = sizeof(dutyCycles) / sizeof(dutyCycles[0]);

// Sampling Parameters
const double sampleRate = 2000; // Samples per second
const int samplesPerCycle = 128; // Samples per cycle

// Inferred Parameters
const int maxFrequencies = frequencySteps;
float sweptFrequencies[maxFrequencies];
float inferredResistance[maxFrequencies][numDutyCycles];
float inferredCapacitance[maxFrequencies][numDutyCycles];
float inferredESR[maxFrequencies][numDutyCycles]; // Array for ESR

// Raw Data Storage (for visualization)
const int rawDataFrequencyIndex = 0; // Store raw data for the first frequency
const int rawDataDutyCycleIndex = 0; // Store raw data for the first duty cycle
float rawTime[samplesPerCycle];
float rawBatteryVoltage[samplesPerCycle];
float rawCurrent[samplesPerCycle];
bool storeRawData = true;

// Known Static Internal Resistance
const float Rint = 0.1; // Example value, replace with your known Rint

// Function Prototypes
void setupTFT();
void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel);
void plotBode(float* freqData, float* magData, int numPoints);
void inferRLC();
float impedanceModel(float frequency, float inductance, float capacitance);
void analyzeTimeDomainData(int freqIndex, int dutyIndex, float vcc[], float batteryDiff[], float resistorVoltage[]);
void plotRawData(float time[], float voltage[], float current[], int numPoints);

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
                    delayMicroseconds(max(1L, min((long)samplingIntervalUs, onTimeUs)));
                } else {
                    digitalWrite(transistorBasePin, LOW);
                    delayMicroseconds(max(1L, min((long)samplingIntervalUs, offTimeUs)));
                }

                vccPulse[sampleIndex] = analogRead(vccVoltagePin);
                batteryDiffPulse[sampleIndex] = analogRead(batteryVoltagePin);
                resistorVoltagePulse[sampleIndex] = analogRead(resistorVoltagePin);
                delayMicroseconds(1);

                if (storeRawData && freqIndex == rawDataFrequencyIndex && dutyIndex == rawDataDutyCycleIndex) {
                    rawTime[sampleIndex] = sampleIndex * (1.0 / sampleRate);
                    rawBatteryVoltage[sampleIndex] = batteryDiffPulse[sampleIndex] * (3.3 / 4095.0);
                    rawCurrent[sampleIndex] = resistorVoltagePulse[sampleIndex] * (3.3 / 4095.0) / shuntResistance;
                }
            }
            digitalWrite(transistorBasePin, LOW);
            delay(1);

            // Analyze the captured time-domain data
            analyzeTimeDomainData(freqIndex, dutyIndex, vccPulse, batteryDiffPulse, resistorVoltagePulse);
        }

        frequency = startFrequency + (endFrequency - startFrequency) * (float)(freqIndex + 1) / frequencySteps;
        delay(10);
        if (freqIndex == rawDataFrequencyIndex && storeRawData) {
            storeRawData = false; // Only store for the first frequency
        }
    }

    // Plot Raw Data
    plotRawData(rawTime, rawBatteryVoltage, rawCurrent, samplesPerCycle);
    delay(5000);

    // Plot Inferred Resistance vs. Frequency for a specific duty cycle
    int dutyCycleToPlot = 1; // Example: 30% duty cycle
    float resistanceToPlot[maxFrequencies];
    for (int i = 0; i < maxFrequencies; i++) {
        resistanceToPlot[i] = inferredResistance[i][dutyCycleToPlot];
    }
    plotGraph("Inferred Resistance vs. Frequency (30% Duty)", sweptFrequencies, resistanceToPlot, frequencySteps, "Frequency (Hz)", "Resistance (Ohm)");
    delay(5000);

    // Plot Inferred Capacitance vs. Frequency for a specific duty cycle
    float capacitanceToPlot[maxFrequencies];
    for (int i = 0; i < maxFrequencies; i++) {
        capacitanceToPlot[i] = inferredCapacitance[i][dutyCycleToPlot];
    }
    plotGraph("Inferred Capacitance vs. Frequency (30% Duty)", sweptFrequencies, capacitanceToPlot, frequencySteps, "Frequency (Hz)", "Capacitance (F)");
    delay(5000);

    // Plot Inferred ESR vs. Frequency for a specific duty cycle
    float esrToPlot[maxFrequencies];
    for (int i = 0; i < maxFrequencies; i++) {
        esrToPlot[i] = inferredESR[i][dutyCycleToPlot];
    }
    plotGraph("Inferred ESR vs. Frequency (30% Duty)", sweptFrequencies, esrToPlot, frequencySteps, "Frequency (Hz)", "ESR (Ohm)");
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
    for (int i = (int)(onSamples * 0.1); i < (int)(onSamples * 0.9); i++) {
        avgBatteryVoltageOn += batteryVoltage[i];
        avgCurrentOn += current[i];
    }
    if ((int)(onSamples * 0.8) > 0) {
        avgBatteryVoltageOn /= (onSamples * 0.8);
        avgCurrentOn /= (onSamples * 0.8);
    }

    float avgBatteryVoltageOff = 0;
    float avgCurrentOff = 0;
    for (int i = onSamples + (int)(offSamples * 0.1); i < onSamples + (int)(offSamples * 0.9); i++) {
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

    // Estimate ESR (from voltage jump, trying to avoid initial transient)
    int esrStartIndex = 10; // Look a bit later after the start
    int esrBaselineEndIndex = 5; // Look a bit later before the start
    if (esrStartIndex < onSamples && esrBaselineEndIndex >= 0 && (onSamples + esrBaselineEndIndex) < samplesPerCycle) {
        float currentChange = current[esrStartIndex] - current[onSamples + esrBaselineEndIndex];
        float voltageChange = batteryVoltage[esrStartIndex] - batteryVoltage[onSamples + esrBaselineEndIndex];
        if (abs(currentChange) > 0.001) { // Avoid division by very small current changes
            inferredESR[freqIndex][dutyIndex] = abs(voltageChange / currentChange);
        } else {
            inferredESR[freqIndex][dutyIndex] = NAN;
        }
    } else {
        inferredESR[freqIndex][dutyIndex] = NAN;
    }

    // Estimate Capacitance (from discharge transient - IMPROVED)
    int dischargeStartIndex = onSamples + 5; // Start a bit into the OFF period
    int dischargeEndIndex = samplesPerCycle - 10; // End a bit before the next ON
    inferredCapacitance[freqIndex][dutyIndex] = NAN; // Initialize to NaN

    if (dischargeStartIndex >= onSamples && dischargeStartIndex < dischargeEndIndex) {
        float v_start = batteryVoltage[dischargeStartIndex];
        float v_mid = batteryVoltage[dischargeStartIndex + (dischargeEndIndex - dischargeStartIndex) / 2]; // Middle point
        float dt = (dischargeEndIndex - dischargeStartIndex) * (1.0 / sampleRate);
        float dt_mid = (dischargeStartIndex + (dischargeEndIndex - dischargeStartIndex) / 2 - dischargeStartIndex) * (1.0 / sampleRate);
        float r_est = inferredResistance[freqIndex][dutyIndex]; // Use the estimated DC resistance

        if (!isnan(r_est) && r_est > 0 && v_start > v_mid && v_mid > batteryVoltage[dischargeEndIndex]) {
            // Using two points to estimate the time constant
            float tau = -dt_mid / log(v_mid / v_start);
            inferredCapacitance[freqIndex][dutyIndex] = tau / r_est;
            if (inferredCapacitance[freqIndex][dutyIndex] < 1e-9 || inferredCapacitance[freqIndex][dutyIndex] > 1) { // Filter unrealistic values
                inferredCapacitance[freqIndex][dutyIndex] = NAN;
            }
        } else {
            inferredCapacitance[freqIndex][dutyIndex] = NAN;
        }
    } else {
        inferredCapacitance[freqIndex][dutyIndex] = NAN;
    }

    Serial.printf("    Inferred Resistance (DC Approx): %.3f Ohm\n", inferredResistance[freqIndex][dutyIndex]);
    Serial.printf("    Inferred Capacitance (Approx): %.3e F\n", inferredCapacitance[freqIndex][dutyIndex]);
    Serial.printf("    Inferred ESR (Approx): %.3f Ohm\n", inferredESR[freqIndex][dutyIndex]);
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
    bool firstValidY = true;

    for (int i = 0; i < numPoints; i++) {
        if (!isnan(yData[i])) {
            if (firstValidY) {
                minY = yData[i];
                maxY = yData[i];
                firstValidY = false;
            } else {
                minY = min(minY, yData[i]);
                maxY = max(maxY, yData[i]);
            }
        }
        minX = min(minX, xData[i]);
        maxX = max(maxX, xData[i]);
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

void plotRawData(float time[], float voltage[], float current[], int numPoints) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.println("Raw Data (First Frequency, First Duty Cycle)");

    // Plot Battery Voltage
    plotGraph("Raw Battery Voltage", time, voltage, numPoints, "Time (s)", "Voltage (V)");
    delay(5000);

    // Plot Current
    plotGraph("Raw Current", time, current, numPoints, "Time (s)", "Current (A)");
    delay(5000);
}
