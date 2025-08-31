/*
  Improved ESP32 sweep + R/ESR/C estimation sketch
  - Uses LEDC for stable PWM generation (frequency + duty)
  - Samples multiple cycles with micros() scheduling
  - Estimates DC resistance from on/off averages
  - Estimates ESR from voltage/current jump at transition
  - Estimates capacitance by linear regression on ln(V) during discharge (OFF)
  - Plots simple graphs to TFT_eSPI

  Notes: calibrate the voltage divider scale factors (VCC_SCALE, BAT_SCALE, SHUNT_SCALE)
         to match your hardware. ADC reads are 12-bit (0..4095) by default.

  Author: assistant (refactor for user's code)
*/

#include <WiFi.h>
#include <TFT_eSPI.h>
#include <math.h>

// TFT Display
TFT_eSPI tft = TFT_eSPI();

// Pin Definitions (ESP32 pins used as in user's code)
const int transistorBasePin = 19; // PWM output (LEDC)
const int batteryVoltagePin = 39; // ADC1_CH3
const int resistorVoltagePin = 34; // ADC1_CH6
const int vccVoltagePin = 35; // ADC1_CH7

// Shunt resistor (Ohms)
const float shuntResistance = 2.5f;

// Hardware ADC scaling (set these to your measured divider factors)
const float VCC_SCALE = 2.0f;     // if Vcc measured through a 1:1 divider (example)
const float BAT_SCALE = 1.0f;     // battery pin divider factor
const float SHUNT_SCALE = 1.0f;   // resistor voltage divider factor (if any)

// Sweep parameters (keep frequency low enough so ADC can sample reliably)
const int startFrequency = 10;
const int endFrequency   = 100;
const int frequencySteps  = 8;   // remains same as original (maxFrequencies)
const int maxFrequencies = frequencySteps;

float sweptFrequencies[maxFrequencies];

// Duty cycles
const float dutyCycles[] = {0.1f, 0.3f, 0.4f, 0.5f, 0.7f};
const int numDutyCycles = sizeof(dutyCycles) / sizeof(dutyCycles[0]);

// Sampling / capture
const int samplesPerCycle = 128; // samples per cycle (user value)
const int captureCycles = 8;     // capture cycle count -> total samples = samplesPerCycle * captureCycles
const int maxTotalSamples = samplesPerCycle * captureCycles;

// Arrays to store inferred params
float inferredResistance[maxFrequencies][numDutyCycles];
float inferredCapacitance[maxFrequencies][numDutyCycles];
float inferredESR[maxFrequencies][numDutyCycles];

// ADC and conversion
const float ADC_REF = 3.3f;
const int ADC_MAX = 4095;

// PWM (LEDC) settings
const int ledcChannel = 0;
const int ledcTimerResolutionBits = 8; // 8-bit duty resolution (0..255)
const int ledcMaxDuty = (1 << ledcTimerResolutionBits) - 1;

// Function prototypes
void setupTFT();
void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel);
void captureAndAnalyze(int freqIndex, int dutyIndex, float frequency, float duty);
float readVoltageCalibration(int pin, float scale);
float estimateCapacitanceFromOffWindow(const float volt[], const unsigned long timesUs[], int startIdx, int endIdx, float R_est);

void setup() {
  Serial.begin(115200);
  delay(100);
  setupTFT();

  // Configure pins
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW);

  analogReadResolution(12); // 12-bit ADC

  // Initialize inferred arrays to NAN
  for (int f = 0; f < maxFrequencies; f++) {
    for (int d = 0; d < numDutyCycles; d++) {
      inferredResistance[f][d] = NAN;
      inferredCapacitance[f][d] = NAN;
      inferredESR[f][d] = NAN;
    }
  }
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Starting Frequency and Duty Sweep...");

  for (int freqIndex = 0; freqIndex < frequencySteps; freqIndex++) {
    // sweep frequencies evenly inclusive of start & end
    float frequency = startFrequency + (endFrequency - startFrequency) * (float)freqIndex / (float)(frequencySteps - 1);
    sweptFrequencies[freqIndex] = frequency;

    Serial.printf("\n=== Frequency: %.2f Hz  (index %d)\n", frequency, freqIndex);

    for (int dutyIndex = 0; dutyIndex < numDutyCycles; dutyIndex++) {
      float duty = dutyCycles[dutyIndex];
      Serial.printf("  Duty: %.1f%%\n", duty * 100.0f);

      // Use LEDC to generate stable PWM on transistorBasePin
      // ledcSetup requires the desired PWM freq. We'll set resolution to ledcTimerResolutionBits.
      ledcSetup(ledcChannel, (int)round(frequency), ledcTimerResolutionBits);
      ledcAttachPin(transistorBasePin, ledcChannel);
      int dutyValue = (int)round(duty * (float)ledcMaxDuty);
      ledcWrite(ledcChannel, dutyValue);

      // Capture and analyze
      captureAndAnalyze(freqIndex, dutyIndex, frequency, duty);

      // Turn PWM OFF for a small pause between runs
      ledcWrite(ledcChannel, 0);
      delay(50);
    }
    delay(20);
  }

  // Example: plot results for dutyCycle index 1 (30%)
  int dutyPlotIndex = 1;
  float resistanceToPlot[maxFrequencies];
  float capacitanceToPlot[maxFrequencies];
  float esrToPlot[maxFrequencies];
  for (int i = 0; i < maxFrequencies; i++) {
    resistanceToPlot[i] = inferredResistance[i][dutyPlotIndex];
    capacitanceToPlot[i] = inferredCapacitance[i][dutyPlotIndex];
    esrToPlot[i] = inferredESR[i][dutyPlotIndex];
  }

  plotGraph("Inferred Resistance vs Freq (30%)", sweptFrequencies, resistanceToPlot, maxFrequencies, "Freq (Hz)", "R (Ohm)");
  delay(3000);
  plotGraph("Inferred Capacitance vs Freq (30%)", sweptFrequencies, capacitanceToPlot, maxFrequencies, "Freq (Hz)", "C (F)");
  delay(3000);
  plotGraph("Inferred ESR vs Freq (30%)", sweptFrequencies, esrToPlot, maxFrequencies, "Freq (Hz)", "ESR (Ohm)");
  delay(3000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2 - 8);
  tft.println("Sweep Completed");
  delay(5000);
}

// Capture many samples using micros() scheduling and analyze them
void captureAndAnalyze(int freqIndex, int dutyIndex, float frequency, float duty) {
  const unsigned long periodUs = (unsigned long)round(1e6f / frequency);
  const float samplingIntervalUsF = (float)periodUs / (float)samplesPerCycle;
  const unsigned long samplingIntervalUs = max(1UL, (unsigned long)round(samplingIntervalUsF));

  const int totalSamples = samplesPerCycle * captureCycles;
  static float vccArr[maxTotalSamples];
  static float batteryArr[maxTotalSamples];
  static float resistorArr[maxTotalSamples];
  static unsigned long timeUsArr[maxTotalSamples];

  // Start time reference
  unsigned long startMicros = micros();
  unsigned long nextSampleAt = startMicros;

  int idx = 0;
  while (idx < totalSamples) {
    unsigned long now = micros();
    if (now >= nextSampleAt) {
      // read ADCs
      float vcc_raw = analogRead(vccVoltagePin);
      float bat_raw = analogRead(batteryVoltagePin);
      float res_raw = analogRead(resistorVoltagePin);

      // convert to voltages (apply scale factors)
      vccArr[idx] = (vcc_raw * (ADC_REF / ADC_MAX)) * VCC_SCALE;
      batteryArr[idx] = (bat_raw * (ADC_REF / ADC_MAX)) * BAT_SCALE;
      resistorArr[idx] = (res_raw * (ADC_REF / ADC_MAX)) * SHUNT_SCALE;

      timeUsArr[idx] = now - startMicros;
      idx++;

      nextSampleAt += samplingIntervalUs;
      // safety if micros() skipped ahead
      if (nextSampleAt < now) nextSampleAt = now + samplingIntervalUs;
    }
  }

  // Now process the block of captured samples (split into cycles)
  // Build per-sample derived values
  static float currentArr[maxTotalSamples];
  static float batteryVoltageArr[maxTotalSamples];

  for (int i = 0; i < totalSamples; i++) {
    currentArr[i] = resistorArr[i] / shuntResistance; // I = V_shunt / Rshunt
    batteryVoltageArr[i] = vccArr[i] - batteryArr[i];
  }

  // We'll compute averages for ON and OFF windows across cycles
  // Determine indices within a single cycle that correspond to ON and OFF
  const int onSamplesPerCycle = max(1, (int)round(samplesPerCycle * duty));
  const int offSamplesPerCycle = samplesPerCycle - onSamplesPerCycle;

  // Guard
  if (offSamplesPerCycle <= 2 || onSamplesPerCycle <= 2) {
    Serial.println("  Warning: too small ON/OFF sample counts for reliable estimation");
  }

  // accumulate averages across cycles
  double sumV_on = 0.0, sumI_on = 0.0;
  double sumV_off = 0.0, sumI_off = 0.0;
  int count_on = 0, count_off = 0;

  for (int cycle = 0; cycle < captureCycles; cycle++) {
    int base = cycle * samplesPerCycle;
    // Take interior part of ON window to avoid edges: 10% trimmed
    int onStart = base + max(1, (int)round(onSamplesPerCycle * 0.1));
    int onEnd = base + max(1, (int)round(onSamplesPerCycle * 0.9));
    if (onEnd <= onStart) { onStart = base; onEnd = base + onSamplesPerCycle - 1; }

    for (int i = onStart; i < base + onSamplesPerCycle && i <= onEnd && i < totalSamples; i++) {
      sumV_on += batteryVoltageArr[i];
      sumI_on += currentArr[i];
      count_on++;
    }

    // OFF window interior
    int offBase = base + onSamplesPerCycle;
    int offStart = offBase + max(1, (int)round(offSamplesPerCycle * 0.1));
    int offEnd = offBase + max(1, (int)round(offSamplesPerCycle * 0.9));
    if (offEnd <= offStart) { offStart = offBase; offEnd = offBase + offSamplesPerCycle - 1; }

    for (int i = offStart; i <= offEnd && i < totalSamples; i++) {
      sumV_off += batteryVoltageArr[i];
      sumI_off += currentArr[i];
      count_off++;
    }
  }

  float avgV_on = (count_on > 0) ? (float)(sumV_on / count_on) : NAN;
  float avgI_on = (count_on > 0) ? (float)(sumI_on / count_on) : NAN;
  float avgV_off = (count_off > 0) ? (float)(sumV_off / count_off) : NAN;
  float avgI_off = (count_off > 0) ? (float)(sumI_off / count_off) : NAN;

  // Estimate DC resistance using difference method
  float R_est = NAN;
  if (!isnan(avgI_on) && !isnan(avgI_off) && fabsf(avgI_on - avgI_off) > 1e-6f) {
    R_est = (avgV_on - avgV_off) / (avgI_on - avgI_off);
    // Filter unrealistic values
    if (!(R_est > 0 && R_est < 1000.0f)) R_est = NAN;
  } else {
    R_est = NAN;
  }
  inferredResistance[freqIndex][dutyIndex] = R_est;

  Serial.printf("    AvgV_on=%.4f V, AvgI_on=%.6f A, AvgV_off=%.4f V, AvgI_off=%.6f A -> R_est=%.3f\n",
                avgV_on, avgI_on, avgV_off, avgI_off, isnan(R_est) ? NAN : R_est);

  // Estimate ESR: look at voltage jump at ON transition and current jump
  // For each cycle, compare first ON sample (index base) vs last OFF sample (index base+onSamplesPerCycle-1)
  double esrAccum = 0.0;
  int esrCount = 0;
  for (int cycle = 0; cycle < captureCycles; cycle++) {
    int base = cycle * samplesPerCycle;
    int firstOnIdx = base; // first point in the cycle belongs to ON region (depending on duty phase)
    int lastOffIdx = base + onSamplesPerCycle - 1;
    if (firstOnIdx >= 0 && firstOnIdx < totalSamples && lastOffIdx >= 0 && lastOffIdx < totalSamples) {
      float dV = batteryVoltageArr[firstOnIdx] - batteryVoltageArr[lastOffIdx];
      float dI = currentArr[firstOnIdx] - currentArr[lastOffIdx];
      // avoid small changes
      if (fabsf(dI) > 1e-5f && fabsf(dV) > 1e-4f) {
        float esrLocal = fabsf(dV / dI);
        if (esrLocal > 0 && esrLocal < 1000.0f) {
          esrAccum += esrLocal;
          esrCount++;
        }
      }
    }
  }
  float esrEst = (esrCount > 0) ? (float)(esrAccum / esrCount) : NAN;
  inferredESR[freqIndex][dutyIndex] = esrEst;
  Serial.printf("    ESR estimate from jumps: %.3f Ohm (n=%d)\n", isnan(esrEst) ? NAN : esrEst, esrCount);

  // Estimate capacitance: perform exponential fit on OFF transient (across cycles)
  // We'll build a combined OFF list of times & voltages across all cycles (only interior off points)
  static float offVoltages[maxTotalSamples];
  static unsigned long offTimes[maxTotalSamples];
  int offCount = 0;
  for (int cycle = 0; cycle < captureCycles; cycle++) {
    int base = cycle * samplesPerCycle;
    int offBase = base + onSamplesPerCycle;
    int offStart = offBase + 1; // small skip to avoid immediate edge
    int offEnd = offBase + offSamplesPerCycle - 2;
    if (offEnd >= totalSamples) offEnd = totalSamples - 1;
    for (int i = offStart; i <= offEnd && i < totalSamples && offCount < maxTotalSamples; i++) {
      offVoltages[offCount] = batteryVoltageArr[i];
      offTimes[offCount] = timeUsArr[i] + (unsigned long)(cycle * periodUs); // approximate continuous time
      offCount++;
    }
  }

  float C_est = NAN;
  if (offCount >= 6 && !isnan(R_est)) { // need enough points and R estimate
    C_est = estimateCapacitanceFromOffWindow(offVoltages, offTimes, 0, offCount - 1, R_est);
    // sanity filter
    if (!(C_est > 1e-12f && C_est < 10.0f)) C_est = NAN;
  } else {
    C_est = NAN;
  }
  inferredCapacitance[freqIndex][dutyIndex] = C_est;

  Serial.printf("    Capacitance estimate: %.3e F\n", isnan(C_est) ? NAN : C_est);
}

// Convert raw ADC reading into calibrated voltage (with scale/divider)
float readVoltageCalibration(int pin, float scale) {
  float raw = analogRead(pin);
  return (raw * (ADC_REF / ADC_MAX)) * scale;
}

// Estimate capacitance by linear regression on ln(V) vs t (least squares).
// volt[]: voltages (V), timesUs[]: time in microseconds (must be increasing), startIdx..endIdx inclusive
// Returns estimated C = tau / R (where tau from slope) if success, otherwise NAN
float estimateCapacitanceFromOffWindow(const float volt[], const unsigned long timesUs[], int startIdx, int endIdx, float R_est) {
  // We are assuming V(t) = V0 * exp(-t/tau) -> ln(V) = ln(V0) - t/tau
  // Linear regression: ln(V) = a + b * t, where b = -1/tau
  double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
  int n = 0;
  // Choose a small epsilon to avoid log of non-positive numbers
  const float epsV = 1e-6f;
  for (int i = startIdx; i <= endIdx; i++) {
    float V = volt[i];
    if (V <= epsV) continue;
    double t = (double)timesUs[i] * 1e-6; // seconds
    double y = log(V);
    Sx += t;
    Sy += y;
    Sxx += t * t;
    Sxy += t * y;
    n++;
  }
  if (n < 4) return NAN;
  double denom = (n * Sxx - Sx * Sx);
  if (fabs(denom) < 1e-12) return NAN;
  double b = (n * Sxy - Sx * Sy) / denom; // slope
  if (fabs(b) < 1e-12) return NAN;
  double tau = -1.0 / b;
  if (!(tau > 0 && tau < 1e6)) return NAN; // sanity limits
  double C = tau / (double)R_est;
  return (float)C;
}

// Basic plotting routine (handles NaNs)
void plotGraph(const char* title, float* xData, float* yData, int numPoints, const char* xAxisLabel, const char* yAxisLabel) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println(title);

  // Find valid min/max
  bool hasValid = false;
  float minX = 0, maxX = 0, minY = 0, maxY = 0;
  for (int i = 0; i < numPoints; i++) {
    if (!isnan(yData[i]) && !isnan(xData[i]) && isfinite(yData[i]) && isfinite(xData[i])) {
      if (!hasValid) {
        minX = maxX = xData[i];
        minY = maxY = yData[i];
        hasValid = true;
      } else {
        if (xData[i] < minX) minX = xData[i];
        if (xData[i] > maxX) maxX = xData[i];
        if (yData[i] < minY) minY = yData[i];
        if (yData[i] > maxY) maxY = yData[i];
      }
    }
  }

  if (!hasValid) {
    tft.setCursor(0, 20);
    tft.setTextColor(TFT_YELLOW);
    tft.println("No valid data to plot.");
    return;
  }

  // Padding if ranges zero
  if (fabs(maxX - minX) < 1e-6f) { maxX += 0.5f; minX -= 0.5f; }
  if (fabs(maxY - minY) < 1e-6f) { maxY += 0.5f; minY -= 0.5f; }

  float xScale = (tft.width() - 40) / (maxX - minX);
  float yScale = (tft.height() - 60) / (maxY - minY);

  // axes
  tft.drawLine(20, tft.height() - 30, tft.width() - 20, tft.height() - 30, TFT_WHITE); // x axis
  tft.drawLine(20, tft.height() - 30, 20, 30, TFT_WHITE); // y axis

  // plot points as polyline when consecutive valid
  tft.setTextColor(TFT_GREEN);
  int lastX = -1, lastY = -1;
  for (int i = 0; i < numPoints; i++) {
    if (isnan(yData[i]) || isnan(xData[i]) || !isfinite(yData[i]) || !isfinite(xData[i])) {
      lastX = lastY = -1; // break the line
      continue;
    }
    int x = 20 + (int)round((xData[i] - minX) * xScale);
    int y = tft.height() - 30 - (int)round((yData[i] - minY) * yScale);
    if (lastX >= 0) {
      tft.drawLine(lastX, lastY, x, y, TFT_GREEN);
    } else {
      tft.fillRect(x - 1, y - 1, 3, 3, TFT_GREEN);
    }
    lastX = x; lastY = y;
  }

  // axis labels
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(tft.width() / 2 - (int)strlen(xAxisLabel) * 3, tft.height() - 20);
  tft.println(xAxisLabel);
  tft.setCursor(2, tft.height() / 2 - (int)strlen(yAxisLabel) * 3);
  tft.println(yAxisLabel);
}

void setupTFT() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
}
