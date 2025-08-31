/* 
  ESP32 NiMH-aware impedance sweep (Goertzel single-frequency DFT)
  - Square-wave excitation via LEDC (stable PWM)
  - Synchronous sampling using micros()
  - Goertzel extraction of complex phasors (voltage & current)
  - Computes Z(f) = V/I (complex), Bode data, phase, Nyquist points
  - Estimates:
      * High-frequency R0/ESR (average real(Z) at top freqs)
      * Warburg slope (log-log linear regression -> slope ~ -0.5 indicates diffusion)
      * Approx CPE exponent via phase slope
  - Plots Bode / Nyquist on TFT_eSPI
  - Notes: tune amplitude, sample counts, freq range for your hardware and patience.

  IMPORTANT hardware notes: use precise shunt, clamp ADC noise, ensure ADC channels are ADC1 pins (avoid ADC2 while WiFi active),
  and calibrate voltage divider scale factors (VCC_SCALE, BAT_SCALE, SHUNT_SCALE).
*/

#include <TFT_eSPI.h>
#include <math.h>

// === Hardware pins ===
const int transistorBasePin = 19; // LEDC PWM output to transistor base
const int batteryVoltagePin = 39; // ADC1_CH3 (battery sense node)
const int resistorVoltagePin = 34; // ADC1_CH6 (shunt across resistor)
const int vccVoltagePin = 35; // ADC1_CH7 (Vcc sense, optional)

// TFT
TFT_eSPI tft = TFT_eSPI();

// === Calibration & constants ===
const float ADC_REF = 3.3f;
const int ADC_MAX = 4095;
const float SHUNT_R = 2.5f;         // Ohms - adjust to your shunt
const float VCC_SCALE = 2.0f;       // voltage divider scaling for Vcc (V_measured = raw*(3.3/4095)*scale)
const float BAT_SCALE = 1.0f;       // battery divider scale
const float SHUNT_SCALE = 1.0f;     // shunt voltage divider (if any)
const int ADC_READ_DELAY_US = 1;    // small settle time if needed

// === Sweep parameters (tune for your run-time patience) ===
const float freqStart = 0.2f;       // Hz (set to 0.2 as practical starting point)
const float freqEnd   = 5000.0f;    // Hz (upper limit depends on ADC/DAC)
const int freqPoints  = 28;         // number of frequency points (log-spaced)
const int cyclesPerFreq = 8;        // capture cycles per frequency
int samplesPerCycle = 64;           // samples per cycle (adaptive later)

// internal storage
struct Complex { double re, im; };
float sweptFreqs[freqPoints];
Complex Zmeas[freqPoints];           // measured complex impedance per freq
float Zmag[freqPoints];
float Zphase[freqPoints];

// === LEDC config ===
const int ledcChannel = 0;
const int ledcResolutionBits = 8; // duty resolution
const int ledcMaxDuty = (1 << ledcResolutionBits) - 1;

// === Function prototypes ===
void setupTFT();
void sweepAndMeasure();
void doFrequencyPoint(int idx, float freq);
Complex goertzelComplex(const float samples[], const unsigned long timesUs[], int N, double fs, double targetFreq);
void computeDiagnostics();
void plotBode();
void plotNyquist();
float readCalibrated(int pin, float scale);

// === Utility: generate log-spaced frequencies ===
void generateLogFreqs() {
  double logStart = log10(freqStart);
  double logEnd = log10(freqEnd);
  for (int i = 0; i < freqPoints; ++i) {
    double t = (double)i / (double)(freqPoints - 1);
    sweptFreqs[i] = pow(10.0, logStart + (logEnd - logStart) * t);
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  setupTFT();

  analogReadResolution(12); // ADC 0..4095
  pinMode(transistorBasePin, OUTPUT);
  ledcSetup(ledcChannel, 1000, ledcResolutionBits); // initial dummy freq
  ledcAttachPin(transistorBasePin, ledcChannel);
  ledcWrite(ledcChannel, 0);

  generateLogFreqs();

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 10);
  tft.println("NiMH Impedance Sweep");
  delay(800);
}

void loop() {
  sweepAndMeasure();
  computeDiagnostics();

  // plot Bode + Nyquist
  plotBode();
  delay(3000);
  plotNyquist();
  delay(5000);

  // pause before repeating (or stop here)
  Serial.println("Sweep complete. Sleeping 10s before repeat.");
  delay(10000);
}

// === Main sweep ===
void sweepAndMeasure() {
  for (int i = 0; i < freqPoints; ++i) {
    float f = sweptFreqs[i];
    doFrequencyPoint(i, f);
    delay(40); // short breathing room
  }
}

void doFrequencyPoint(int idx, float freq) {
  // adapt samplesPerCycle to keep sampling rate within limits:
  // target sample rate <= ~200k sample/s (practical limit for analogRead in blocking mode)
  // choose samplesPerCycle so sampleRate = samplesPerCycle * cyclesPerFreq * freq is reasonable.
  int spc = samplesPerCycle;
  double approxRate = spc * cyclesPerFreq * freq;
  if (approxRate > 100000.0) {
    // reduce samples per cycle
    spc = max(16, (int)round(100000.0 / (cyclesPerFreq * freq)));
  }
  // ensure at least 16 samples/cycle
  if (spc < 16) spc = 16;

  int totalSamples = spc * cyclesPerFreq;
  static float vSamples[4096];
  static float iSamples[4096];
  static unsigned long tSamples[4096];

  if (totalSamples > 4096) {
    Serial.printf("Too many samples (%d) for available buffer, reducing cycles.\n", totalSamples);
    // reduce cycles to fit
    int newCycles = 4096 / spc;
    if (newCycles < 1) newCycles = 1;
    Serial.printf("Reducing cycles from %d to %d\n", cyclesPerFreq, newCycles);
  }

  // configure LEDC PWM for square excitation
  // keep duty small if you want small perturbation - but you use current measurement to normalize.
  int dutyValue = ledcMaxDuty / 4; // 25% (tune to get small current perturbation)
  ledcSetup(ledcChannel, (int)round(freq), ledcResolutionBits);
  ledcWrite(ledcChannel, dutyValue);

  // sampling timing
  double samplesPerSec = spc * freq; // per cycle times freq
  unsigned long sampleIntervalUs = (unsigned long)max(1.0, 1e6 / samplesPerSec);

  // capture loop
  unsigned long t0 = micros();
  unsigned long next = t0;
  int captured = 0;
  while (captured < totalSamples) {
    unsigned long now = micros();
    if (now >= next) {
      // read ADCs (read shunt for current, battery sense for voltage)
      int rawV = analogRead(batteryVoltagePin);
      int rawI = analogRead(resistorVoltagePin);
      // convert
      vSamples[captured] = (rawV * (ADC_REF / ADC_MAX)) * BAT_SCALE;
      float vsh = (rawI * (ADC_REF / ADC_MAX)) * SHUNT_SCALE;
      iSamples[captured] = vsh / SHUNT_R;
      tSamples[captured] = now - t0;
      captured++;
      next += sampleIntervalUs;
      // if micros jumped too far, resync
      if (next < now) next = now + sampleIntervalUs;
    }
  }

  // stop excitation quickly
  ledcWrite(ledcChannel, 0);

  // remove DC mean, optional Hann window in goertzel function
  Complex Vc = goertzelComplex(vSamples, tSamples, totalSamples, samplesPerSec, freq);
  Complex Ic = goertzelComplex(iSamples, tSamples, totalSamples, samplesPerSec, freq);

  // compute Z = V/I (complex)
  Complex Z;
  if (fabs(Ic.re) < 1e-12 && fabs(Ic.im) < 1e-12) {
    Z.re = NAN; Z.im = NAN;
  } else {
    // division (a+jb)/(c+jd) = ...
    double denom = Ic.re*Ic.re + Ic.im*Ic.im;
    Z.re = (Vc.re * Ic.re + Vc.im * Ic.im) / denom;
    Z.im = (Vc.im * Ic.re - Vc.re * Ic.im) / denom;
  }
  Zmeas[idx] = Z;
  Zmag[idx] = (float)sqrt(Z.re*Z.re + Z.im*Z.im);
  Zphase[idx] = (float)atan2(Z.im, Z.re) * 180.0f / M_PI;

  Serial.printf("f=%.3f Hz  | Z=%.3f + j%.3f  | |Z|=%.3f  ph=%.2f deg\n", freq, Z.re, Z.im, Zmag[idx], Zphase[idx]);
}

// === Goertzel for arbitrary frequency (returns complex amplitude normalized by N) ===
Complex goertzelComplex(const float samples[], const unsigned long timesUs[], int N, double fs, double targetFreq) {
  // We'll compute X = sum_{n=0}^{N-1} x[n] * exp(-j*omega*n*dt)
  // using recurrence with coeff = 2*cos(omega)
  double dt = 1.0 / fs;                    // seconds (sampling frequency used during capture)
  double omega = 2.0 * M_PI * targetFreq * dt; // rad/sample
  double coeff = 2.0 * cos(omega);
  double s_prev = 0.0, s_prev2 = 0.0;
  // subtract mean to remove DC
  double mean = 0.0;
  for (int i = 0; i < N; ++i) mean += samples[i];
  mean /= (double)N;

  // optional Hann window to reduce leakage (improves non-integer periods)
  for (int n = 0; n < N; ++n) {
    double xw = (samples[n] - mean) * (0.5 * (1.0 - cos(2.0*M_PI*n/(N-1)))); // Hann
    double s = xw + coeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }
  // compute real/imag parts
  double realPart = s_prev - s_prev2 * cos(omega);
  double imagPart = s_prev2 * sin(omega);

  // normalization: the Goertzel output equals approx (N/2) * Xk if windowed, but since we only need ratio V/I
  // a consistent scaling factor cancels out; we still normalize by N to keep magnitudes sensible for diagnostics
  Complex out;
  out.re = realPart / (double)N;
  out.im = imagPart / (double)N;
  return out;
}

// Diagnostics: estimate R0 (HF real), detect Warburg slope etc.
void computeDiagnostics() {
  // Estimate R0 as average real(Z) across top 2 frequencies (highest freqs)
  int topCount = 3;
  double sumR = 0.0; int nR = 0;
  for (int i = freqPoints - topCount; i < freqPoints; ++i) {
    if (i >= 0 && isfinite(Zmeas[i].re)) { sumR += Zmeas[i].re; nR++; }
  }
  double R0 = (nR>0)?(sumR / nR):NAN;
  Serial.printf("Estimated high-frequency R0 (series/internal): %.4f Ohm\n", R0);

  // Warburg slope: linear regression on log(|Z|) vs log(f) for low-mid freq window
  int startIdx = (int)(freqPoints * 0.06), endIdx = (int)(freqPoints * 0.5);
  if (startIdx < 0) startIdx = 0;
  if (endIdx <= startIdx+3) endIdx = min(freqPoints-1, startIdx + 4);
  double Sx=0,Sy=0,Sxx=0,Sxy=0; int n=0;
  for (int i = startIdx; i <= endIdx; ++i) {
    if (!isnan(Zmag[i]) && Zmag[i] > 1e-9) {
      double lx = log10(sweptFreqs[i]);
      double ly = log10(Zmag[i]);
      Sx += lx; Sy += ly; Sxx += lx*lx; Sxy += lx*ly; n++;
    }
  }
  double warburgSlope = NAN;
  if (n >= 4) {
    double denom = n*Sxx - Sx*Sx;
    if (fabs(denom) > 1e-12) warburgSlope = (n*Sxy - Sx*Sy) / denom;
  }
  Serial.printf("Warburg-like slope (log|Z| vs log f) in idx %d..%d : %.4f  (Warburg expected ~ -0.5)\n", startIdx, endIdx, isnan(warburgSlope)?NAN:warburgSlope);

  // Estimate CPE-like behavior: phase in mid-freq range average
  int midStart = (int)(freqPoints * 0.2), midEnd = (int)(freqPoints * 0.6);
  double phaseSum = 0.0; int pcount = 0;
  for (int i = midStart; i <= midEnd && i < freqPoints; ++i) {
    if (isfinite(Zphase[i])) { phaseSum += Zphase[i]; pcount++; }
  }
  double avgPhase = (pcount>0)?(phaseSum/pcount):NAN;
  Serial.printf("Avg phase (mid-freq) = %.2f deg (CPE shows -90*n degrees where n in (0..1))\n", avgPhase);

  // store some diagnostic values onto TFT (optional)
  tft.fillRect(0, 0, 220, 40, TFT_BLACK);
  tft.setCursor(0,0); tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
  tft.printf("R0: %.3f Ohm  Warburg slope: %.3f\n", R0, isnan(warburgSlope)?0.0:warburgSlope);
}

// === Simple plots ===
void plotBode() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0,0);
  tft.println("Bode magnitude");
  // Draw magnitude vs index (log f spacing on x)
  float minM = 1e12, maxM = -1e12;
  for (int i = 0; i < freqPoints; ++i) if (isfinite(Zmag[i])) { if (Zmag[i] < minM) minM = Zmag[i]; if (Zmag[i] > maxM) maxM = Zmag[i]; }
  if (minM>maxM) { tft.setCursor(0,20); tft.println("No data"); return; }
  if (maxM==minM) { maxM = minM*1.1 + 1e-6; }
  int w = tft.width(), h = tft.height();
  int x0 = 20, y0 = h-30;
  tft.drawLine(x0, y0, w-10, y0, TFT_WHITE);
  tft.drawLine(x0, y0, x0, 20, TFT_WHITE);
  int lastX=-1,lastY=-1;
  for (int i=0;i<freqPoints;i++) {
    if (!isfinite(Zmag[i])) { lastX=-1; continue; }
    float normX = (float)i/(float)(freqPoints-1);
    int x = x0 + (int)(normX*(w-30));
    int y = y0 - (int)((Zmag[i]-minM)/(maxM-minM)*(h-60));
    if (lastX>=0) tft.drawLine(lastX,lastY,x,y,TFT_GREEN);
    lastX=x; lastY=y;
    // small tick freq label every few
    if (i%4==0) {
      tft.setCursor(x-10, y0+4); tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
      tft.printf("%.1g", sweptFreqs[i]);
    }
  }
}

void plotNyquist() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0,0);
  tft.println("Nyquist (Re vs -Im)");
  // find ranges
  float minRe=1e12,maxRe=-1e12,minIm=1e12,maxIm=-1e12;
  for (int i=0;i<freqPoints;i++) {
    if (!isnan(Zmeas[i].re) && isfinite(Zmeas[i].re)) {
      if (Zmeas[i].re < minRe) minRe = Zmeas[i].re;
      if (Zmeas[i].re > maxRe) maxRe = Zmeas[i].re;
      float nim = -Zmeas[i].im;
      if (nim < minIm) minIm = nim;
      if (nim > maxIm) maxIm = nim;
    }
  }
  if (minRe>maxRe) { tft.setCursor(0,20); tft.println("No data"); return; }
  if (maxRe==minRe) { maxRe += 0.5; minRe -= 0.5; }
  if (maxIm==minIm) { maxIm += 0.5; minIm -= 0.5; }

  int x0 = 20, y0 = tft.height()-30, w = tft.width()-40, h = tft.height()-60;
  int lastX=-1,lastY=-1;
  for (int i=0;i<freqPoints;i++) {
    if (isnan(Zmeas[i].re) || !isfinite(Zmeas[i].re)) { lastX=-1; continue; }
    float rx = (Zmeas[i].re - minRe) / (maxRe - minRe);
    float iy = (-Zmeas[i].im - minIm) / (maxIm - minIm);
    int x = x0 + (int)(rx * w);
    int y = y0 - (int)(iy * h);
    if (lastX>=0) tft.drawLine(lastX,lastY,x,y,TFT_GREEN);
    lastX=x; lastY=y;
  }
}

// simple ADC read conversion helper
float readCalibrated(int pin, float scale) {
  int raw = analogRead(pin);
  return (raw * (ADC_REF / ADC_MAX)) * scale;
}

void setupTFT() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
}
