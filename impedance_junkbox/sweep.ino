#include <TFT_eSPI.h>
#include <math.h>

// ------ Pin Definitions ------
const int transistorBasePin = 25;      // Digital output to control NPN transistor base
const int batteryVoltagePin  = 34;      // ADC input for battery negative terminal voltage
const int resistorVoltagePin = 35;      // ADC input across 2.5Ω resistor (shunted by 2000µF cap)

// ------ TFT Display Setup ------
TFT_eSPI tft = TFT_eSPI();             // Use default TFT_eSPI config (set in User_Setup.h)

// ------ Sine Lookup Table for waveform generation ------
const int sineTableSize = 256;
float sineTable[sineTableSize];

void generateSineTable() {
  // Generate one period of a sine wave scaled between 0 and 1.
  for (int i = 0; i < sineTableSize; i++) {
    sineTable[i] = (sin(2 * PI * i / sineTableSize) + 1.0) / 2.0;
  }
}

// ------ Frequency Sweep Parameters ------
const int numFrequencies = 50;         // Number of frequency points in the sweep
float freqArray[numFrequencies];
float magArray[numFrequencies];        // Measured “transfer function” magnitude
float phaseArray[numFrequencies];      // (Phase measurement not implemented in this example)

void setupFrequencies() {
  // Create logarithmically spaced frequencies from 100 Hz to 40 kHz.
  float fStart = 100.0;
  float fEnd = 40000.0;
  for (int i = 0; i < numFrequencies; i++) {
    float ratio = (float)i / (numFrequencies - 1);
    freqArray[i] = fStart * pow((fEnd / fStart), ratio);
  }
}

// ------ Waveform Generation and Measurement ------
// For each frequency we output a sine waveform (using our lookup table)
// by toggling the transistor base pin (using a simple threshold method)
// while sampling the resistor voltage. (Here we assume that the large 2000µF capacitor
// “smooths” the resistor voltage so that it represents the DC response to the AC injection.)

float measureAtFrequency(float frequency) {
  const int cycles = 10;  // Number of complete cycles for averaging
  // Compute period (in microseconds) and the sample interval per table entry.
  unsigned long periodMicros = (1.0 / frequency) * 1e6;
  unsigned long sampleInterval = periodMicros / sineTableSize;
  int totalSamples = cycles * sineTableSize;
  
  float voltageSum = 0;
  for (int i = 0; i < totalSamples; i++) {
    int index = i % sineTableSize;
    // Output “analog” excitation by using the sine table.
    // (Here we use a crude method: output HIGH if sine value > 0.5, LOW otherwise.)
    if (sineTable[index] > 0.5) {
      digitalWrite(transistorBasePin, HIGH);
    } else {
      digitalWrite(transistorBasePin, LOW);
    }
    // Wait for the appropriate sample interval.
    delayMicroseconds(sampleInterval);
    
    // Read ADC from the resistor voltage node.
    int adcValue = analogRead(resistorVoltagePin);
    // Convert ADC reading to voltage (assuming 3.3V reference and 12-bit ADC resolution: 0–4095).
    float voltage = (adcValue / 4095.0) * 3.3;
    voltageSum += voltage;
  }
  
  // Return the average voltage over the samples as the measured response amplitude.
  float avgVoltage = voltageSum / totalSamples;
  return avgVoltage;
}

void performFrequencySweep() {
  for (int i = 0; i < numFrequencies; i++) {
    float f = freqArray[i];
    // Get the measured amplitude at this frequency.
    float response = measureAtFrequency(f);
    magArray[i] = response;
    // Phase measurement is not implemented; set as zero placeholder.
    phaseArray[i] = 0;
    
    // Optionally update TFT display with current progress.
    tft.fillRect(0, 0, 320, 20, TFT_BLACK);
    tft.setCursor(0, 0);
    tft.printf("Sweep: %.1f Hz", f);
  }
}

// ------ RLC Parameter Inference ------
// This stub finds the frequency with maximum response and, assuming a nominal capacitor value,
// computes L from the resonance formula f_res = 1/(2*pi*sqrt(L*C)).
// R is set as a placeholder.
void inferRLCParameters(float &R, float &L, float &C) {
  int maxIndex = 0;
  float maxVal = 0;
  for (int i = 0; i < numFrequencies; i++) {
    if (magArray[i] > maxVal) {
      maxVal = magArray[i];
      maxIndex = i;
    }
  }
  float fRes = freqArray[maxIndex];
  // For a series RLC, f_res = 1/(2*pi*sqrt(L*C)).
  // Here, we assume the capacitor value is known (for demonstration, we use 2000µF).
  C = 2000e-6;  
  L = 1.0 / ( (2 * PI * fRes) * (2 * PI * fRes) * C );
  // R could be estimated from the bandwidth (not implemented here).
  R = 10.0;  // Placeholder value
}

// ------ Plotting Functions ------
// Plot the measured transfer function (magnitude vs. frequency) using autoscaling.
void plotTransferFunction() {
  tft.fillScreen(TFT_BLACK);
  // Find minimum and maximum measured magnitudes.
  float minMag = magArray[0], maxMag = magArray[0];
  for (int i = 0; i < numFrequencies; i++) {
    if (magArray[i] < minMag) minMag = magArray[i];
    if (magArray[i] > maxMag) maxMag = magArray[i];
  }
  
  // Plot by mapping log10(frequency) to x and magnitude to y.
  for (int i = 0; i < numFrequencies - 1; i++) {
    float x1 = map(log10(freqArray[i]) * 100, log10(freqArray[0]) * 100, log10(freqArray[numFrequencies - 1]) * 100, 0, 320);
    float y1 = map(magArray[i], minMag, maxMag, 240, 0);
    float x2 = map(log10(freqArray[i + 1]) * 100, log10(freqArray[0]) * 100, log10(freqArray[numFrequencies - 1]) * 100, 0, 320);
    float y2 = map(magArray[i + 1], minMag, maxMag, 240, 0);
    tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
  }
}

// Plot a theoretical bode plot based on the inferred RLC parameters.
// In this example we plot the impedance magnitude of a series RLC as a function of frequency.
void plotBodePlot(float R, float L, float C) {
  tft.fillScreen(TFT_BLACK);
  // For scaling, we assume the impedance magnitude will be in the range 0 to about 100 Ω (adjust as needed).
  for (int i = 0; i < numFrequencies - 1; i++) {
    float w1 = 2 * PI * freqArray[i];
    float w2 = 2 * PI * freqArray[i + 1];
    // Impedance magnitude: Z = sqrt( R^2 + (wL - 1/(wC))^2 )
    float Z1 = sqrt(R * R + pow((w1 * L - 1 / (w1 * C)), 2));
    float Z2 = sqrt(R * R + pow((w2 * L - 1 / (w2 * C)), 2));
    float x1 = map(log10(freqArray[i]) * 100, log10(freqArray[0]) * 100, log10(freqArray[numFrequencies - 1]) * 100, 0, 320);
    float y1 = map(Z1, 0, 100, 240, 0);
    float x2 = map(log10(freqArray[i + 1]) * 100, log10(freqArray[0]) * 100, log10(freqArray[numFrequencies - 1]) * 100, 0, 320);
    float y2 = map(Z2, 0, 100, 240, 0);
    tft.drawLine(x1, y1, x2, y2, TFT_CYAN);
  }
}

void setup() {
  Serial.begin(115200);
  // Initialize transistor control pin.
  pinMode(transistorBasePin, OUTPUT);
  digitalWrite(transistorBasePin, LOW);
  
  // (ESP32 analogRead is available by default on designated ADC pins.)
  
  // Initialize TFT display.
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  
  // Generate sine lookup table and set up the frequency sweep.
  generateSineTable();
  setupFrequencies();
  
  // Display startup message.
  tft.setCursor(0, 0);
  tft.println("Starting Sweep...");
  delay(1000);
  
  // Perform the frequency sweep.
  performFrequencySweep();
  
  // Plot the measured transfer function.
  plotTransferFunction();
  
  // Infer RLC parameters from the measured data.
  float R, L, C;
  inferRLCParameters(R, L, C);
  Serial.printf("Inferred R: %.2f Ohm, L: %.6f H, C: %.6f F\n", R, L, C);
  
  delay(2000);
  
  // Plot the theoretical bode plot from the inferred parameters.
  plotBodePlot(R, L, C);
}

void loop() {
  // Nothing to do in loop
}
