#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>
#include <SPI.h>
#include <Wire.h>

// Display Configuration
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;

// Analysis Configuration
constexpr size_t MAX_DATA_POINTS = 100;
constexpr size_t MAX_POLYNOMIAL_DEGREE = 3;
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 10.0f;
constexpr float MAX_FEATURE_DURATION = 300.0f;    // Maximum duration for a feature in seconds
constexpr float MIN_FEATURE_DURATION = 10.0f;     // Minimum duration for a feature in seconds
constexpr float MAX_LOOKBACK_TIME = 600.0f;       // Maximum time to look back for features
constexpr float MIN_TIME_STEP = 0.1f;            // Minimum time between samples
constexpr size_t MAX_FEATURES = 3;               // Track only last 3 features
constexpr float SIMILARITY_THRESHOLD = 0.15f;     // Maximum difference in growth rates to be considered same feature

// Data Structures
struct DataPoint {
    float timestamp;
    float value;
};

struct PolynomialFit {
    float coefficients[MAX_POLYNOMIAL_DEGREE + 1];
    uint8_t degree;
    float growthRate;
    float error;
};

struct ExponentialFeature {
    float startTime;
    float endTime;
    float growthRate;
    float baseValue;
    float finalValue;
    float error;
    bool active;
};

// Polynomial Fitting Class
class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 50;
    static constexpr float LEARNING_RATE = 0.01f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-5f;
    static constexpr float L2_LAMBDA = 0.001f;

    float evaluatePolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float calculateError(const float* coeffs, uint8_t degree, 
                        const float* x, const float* y, size_t n) const {
        float totalError = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            float prediction = evaluatePolynomial(coeffs, degree, x[i]);
            float diff = prediction - y[i];
            totalError += diff * diff;
        }
        return totalError / n;
    }

public:
    PolynomialFit fitPolynomial(const float* x, const float* y, 
                               size_t n, uint8_t degree) {
        PolynomialFit result;
        result.degree = degree;
        
        for (uint8_t i = 0; i <= degree; ++i) {
            result.coefficients[i] = 0.0f;
        }

        float learningRate = LEARNING_RATE;
        float prevError = calculateError(result.coefficients, degree, x, y, n);
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};
            
            for (size_t i = 0; i < n; ++i) {
                float prediction = evaluatePolynomial(result.coefficients, degree, x[i]);
                float error = prediction - y[i];
                float xPow = 1.0f;
                
                for (uint8_t j = 0; j <= degree; ++j) {
                    gradients[j] += 2.0f * error * xPow / n;
                    xPow *= x[i];
                }
            }

            for (uint8_t j = 0; j <= degree; ++j) {
                gradients[j] += 2.0f * L2_LAMBDA * result.coefficients[j];
                result.coefficients[j] -= learningRate * gradients[j];
            }

            float currentError = calculateError(result.coefficients, degree, x, y, n);
            if (currentError > prevError) {
                learningRate *= 0.5f;
            }

            if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                break;
            }
            prevError = currentError;
        }

        result.error = prevError;
        
        if (n > 0) {
            float lastX = x[n-1];
            float lastValue = evaluatePolynomial(result.coefficients, degree, lastX);
            float nextValue = evaluatePolynomial(result.coefficients, degree, lastX + 1.0f);
            result.growthRate = (lastValue != 0.0f) ? (nextValue - lastValue) / lastValue : 0.0f;
        } else {
            result.growthRate = 0.0f;
        }

        return result;
    }
};

// OLED Display Class
class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        if (in_max == in_min) return out_min;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    bool begin() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            return false;
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        return true;
    }

    void visualizeGrowthAnalysis(const float* xData,
                                const float* yData,
                                size_t dataSize,
                                const CircularBuffer<ExponentialFeature, MAX_FEATURES>& features) {
        if (dataSize < 2) {
            displayErrorState("Insufficient data");
            return;
        }

        display.clearDisplay();

        // Find min/max values
        float xMin = xData[0], xMax = xData[0];
        float yMin = yData[0], yMax = yData[0];
        for (size_t i = 1; i < dataSize; ++i) {
            xMin = min(xMin, xData[i]);
            xMax = max(xMax, xData[i]);
            yMin = min(yMin, yData[i]);
            yMax = max(yMax, yData[i]);
        }

        float yMargin = (yMax - yMin) * 0.1f;
        yMin -= yMargin;
        yMax += yMargin;

        const int plotHeight = SCREEN_HEIGHT - 24;
        
        // Draw data points
        for (size_t i = 0; i < dataSize; ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, plotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Draw features
        for (size_t i = 0; i < features.size(); i++) {
            const auto& feature = features[i];
            
            int16_t startX = mapFloat(feature.startTime, xMin, xMax, 0, SCREEN_WIDTH-1);
            int16_t endX = mapFloat(feature.endTime, xMin, xMax, 0, SCREEN_WIDTH-1);
            
            display.drawFastVLine(startX, 0, plotHeight, SSD1306_WHITE);
            if (!feature.active) {
                display.drawFastVLine(endX, 0, plotHeight, SSD1306_WHITE);
            }
            
            display.setCursor(0, plotHeight + 2 + (i * 10));
            display.print(F("F"));
            display.print(i+1);
            display.print(F(":"));
            display.print(feature.growthRate, 1);
        }

        display.display();
    }

    void displayErrorState(const char* errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("ERROR:"));
        display.println(errorMsg);
        display.display();
    }
};

// Time Window Analysis Class
class TimeWindowAnalyzer {
private:
    CircularBuffer<DataPoint, MAX_DATA_POINTS> dataBuffer;
    float currentTime = 0.0f;

    bool isValidTimeWindow(float startTime, float endTime) const {
        float duration = endTime - startTime;
        return duration >= MIN_FEATURE_DURATION && duration <= MAX_FEATURE_DURATION;
    }

    bool getTimeWindowIndices(float targetStartTime, float targetEndTime, 
                            size_t& startIdx, size_t& endIdx) const {
        if (dataBuffer.isEmpty()) return false;

        bool foundStart = false, foundEnd = false;
        for (size_t i = 0; i < dataBuffer.size(); i++) {
            float t = dataBuffer[i].timestamp;
            if (!foundStart && t >= targetStartTime) {
                startIdx = i;
                foundStart = true;
            }
            if (!foundEnd && t >= targetEndTime) {
                endIdx = i;
                foundEnd = true;
                break;
            }
        }

        if (!foundEnd) {
            endIdx = dataBuffer.size() - 1;
        }

        return foundStart;
    }

public:
    void addDataPoint(float timestamp, float value) {
        currentTime = timestamp;
        
        while (!dataBuffer.isEmpty() && 
               (currentTime - dataBuffer.first().timestamp > MAX_LOOKBACK_TIME)) {
            dataBuffer.shift();
        }

        DataPoint point = {timestamp, value};
        if (dataBuffer.isFull()) {
            dataBuffer.shift();
        }
        dataBuffer.push(point);
    }

    bool getTimeWindow(float startTime, float endTime, 
                      float* times, float* values, size_t& count) {
        size_t startIdx, endIdx;
        if (!getTimeWindowIndices(startTime, endTime, startIdx, endIdx)) {
            return false;
        }

        count = 0;
        for (size_t i = startIdx; i <= endIdx; i++) {
            times[count] = dataBuffer[i].timestamp;
            values[count] = dataBuffer[i].value;
            count++;
        }

        return count > 0;
    }

    float getCurrentTime() const { return currentTime; }
    size_t getDataCount() const { return dataBuffer.size(); }
};

// Feature Tracking Class
class FeatureTracker {
private:
    CircularBuffer<ExponentialFeature, MAX_FEATURES> features;
    
    bool isSimilarFeature(const ExponentialFeature& f1, const ExponentialFeature& f2) const {
        return abs(f1.growthRate - f2.growthRate) < SIMILARITY_THRESHOLD;
    }

public:
    void updateFeature(const ExponentialFeature& newFeature) {
        if (!features.isEmpty() && features.last().active) {
            auto& lastFeature = features.last();
            if (isSimilarFeature(lastFeature, newFeature)) {
                lastFeature.endTime = newFeature.endTime;
                lastFeature.finalValue = newFeature.finalValue;
                lastFeature.error = (lastFeature.error + newFeature.error) / 2;
                return;
            }
        }

        if (!features.isEmpty()) {
            features.last().active = false;
        }

        if (features.isFull()) {
            features.shift();
        }
        features.push(newFeature);
    }

    const CircularBuffer<ExponentialFeature, MAX_FEATURES>& getFeatures() const {
        return features;
    }
};

// Main Detector Class
class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer display;
    TimeWindowAnalyzer timeAnalyzer;
    FeatureTracker featureTracker;

    static constexpr size_t MAX_WINDOW_POINTS = 50;
    float windowTimes[MAX_WINDOW_POINTS];
    float windowValues[MAX_WINDOW_POINTS];

    ExponentialFeature analyzeTimeWindow(float startTime, float endTime) {
        size_t windowSize;
        if (!timeAnalyzer.getTimeWindow(startTime, endTime, 
                                      windowTimes, windowValues, windowSize)) {
            return ExponentialFeature{};
        }

        // Scale values
        float minVal = windowValues[0], maxVal = windowValues[0];
        for (size_t i = 1; i < windowSize; i++) {
            minVal = min(minVal, windowValues[i]);
            maxVal = max(maxVal, windowValues[i]);
        }

        float scale = (maxVal - minVal) > 1e-6 ? 1.0f / (maxVal - minVal) : 1.0f;
        float scaledValues[MAX_WINDOW_POINTS];
        
        for (size_t i = 0; i < windowSize; i++) {
            scaledValues[i] = (windowValues[i] - minVal) * scale;
        }

        // Normalize timestamps
        float normalizedTimes[MAX_WINDOW_POINTS];
        for (size_t i = 0; i < windowSize; i++) {
            normalizedTimes[i] = windowTimes[i] - startTime;
        }

        PolynomialFit fit = fitter.fitPolynomial(normalizedTimes, scaledValues, 
                                                windowSize, MAX_POLYNOMIAL_DEGREE);

        ExponentialFeature feature;
        feature.startTime = startTime;
        feature.endTime = endTime;
        feature.growthRate = fit.growthRate / scale;
        feature.baseValue = windowValues[0];
        feature.finalValue = windowValues[windowSize - 1];
        feature.error = fit.error;
        feature.active = true;

        return feature;
    }

public:
    bool begin() {
        return display.begin();
    }

void addDataPoint(float timestamp, float value) {
        timeAnalyzer.addDataPoint(timestamp, value);
        
        float currentTime = timeAnalyzer.getCurrentTime();
        
        // Try different time windows with geometric progression
        for (float duration = MIN_FEATURE_DURATION; 
             duration <= MAX_FEATURE_DURATION; 
             duration *= 1.5f) {
            
            float startTime = currentTime - duration;
            if (startTime < currentTime - MAX_LOOKBACK_TIME) continue;

            ExponentialFeature feature = analyzeTimeWindow(startTime, currentTime);
            
            if (feature.growthRate >= MIN_GROWTH_RATE && 
                feature.growthRate <= MAX_GROWTH_RATE) {
                featureTracker.updateFeature(feature);
                break;  // Found a valid feature, stop searching
            }
        }

        updateVisualization();
    }

    void updateVisualization() {
        size_t windowSize;
        float startTime = timeAnalyzer.getCurrentTime() - MAX_LOOKBACK_TIME;
        timeAnalyzer.getTimeWindow(startTime, timeAnalyzer.getCurrentTime(),
                                 windowTimes, windowValues, windowSize);

        display.visualizeGrowthAnalysis(windowTimes, windowValues, windowSize,
                                      featureTracker.getFeatures());
    }

    void reset() {
        timeAnalyzer = TimeWindowAnalyzer();
    }

    // Debug methods
    void printFeatureInfo() {
        const auto& features = featureTracker.getFeatures();
        Serial.println(F("Current Features:"));
        for (size_t i = 0; i < features.size(); i++) {
            const auto& feature = features[i];
            Serial.print(F("Feature "));
            Serial.print(i + 1);
            Serial.print(F(": Start="));
            Serial.print(feature.startTime);
            Serial.print(F(", End="));
            Serial.print(feature.endTime);
            Serial.print(F(", Growth="));
            Serial.print(feature.growthRate);
            Serial.print(F(", Active="));
            Serial.println(feature.active ? "Yes" : "No");
        }
    }
};

// Global instance
ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    
    if (!growthDetector.begin()) {
        Serial.println(F("Failed to initialize display"));
        while (1) delay(100);
    }
    
    Serial.println(F("Exponential Growth Detector Started"));
    Serial.println(F("Format: timestamp,value"));
}

void loop() {
    // Check for serial input
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Parse CSV format: timestamp,value
        int commaIndex = input.indexOf(',');
        if (commaIndex > 0) {
            float timestamp = input.substring(0, commaIndex).toFloat();
            float value = input.substring(commaIndex + 1).toFloat();
            
            growthDetector.addDataPoint(timestamp, value);
            
            // Print debug info
            Serial.print(F("Added point: t="));
            Serial.print(timestamp);
            Serial.print(F(", v="));
            Serial.println(value);
            
            growthDetector.printFeatureInfo();
        }
    }

    // Optional: Generate test data
    static float testTime = 0.0f;
    if (millis() % 1000 == 0) {  // Generate test point every second
        // Generate sample data with exponential growth and noise
        float value = 10.0f * exp(0.2f * testTime) + random(-80, 81) / 10.0f;
        
        growthDetector.addDataPoint(testTime, value);
        
        testTime += 1.0f + random(0, 11) / 10.0f;  // Add some time variation
        delay(1);  // Prevent multiple triggers in same millisecond
    }
}
