#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>

// Configuration constants
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr size_t MAX_DATA_POINTS = 30;
constexpr size_t MAX_POLYNOMIAL_DEGREE = 3;
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 10.0f;
constexpr float MAX_TIME_WINDOW = 30.0f;  // Sliding window in seconds
constexpr size_t MAX_FEATURES = 3;       // Track last 3 features

// Feature structure
struct Feature {
    float startTime;
    float endTime;
    float growthRate;
};

// Static buffer for tracking features
CircularBuffer<Feature, MAX_FEATURES> featureBuffer;

class ExponentialGrowthDetector {
private:
    float timestamps[MAX_DATA_POINTS];
    float values[MAX_DATA_POINTS];
    size_t dataCount = 0;

    // Sliding window constraints
    void applySlidingWindow(float currentTime) {
        size_t validCount = 0;
        for (size_t i = 0; i < dataCount; ++i) {
            if (timestamps[i] >= currentTime - MAX_TIME_WINDOW) {
                timestamps[validCount] = timestamps[i];
                values[validCount] = values[i];
                validCount++;
            }
        }
        dataCount = validCount;
    }

    // Min-max scaling for normalization
    void scaleData(float &xMin, float &xMax, float &yMin, float &yMax) {
        xMin = timestamps[0];
        xMax = timestamps[0];
        yMin = values[0];
        yMax = values[0];

        for (size_t i = 1; i < dataCount; ++i) {
            xMin = min(xMin, timestamps[i]);
            xMax = max(xMax, timestamps[i]);
            yMin = min(yMin, values[i]);
            yMax = max(yMax, values[i]);
        }

        // Scale the data
        for (size_t i = 0; i < dataCount; ++i) {
            timestamps[i] = (timestamps[i] - xMin) / (xMax - xMin);
            values[i] = (values[i] - yMin) / (yMax - yMin);
        }
    }

    // Add detected feature to the buffer
    void storeFeature(float startTime, float endTime, float growthRate) {
        if (featureBuffer.isFull()) {
            featureBuffer.pop();
        }
        featureBuffer.push({startTime, endTime, growthRate});
    }

public:
    void addDataPoint(float timestamp, float value) {
        if (dataCount >= MAX_DATA_POINTS) {
            for (size_t i = 1; i < MAX_DATA_POINTS; ++i) {
                timestamps[i - 1] = timestamps[i];
                values[i - 1] = values[i];
            }
            dataCount = MAX_DATA_POINTS - 1;
        }

        timestamps[dataCount] = timestamp;
        values[dataCount] = value;
        dataCount++;
    }

    bool detectExponentialGrowth(float currentTime) {
        if (dataCount < 10) return false;

        applySlidingWindow(currentTime);

        // Scale data
        float xMin, xMax, yMin, yMax;
        scaleData(xMin, xMax, yMin, yMax);

        // Fit polynomial to detect exponential growth
        PolynomialFit bestFit = fitter.fitPolynomial(timestamps, values, dataCount, 2);
        if (bestFit.growthRate >= MIN_GROWTH_RATE && bestFit.growthRate <= MAX_GROWTH_RATE) {
            float startTime = xMin * (xMax - xMin) + xMin;
            float endTime = xMax * (xMax - xMin) + xMin;
            storeFeature(startTime, endTime, bestFit.growthRate);
            return true;
        }

        return false;
    }

    void displayFeatures() {
        for (size_t i = 0; i < featureBuffer.size(); ++i) {
            const Feature &feature = featureBuffer[i];
            Serial.print("Feature ");
            Serial.print(i);
            Serial.print(": Start=");
            Serial.print(feature.startTime, 2);
            Serial.print(", End=");
            Serial.print(feature.endTime, 2);
            Serial.print(", GrowthRate=");
            Serial.println(feature.growthRate, 2);
        }
    }

    void reset() {
        dataCount = 0;
    }
};

// Global instance
ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    Serial.println(F("Initializing Exponential Growth Detector..."));
    growthDetector.reset();
}

void loop() {
    static float time = 0.0f;

    // Simulate data
    float value = 10.0f * exp(0.2f * time) + random(-80, 81) / 10.0f;

    // Add data point and detect growth
    growthDetector.addDataPoint(time, value);
    if (growthDetector.detectExponentialGrowth(time)) {
        Serial.println(F("Exponential growth detected!"));
    }

    growthDetector.displayFeatures();

    time += random(1, 3) / 10.0f;
    delay(500);
}
