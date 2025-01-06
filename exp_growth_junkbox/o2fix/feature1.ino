#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>

// Configuration constants
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr size_t MAX_DATA_POINTS = 30;
constexpr size_t MAX_POLYNOMIAL_DEGREE = 3;
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 10.0f;

// New constants for feature detection and tracking
constexpr size_t MAX_FEATURES = 3;  // Track only last 3 features
constexpr float MIN_FEATURE_DURATION = 5.0f;  // Minimum time span for a valid feature
constexpr float MAX_FEATURE_DURATION = 30.0f;  // Maximum time span for a valid feature
constexpr float MAX_LOOKBACK_TIME = 60.0f;    // Maximum time to look backward
constexpr float MIN_CONFIDENCE_THRESHOLD = 0.8f;

// Structure to store detected exponential growth features
struct ExponentialFeature {
    float startTime;
    float endTime;
    float growthRate;
    float confidence;
    bool active;
};

// New class for feature scaling and detection
class FeatureScaler {
private:
    float minValue;
    float maxValue;
    float scaledMin;
    float scaledMax;

public:
    FeatureScaler(float min = 0.0f, float max = 1.0f) 
        : minValue(min), maxValue(max), scaledMin(0.0f), scaledMax(1.0f) {}

    void updateBounds(float newMin, float newMax) {
        minValue = newMin;
        maxValue = newMax;
    }

    float scale(float value) const {
        return mapFloat(value, minValue, maxValue, scaledMin, scaledMax);
    }

    float unscale(float scaled) const {
        return mapFloat(scaled, scaledMin, scaledMax, minValue, maxValue);
    }

private:
    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        if (in_max == in_min) return out_min;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

// Enhanced ExponentialGrowthDetector with feature tracking
class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer display;
    FeatureScaler scaler;
    
    CircularBuffer<float, MAX_DATA_POINTS> timestamps;
    CircularBuffer<float, MAX_DATA_POINTS> values;
    
    ExponentialFeature features[MAX_FEATURES];
    size_t featureCount;
    
    float windowStart;
    float windowEnd;

    struct SlidingWindow {
        float startTime;
        float endTime;
        size_t dataCount;
    };

public:
    ExponentialGrowthDetector() : featureCount(0), windowStart(0), windowEnd(0) {
        for (auto& feature : features) {
            feature.active = false;
        }
    }

    bool begin() {
        return display.begin();
    }

    void addDataPoint(float timestamp, float value) {
        // Update sliding window
        if (timestamps.isEmpty()) {
            windowStart = timestamp;
        }
        windowEnd = timestamp;

        // Remove old data points outside the maximum lookback time
        while (!timestamps.isEmpty() && 
               (windowEnd - timestamps.first()) > MAX_LOOKBACK_TIME) {
            timestamps.shift();
            values.shift();
        }

        // Add new data point
        timestamps.push(timestamp);
        values.push(value);

        // Update scaler bounds
        float minVal = values.first(), maxVal = values.first();
        for (size_t i = 0; i < values.size(); i++) {
            minVal = min(minVal, values[i]);
            maxVal = max(maxVal, values[i]);
        }
        scaler.updateBounds(minVal, maxVal);
    }

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) {
            display.displayErrorState("Need more data");
            return false;
        }

        // Create sliding windows for feature detection
        vector<SlidingWindow> windows = createSlidingWindows();
        
        // Analyze each window for exponential growth
        for (const auto& window : windows) {
            if (isExponentialGrowth(window)) {
                updateFeatures(window);
            }
        }

        // Remove inactive features
        cleanupFeatures();

        // Visualize current features
        displayFeatures();

        return featureCount > 0;
    }

private:
    vector<SlidingWindow> createSlidingWindows() {
        vector<SlidingWindow> windows;
        
        float minWindow = MIN_FEATURE_DURATION;
        float maxWindow = min(MAX_FEATURE_DURATION, windowEnd - windowStart);
        
        // Create overlapping windows of different sizes
        for (float windowSize = minWindow; windowSize <= maxWindow; 
             windowSize *= 1.5f) {
            for (float start = windowEnd - windowSize; 
                 start >= max(windowStart, windowEnd - MAX_LOOKBACK_TIME);
                 start -= windowSize * 0.5f) {
                
                SlidingWindow window;
                window.startTime = start;
                window.endTime = start + windowSize;
                window.dataCount = countPointsInWindow(window);
                
                if (window.dataCount >= 10) {
                    windows.push_back(window);
                }
            }
        }
        
        return windows;
    }

    size_t countPointsInWindow(const SlidingWindow& window) {
        size_t count = 0;
        for (size_t i = 0; i < timestamps.size(); i++) {
            if (timestamps[i] >= window.startTime && 
                timestamps[i] <= window.endTime) {
                count++;
            }
        }
        return count;
    }

    bool isExponentialGrowth(const SlidingWindow& window) {
        // Extract data points in window
        vector<float> windowTimes, windowValues;
        for (size_t i = 0; i < timestamps.size(); i++) {
            if (timestamps[i] >= window.startTime && 
                timestamps[i] <= window.endTime) {
                windowTimes.push_back(scaler.scale(timestamps[i]));
                windowValues.push_back(scaler.scale(values[i]));
            }
        }

        // Fit polynomial and check growth rate
        PolynomialFit fit = fitter.fitPolynomial(
            windowTimes.data(), windowValues.data(), 
            windowTimes.size(), MAX_POLYNOMIAL_DEGREE
        );

        return fit.growthRate >= MIN_GROWTH_RATE && 
               fit.growthRate <= MAX_GROWTH_RATE &&
               fit.error < (1.0f - MIN_CONFIDENCE_THRESHOLD);
    }

    void updateFeatures(const SlidingWindow& window) {
        // Check if window overlaps with existing features
        for (auto& feature : features) {
            if (feature.active && 
                intervalsOverlap(feature.startTime, feature.endTime,
                               window.startTime, window.endTime)) {
                // Merge or update existing feature
                feature.startTime = min(feature.startTime, window.startTime);
                feature.endTime = max(feature.endTime, window.endTime);
                return;
            }
        }

        // Add new feature if space available
        if (featureCount < MAX_FEATURES) {
            size_t idx = featureCount++;
            features[idx].startTime = window.startTime;
            features[idx].endTime = window.endTime;
            features[idx].active = true;
        } else {
            // Replace oldest feature
            size_t oldestIdx = 0;
            float oldestTime = features[0].startTime;
            for (size_t i = 1; i < MAX_FEATURES; i++) {
                if (features[i].startTime < oldestTime) {
                    oldestTime = features[i].startTime;
                    oldestIdx = i;
                }
            }
            features[oldestIdx].startTime = window.startTime;
            features[oldestIdx].endTime = window.endTime;
            features[oldestIdx].active = true;
        }
    }

    void cleanupFeatures() {
        for (size_t i = 0; i < MAX_FEATURES; i++) {
            if (features[i].active && 
                features[i].endTime < windowEnd - MAX_LOOKBACK_TIME) {
                features[i].active = false;
                if (i < featureCount - 1) {
                    features[i] = features[featureCount - 1];
                }
                featureCount--;
            }
        }
    }

    void displayFeatures() {
        display.clearDisplay();
        
        // Display data points and features
        for (size_t i = 0; i < timestamps.size(); i++) {
            float x = scaler.scale(timestamps[i]);
            float y = scaler.scale(values[i]);
            display.drawPoint(x, y);
        }

        // Highlight active features
        for (size_t i = 0; i < featureCount; i++) {
            if (features[i].active) {
                display.highlightRegion(
                    scaler.scale(features[i].startTime),
                    scaler.scale(features[i].endTime)
                );
            }
        }

        display.display();
    }

    static bool intervalsOverlap(float start1, float end1, 
                               float start2, float end2) {
        return start1 <= end2 && start2 <= end1;
    }
};
