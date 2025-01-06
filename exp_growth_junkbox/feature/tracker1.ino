#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>

// Previous display configuration constants remain the same...

// Time-based window configuration
constexpr float MAX_FEATURE_DURATION = 300.0f;    // Maximum duration for a feature in seconds
constexpr float MIN_FEATURE_DURATION = 10.0f;     // Minimum duration for a feature in seconds
constexpr float MAX_LOOKBACK_TIME = 600.0f;       // Maximum time to look back for features
constexpr float MIN_TIME_STEP = 0.1f;            // Minimum time between samples
constexpr size_t MAX_FEATURES = 3;               // Track only last 3 features

struct DataPoint {
    float timestamp;
    float value;
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

class TimeWindowAnalyzer {
private:
    CircularBuffer<DataPoint, MAX_DATA_POINTS> dataBuffer;
    float currentTime = 0.0f;

    bool isValidTimeWindow(float startTime, float endTime) const {
        float duration = endTime - startTime;
        return duration >= MIN_FEATURE_DURATION && duration <= MAX_FEATURE_DURATION;
    }

    // Get indices for a time window
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
        
        // Remove old data points beyond MAX_LOOKBACK_TIME
        while (!dataBuffer.isEmpty() && 
               (currentTime - dataBuffer.first().timestamp > MAX_LOOKBACK_TIME)) {
            dataBuffer.shift();
        }

        // Add new data point
        DataPoint point = {timestamp, value};
        if (dataBuffer.isFull()) {
            dataBuffer.shift();
        }
        dataBuffer.push(point);
    }

    // Get data for a specific time window
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
                // Update existing feature
                lastFeature.endTime = newFeature.endTime;
                lastFeature.finalValue = newFeature.finalValue;
                lastFeature.error = (lastFeature.error + newFeature.error) / 2;
                return;
            }
        }

        // Mark previous active feature as inactive
        if (!features.isEmpty()) {
            features.last().active = false;
        }

        // Add new feature
        if (features.isFull()) {
            features.shift();
        }
        features.push(newFeature);
    }

    const CircularBuffer<ExponentialFeature, MAX_FEATURES>& getFeatures() const {
        return features;
    }
};

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

        // Scale values to improve numerical stability
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

        // Normalize timestamps relative to start
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
        
        // Analyze potential features with varying time scales
        float currentTime = timeAnalyzer.getCurrentTime();
        
        // Try different time windows
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
        // Implementation remains the same...
    }
};

// Update AdvancedOLEDVisualizer to use CircularBuffer
class AdvancedOLEDVisualizer {
    // Previous code remains the same...

    void visualizeGrowthAnalysis(const float* xData,
                                const float* yData,
                                size_t dataSize,
                                const CircularBuffer<ExponentialFeature, MAX_FEATURES>& features) {
        // Previous visualization code adapted to use CircularBuffer...
        
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
};
