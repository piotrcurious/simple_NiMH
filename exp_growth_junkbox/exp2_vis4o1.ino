#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <random>

// Configuration constants
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr size_t MAX_DATA_POINTS = 50;  // Reduced from 100 to save memory
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 10.0f;

// Forward declarations
class AdvancedPolynomialFitter;
class AdvancedOLEDVisualizer;
class ExponentialGrowthDetector;

struct FittingResult {
    std::vector<float> coefficients;
    float growthRate;
    float error;
};

class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 100;  // Reduced from 200
    static constexpr float LEARNING_RATE = 0.01f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-5f;
    static constexpr float L2_LAMBDA = 0.001f;

    float calculateError(const std::vector<float>& coeffs,
                        const std::vector<float>& x,
                        const std::vector<float>& y) const {
        float totalError = 0.0f;
        const size_t n = x.size();
        
        for (size_t i = 0; i < n; ++i) {
            float prediction = evaluatePolynomial(coeffs, x[i]);
            float diff = prediction - y[i];
            totalError += diff * diff;
        }
        return totalError / n;
    }

    float evaluatePolynomial(const std::vector<float>& coeffs, float x) const {
        float result = coeffs[0];
        float xPow = x;
        
        for (size_t i = 1; i < coeffs.size(); ++i) {
            result += coeffs[i] * xPow;
            xPow *= x;
        }
        return result;
    }

public:
    FittingResult fitPolynomial(const std::vector<float>& x,
                               const std::vector<float>& y,
                               int degree) {
        std::vector<float> coeffs(degree + 1, 0.0f);
        float learningRate = LEARNING_RATE;
        float prevError = calculateError(coeffs, x, y);
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(coeffs.size(), 0.0f);
            const size_t n = x.size();

            // Calculate gradients
            for (size_t j = 0; j < coeffs.size(); ++j) {
                for (size_t i = 0; i < n; ++i) {
                    float prediction = evaluatePolynomial(coeffs, x[i]);
                    float error = prediction - y[i];
                    float xPow = (j == 0) ? 1.0f : pow(x[i], j);
                    gradients[j] += 2.0f * error * xPow / n;
                }
                gradients[j] += 2.0f * L2_LAMBDA * coeffs[j];  // L2 regularization
            }

            // Update coefficients
            for (size_t j = 0; j < coeffs.size(); ++j) {
                coeffs[j] -= learningRate * gradients[j];
            }

            float currentError = calculateError(coeffs, x, y);
            if (currentError > prevError) {
                learningRate *= 0.5f;
            }

            if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                break;
            }
            prevError = currentError;
        }

        float growthRate = calculateGrowthRate(coeffs, x);
        return {coeffs, growthRate, prevError};
    }

    float calculateGrowthRate(const std::vector<float>& coeffs,
                            const std::vector<float>& x) const {
        if (x.empty()) return 0.0f;
        
        float lastX = x.back();
        float lastValue = evaluatePolynomial(coeffs, lastX);
        float nextValue = evaluatePolynomial(coeffs, lastX + 1.0f);
        
        return (nextValue - lastValue) / lastValue;
    }
};

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    bool begin() {
        Wire.begin();
        return display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    }

    void visualizeGrowthAnalysis(const std::vector<float>& xData,
                                const std::vector<float>& yData,
                                const std::vector<float>& coeffs,
                                float growthRate) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        if (xData.empty() || yData.empty()) {
            displayErrorState("No data");
            return;
        }

        // Plot data points and fitted curve
        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        const int plotHeight = SCREEN_HEIGHT - 24;
        
        // Draw data points
        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, plotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Draw fitted curve
        float prevY = 0;
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, SCREEN_WIDTH-1, xMin, xMax);
            float yFitted = coeffs[0];
            float xPow = dataX;
            
            for (size_t j = 1; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * xPow;
                xPow *= dataX;
            }

            int y = mapFloat(yFitted, yMin, yMax, plotHeight-1, 0);
            if (x > 0) {
                display.drawLine(x-1, prevY, x, y, SSD1306_WHITE);
            }
            prevY = y;
        }

        // Display statistics
        display.setCursor(0, plotHeight + 2);
        display.print(growthRate >= MIN_GROWTH_RATE ? "GROWTH:" : "NO GROWTH");
        display.print(" R:");
        display.print(growthRate, 2);

        display.setCursor(0, plotHeight + 12);
        display.print("Deg:");
        display.print(coeffs.size()-1);
        display.print(" N:");
        display.print(xData.size());

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

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer display;
    std::vector<float> timestamps;
    std::vector<float> values;
    
    static constexpr std::array<int, 3> POLYNOMIAL_DEGREES = {2, 3, 4};

public:
    bool begin() {
        timestamps.reserve(MAX_DATA_POINTS);
        values.reserve(MAX_DATA_POINTS);
        return display.begin();
    }

    void addDataPoint(float timestamp, float value) {
        if (timestamps.size() >= MAX_DATA_POINTS) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
        timestamps.push_back(timestamp);
        values.push_back(value);
    }

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) {
            display.displayErrorState("Insufficient data");
            return false;
        }

        FittingResult bestFit = {std::vector<float>(), 0.0f, INFINITY};
        
        for (int degree : POLYNOMIAL_DEGREES) {
            FittingResult result = fitter.fitPolynomial(timestamps, values, degree);
            
            if (result.error < bestFit.error && 
                result.growthRate >= MIN_GROWTH_RATE && 
                result.growthRate <= MAX_GROWTH_RATE) {
                bestFit = result;
            }
        }

        display.visualizeGrowthAnalysis(
            timestamps, values, bestFit.coefficients, bestFit.growthRate
        );
        
        return bestFit.growthRate >= MIN_GROWTH_RATE;
    }
};

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    if (!growthDetector.begin()) {
        Serial.println(F("Failed to initialize display"));
        while (1) delay(100);
    }
}

void loop() {
    static float time = 0.0f;
    
    // Generate sample data with exponential growth and noise
    float value = 10.0f * exp(0.2f * time) + random(-80, 81) / 10.0f;
    
    growthDetector.addDataPoint(time, value);
    growthDetector.detectExponentialGrowth();
    
    time += random(0, 11) / 10.0f;
    delay(500);
}
