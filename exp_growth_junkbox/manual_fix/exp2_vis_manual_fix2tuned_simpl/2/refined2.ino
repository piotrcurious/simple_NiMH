#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <algorithm>
#include <cmath>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
#define MAX_DATASET_WINDOW 50

// Polynomial Fitter Class
class AdvancedPolynomialFitter {
private:
    constexpr static int MAX_ITERATIONS = 500;
    constexpr static double LEARNING_RATE = 1e-9;
    constexpr static float CONVERGENCE_THRESHOLD = 1e-6;
    constexpr static float L1_LAMBDA = 0.01;
    constexpr static float L2_LAMBDA = 0.001;

    enum RegularizationType { NONE, L1_LASSO, L2_RIDGE, ELASTIC_NET };

    double calculateMSE(const std::vector<float>& coeffs, const std::vector<float>& x, const std::vector<float>& y) {
        double mse = 0.0;
        for (size_t i = 0; i < x.size(); ++i) {
            double prediction = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                prediction += coeffs[j] * pow(x[i], j);
            }
            double error = prediction - y[i];
            mse += error * error;
        }
        return mse / x.size();
    }

    std::vector<float> gradientDescentFit(const std::vector<float>& x, const std::vector<float>& y, int degree, RegularizationType regType = NONE) {
        std::vector<float> coeffs(degree + 1, 0.0);
        double dynamicLearningRate = LEARNING_RATE;

        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(degree + 1, 0.0);

            for (size_t i = 0; i < x.size(); ++i) {
                double prediction = 0.0;
                for (int j = 0; j <= degree; ++j) {
                    prediction += coeffs[j] * pow(x[i], j);
                }
                double error = prediction - y[i];
                for (int j = 0; j <= degree; ++j) {
                    double gradient = 2 * error * pow(x[i], j);
                    if (regType == L1_LASSO) gradient += L1_LAMBDA * (coeffs[j] > 0 ? 1 : -1);
                    if (regType == L2_RIDGE) gradient += 2 * L2_LAMBDA * coeffs[j];
                    gradients[j] += gradient;
                }
            }

            for (int j = 0; j <= degree; ++j) {
                coeffs[j] -= dynamicLearningRate * gradients[j] / x.size();
            }

            double mse = calculateMSE(coeffs, x, y);
            if (mse < CONVERGENCE_THRESHOLD) break;
        }

        return coeffs;
    }

public:
    enum OptimizationMethod { GRADIENT_DESCENT };

    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method = GRADIENT_DESCENT) {
        if (method == GRADIENT_DESCENT) {
            return gradientDescentFit(x, y, degree);
        }
        return {};  // Default to empty if unsupported method
    }
};

// OLED Visualizer Class
class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    void begin() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            while (true);
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
    }

    void visualize(const std::vector<float>& xData, const std::vector<float>& yData, const std::vector<float>& coeffs, float growthRate) {
        display.clearDisplay();

        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH - 1);
            int y = mapFloat(yData[i], yMin, yMax, SCREEN_HEIGHT - 25, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, SCREEN_WIDTH - 1, xMin, xMax);
            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX, j);
            }
            int y = mapFloat(yFitted, yMin, yMax, SCREEN_HEIGHT - 25, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        display.setCursor(0, SCREEN_HEIGHT - 20);
        display.print("Growth Rate: ");
        display.print(growthRate, 2);
        display.display();
    }
};

// Growth Detector Class
class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer visualizer;
    std::vector<float> timestamps;
    std::vector<float> values;

public:
    void begin() {
        visualizer.begin();
    }

    void addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);
        if (timestamps.size() > MAX_DATASET_WINDOW) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
    }

    void detectAndVisualize() {
        if (timestamps.size() < 10) return;

        std::vector<float> coeffs = fitter.fitPolynomial(timestamps, values, 4);
        float growthRate = computeGrowthRate(coeffs);

        visualizer.visualize(timestamps, values, coeffs, growthRate);
    }

    float computeGrowthRate(const std::vector<float>& coeffs) {
        if (coeffs.empty()) return 0.0;

        float lastX = timestamps.back();
        float lastValue = 0.0;
        float nextValue = 0.0;

        for (size_t j = 0; j < coeffs.size(); ++j) {
            lastValue += coeffs[j] * pow(lastX, j);
            nextValue += coeffs[j] * pow(lastX + 1, j);
        }

        return (nextValue - lastValue) / lastValue;
    }
};

// Main Program
ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    growthDetector.begin();
}

void loop() {
    static float time = 0;
    float value = 1 * sin(0.2 * time) * exp(0.1 * time) + random(-80, 80) / 10.0;
    growthDetector.addDataPoint(time, value);
    growthDetector.detectAndVisualize();
    time += random(0, 100) / 100.0;
    delay(200);
}
