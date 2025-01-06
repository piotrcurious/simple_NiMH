#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>

// Display configuration
static constexpr uint16_t SCREEN_WIDTH = 128;
static constexpr uint16_t SCREEN_HEIGHT = 64;
static constexpr uint8_t OLED_DC = 16;
static constexpr uint8_t OLED_CS = 5;
static constexpr uint8_t OLED_RESET = 17;
static constexpr uint32_t OLED_BITRATE = 80000000;
static constexpr uint8_t SCREEN_ADDRESS = 0x3C;
static constexpr size_t MAX_DATASET_WINDOW = 50;

class AdvancedPolynomialFitter {
private:
    struct OptimizationConfig {
        static constexpr int MAX_ITERATIONS = 500;
        static constexpr double LEARNING_RATE = 1e-9;
        static constexpr float CONVERGENCE_THRESHOLD = 1e-6;
        static constexpr float L1_LAMBDA = 0.01f;
        static constexpr float L2_LAMBDA = 0.001f;
    };

    enum class RegularizationType {
        NONE,
        L1_LASSO,
        L2_RIDGE,
        ELASTIC_NET
    };

    // Welford's online algorithm for numerically stable MSE calculation
    double calculateMSE(const std::vector<float>& coeffs,
                       const std::vector<float>& x,
                       const std::vector<float>& y) const {
        double mean = 0.0;
        double M2 = 0.0;

        for (size_t i = 0; i < x.size(); ++i) {
            float prediction = evaluatePolynomial(coeffs, x[i]);
            float error = prediction - y[i];
            double squaredError = error * error;
            double delta = squaredError - mean;
            mean += delta / static_cast<double>(i + 1);
            M2 += delta * (squaredError - mean);
        }

        return mean;
    }

    float evaluatePolynomial(const std::vector<float>& coeffs, float x) const {
        float result = 0.0f;
        float power = 1.0f;
        
        for (const float coeff : coeffs) {
            result += coeff * power;
            power *= x;
        }
        
        return result;
    }

    std::vector<float> gradientDescentFit(
        const std::vector<float>& x,
        const std::vector<float>& y,
        size_t degree,
        RegularizationType regType = RegularizationType::NONE) {
        
        if (x.empty() || y.empty() || x.size() != y.size()) {
            return std::vector<float>();
        }

        // Normalize input data
        const float x_min = *std::min_element(x.begin(), x.end());
        std::vector<float> x_norm(x.size());
        std::transform(x.begin(), x.end(), x_norm.begin(),
                      [x_min](float val) { return val - x_min; });

        std::vector<float> coeffs(degree + 1, 0.0f);
        double learningRate = OptimizationConfig::LEARNING_RATE;

        for (int iteration = 0; iteration < OptimizationConfig::MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(degree + 1, 0.0f);
            
            // Compute gradients for all coefficients
            for (size_t j = 0; j <= degree; ++j) {
                float gradient = 0.0f;
                for (size_t i = 0; i < x_norm.size(); ++i) {
                    float prediction = evaluatePolynomial(coeffs, x_norm[i]);
                    float error = prediction - y[i];
                    float term = error * std::pow(x_norm[i], j);
                    gradient += term;
                }
                gradient *= 2.0f / x_norm.size();

                // Add regularization terms
                switch (regType) {
                    case RegularizationType::L1_LASSO:
                        gradient += OptimizationConfig::L1_LAMBDA * (coeffs[j] > 0 ? 1.0f : -1.0f);
                        break;
                    case RegularizationType::L2_RIDGE:
                        gradient += 2.0f * OptimizationConfig::L2_LAMBDA * coeffs[j];
                        break;
                    default:
                        break;
                }
                
                gradients[j] = gradient;
            }

            // Update coefficients
            for (size_t j = 0; j <= degree; ++j) {
                coeffs[j] -= learningRate * gradients[j];
            }
        }

        return coeffs;
    }

public:
    enum class OptimizationMethod {
        GRADIENT_DESCENT
    };

    std::vector<float> fitPolynomial(
        const std::vector<float>& x,
        const std::vector<float>& y,
        size_t degree,
        OptimizationMethod method = OptimizationMethod::GRADIENT_DESCENT) {
        
        return gradientDescentFit(x, y, degree, RegularizationType::L2_RIDGE);
    }

    float evaluatePolynomial(const std::vector<float>& coeffs,
                            float x,
                            float& errorBound) const {
        errorBound = 0.01f * std::abs(x); // Simplified error estimation
        return evaluatePolynomial(coeffs, x);
    }
};

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedOLEDVisualizer()
        : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    bool begin() {
        return display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    }

    void visualizeGrowthAnalysis(
        const std::vector<float>& xData,
        const std::vector<float>& yData,
        const std::vector<float>& coeffs,
        bool growthDetected,
        float growthRate) {
        
        if (xData.empty() || yData.empty() || coeffs.empty()) {
            displayErrorState("Invalid data");
            return;
        }

        display.clearDisplay();

        const float xMin = *std::min_element(xData.begin(), xData.end());
        const float xMax = *std::max_element(xData.begin(), xData.end());
        const float yMin = *std::min_element(yData.begin(), yData.end());
        const float yMax = *std::max_element(yData.begin(), yData.end());

        static constexpr int dataPlotHeight = 40;

        // Plot data points
        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH - 1);
            int y = mapFloat(yData[i], yMin, yMax, dataPlotHeight - 1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Plot fitted curve
        float prevY = 0;
        bool firstPoint = true;
        
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, SCREEN_WIDTH - 1, xMin, xMax);
            float yFitted = 0.0f;
            float power = 1.0f;
            
            for (const float coeff : coeffs) {
                yFitted += coeff * power;
                power *= (dataX - xMin);
            }
            
            int y = mapFloat(yFitted, yMin, yMax, dataPlotHeight - 1, 0);
            
            if (!firstPoint) {
                display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);
            }
            prevY = y;
            firstPoint = false;
        }

        // Display information
        display.setCursor(0, dataPlotHeight + 2);
        display.print(growthDetected ? "GROWTH:" : "NO GROWTH");
        display.print(" R:");
        display.print(growthRate, 2);

        display.setCursor(0, dataPlotHeight + 12);
        display.print("Deg:");
        display.print(coeffs.size() - 1);
        display.print(" Pts:");
        display.print(xData.size());

        display.display();
    }

    void displayErrorState(const String& errorMsg) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("ERROR:");
        display.println(errorMsg);
        display.display();
    }
};

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    std::vector<float> timestamps;
    std::vector<float> values;
    AdvancedOLEDVisualizer oledViz;

    static constexpr float MIN_GROWTH_RATE = 0.2f;
    static constexpr float MAX_GROWTH_RATE = 10.0f;
    static constexpr size_t MIN_DATA_POINTS = 10;

    float computeGrowthRate(const std::vector<float>& coeffs,
                           const std::vector<float>& xData) const {
        if (xData.empty()) return 0.0f;

        const float lastX = xData.back();
        float lastValue = 0.0f;
        float nextValue = 0.0f;
        float power = 1.0f;

        for (const float coeff : coeffs) {
            lastValue += coeff * power;
            nextValue += coeff * power * (lastX + 1.0f) / lastX;
            power *= lastX;
        }

        return (nextValue - lastValue) / lastValue;
    }

public:
    bool begin() {
        return oledViz.begin();
    }

    void addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);

        if (timestamps.size() > MAX_DATASET_WINDOW) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
    }

    bool detectExponentialGrowth() {
        if (timestamps.size() < MIN_DATA_POINTS) {
            return false;
        }

        static constexpr uint8_t POLYNOMIAL_DEGREE = 4;
        const std::vector<float> coeffs = polynomialFitter.fitPolynomial(
            timestamps, values, POLYNOMIAL_DEGREE,
            AdvancedPolynomialFitter::OptimizationMethod::GRADIENT_DESCENT
        );

        const float growthRate = computeGrowthRate(coeffs, timestamps);
        const bool growthDetected = (growthRate > MIN_GROWTH_RATE && 
                                   growthRate < MAX_GROWTH_RATE);

        oledViz.visualizeGrowthAnalysis(
            timestamps, values, coeffs,
            growthDetected, growthRate
        );

        return growthDetected;
    }
};

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    if (!growthDetector.begin()) {
        Serial.println(F("Failed to initialize OLED display"));
        while (true) { delay(100); }
    }
}

void loop() {
    static float time = 0.0f;
    const float value = std::sin(0.2f * time) * std::exp(0.1f * time) + 
                       random(-80, 80) / 10.0f;
    
    growthDetector.addDataPoint(time, value);
    growthDetector.detectExponentialGrowth();
    
    time += random(0, 100) / 100.0f;
    delay(200);
}
