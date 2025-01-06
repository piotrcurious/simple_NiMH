#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>

// Configuration constants
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t MAX_DATASET_WINDOW = 50;

// Forward declarations
class AdvancedPolynomialFitter;
class AdvancedOLEDVisualizer;
class ExponentialGrowthDetector;

class AdvancedPolynomialFitter {
public:
    enum class RegularizationType {
        NONE,
        L1_LASSO,
        L2_RIDGE,
        ELASTIC_NET
    };

    enum class OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD
    };

private:
    // Optimization parameters
    static constexpr int MAX_ITERATIONS = 500;
    static constexpr double LEARNING_RATE = 1e-9;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6;
    static constexpr float L1_LAMBDA = 0.01f;
    static constexpr float L2_LAMBDA = 0.001f;

    double calculateMSE(const std::vector<float>& coeffs, 
                       const std::vector<float>& x, 
                       const std::vector<float>& y) const {
        double mean = 0.0;
        const size_t n = x.size();
        
        for (size_t i = 0; i < n; ++i) {
            float prediction = evaluatePolynomial(coeffs, x[i]);
            float error = prediction - y[i];
            double delta = (error * error - mean);
            mean += delta / (i + 1);
        }
        
        return mean;
    }

    float evaluatePolynomial(const std::vector<float>& coeffs, float x) const {
        float result = coeffs[0];
        float power = x;
        
        for (size_t i = 1; i < coeffs.size(); ++i) {
            result += coeffs[i] * power;
            power *= x;
        }
        
        return result;
    }

    std::vector<float> normalizeData(const std::vector<float>& data) const {
        float min_val = *std::min_element(data.begin(), data.end());
        float max_val = *std::max_element(data.begin(), data.end());
        float range = max_val - min_val;
        
        std::vector<float> normalized(data.size());
        std::transform(data.begin(), data.end(), normalized.begin(),
                      [min_val, range](float val) { return (val - min_val) / range; });
        
        return normalized;
    }

    std::vector<float> gradientDescentFit(const std::vector<float>& x, 
                                         const std::vector<float>& y, 
                                         size_t degree,
                                         RegularizationType regType = RegularizationType::L2_RIDGE) {
        std::vector<float> x_norm = normalizeData(x);
        std::vector<float> coeffs(degree + 1, 0.0f);
        double learning_rate = LEARNING_RATE;
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(degree + 1, 0.0f);
            double prev_mse = calculateMSE(coeffs, x_norm, y);
            
            // Compute gradients
            for (size_t j = 0; j <= degree; ++j) {
                float gradient = 0.0f;
                for (size_t i = 0; i < x_norm.size(); ++i) {
                    float prediction = evaluatePolynomial(coeffs, x_norm[i]);
                    float error = prediction - y[i];
                    gradient += error * std::pow(x_norm[i], j);
                }
                gradient = (2.0f * gradient) / x_norm.size();
                
                // Add regularization
                switch (regType) {
                    case RegularizationType::L1_LASSO:
                        gradient += L1_LAMBDA * ((coeffs[j] > 0) ? 1.0f : -1.0f);
                        break;
                    case RegularizationType::L2_RIDGE:
                        gradient += 2.0f * L2_LAMBDA * coeffs[j];
                        break;
                    case RegularizationType::ELASTIC_NET:
                        gradient += L1_LAMBDA * ((coeffs[j] > 0) ? 1.0f : -1.0f) + 
                                  2.0f * L2_LAMBDA * coeffs[j];
                        break;
                    default:
                        break;
                }
                gradients[j] = gradient;
            }
            
            // Update coefficients
            for (size_t j = 0; j <= degree; ++j) {
                coeffs[j] -= learning_rate * gradients[j];
            }
            
            // Adaptive learning rate
            double current_mse = calculateMSE(coeffs, x_norm, y);
            if (current_mse > prev_mse) {
                learning_rate *= 0.5;
            } else {
                learning_rate *= 1.05;
            }
            
            // Check convergence
            if (std::abs(current_mse - prev_mse) < CONVERGENCE_THRESHOLD) {
                break;
            }
        }
        
        return coeffs;
    }

public:
    std::vector<float> fitPolynomial(const std::vector<float>& x, 
                                    const std::vector<float>& y, 
                                    size_t degree,
                                    OptimizationMethod method = OptimizationMethod::GRADIENT_DESCENT) {
        return gradientDescentFit(x, y, degree);
    }

    float predict(const std::vector<float>& coeffs, float x, float& error_bound) {
        error_bound = 0.0f;
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
    AdvancedOLEDVisualizer() : 
        display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    bool begin() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            return false;
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        return true;
    }

    void visualizeGrowthAnalysis(const std::vector<float>& x_data, 
                                const std::vector<float>& y_data,
                                const std::vector<float>& coeffs,
                                bool growth_detected,
                                float growth_rate) {
        if (x_data.empty() || y_data.empty()) return;

        display.clearDisplay();
        
        // Find data ranges
        float x_min = *std::min_element(x_data.begin(), x_data.end());
        float x_max = *std::max_element(x_data.begin(), x_data.end());
        float y_min = *std::min_element(y_data.begin(), y_data.end());
        float y_max = *std::max_element(y_data.begin(), y_data.end());
        
        constexpr int PLOT_HEIGHT = 40;
        
        // Plot data points
        for (size_t i = 0; i < x_data.size(); ++i) {
            int x = mapFloat(x_data[i], x_min, x_max, 0, SCREEN_WIDTH - 1);
            int y = mapFloat(y_data[i], y_min, y_max, PLOT_HEIGHT - 1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }
        
        // Plot fitted curve
        float x_step = (x_max - x_min) / SCREEN_WIDTH;
        for (int i = 0; i < SCREEN_WIDTH; ++i) {
            float x = x_min + i * x_step;
            float y = 0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                y += coeffs[j] * std::pow(x - x_min, j);
            }
            int plot_y = mapFloat(y, y_min, y_max, PLOT_HEIGHT - 1, 0);
            if (plot_y >= 0 && plot_y < PLOT_HEIGHT) {
                display.drawPixel(i, plot_y, SSD1306_WHITE);
            }
        }
        
        // Display information
        display.setCursor(0, PLOT_HEIGHT + 2);
        display.print(growth_detected ? F("GROWTH:") : F("NO GROWTH"));
        display.print(F(" R:"));
        display.print(growth_rate, 2);
        
        display.setCursor(0, PLOT_HEIGHT + 12);
        display.print(F("Deg:"));
        display.print(coeffs.size() - 1);
        display.print(F(" Pts:"));
        display.print(x_data.size());
        
        display.display();
    }
};

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer oled;
    std::vector<float> timestamps;
    std::vector<float> values;
    
    static constexpr float GROWTH_THRESHOLD_MIN = 0.2f;
    static constexpr float GROWTH_THRESHOLD_MAX = 10.0f;
    static constexpr uint8_t MIN_DATA_POINTS = 10;
    
    float computeGrowthRate(const std::vector<float>& coeffs, float x) const {
        float current = 0;
        float next = 0;
        
        for (size_t i = 0; i < coeffs.size(); ++i) {
            current += coeffs[i] * std::pow(x, i);
            next += coeffs[i] * std::pow(x + 1, i);
        }
        
        return (next - current) / (std::abs(current) + 1e-6f);
    }

public:
    bool begin() {
        return oled.begin();
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
        if (timestamps.size() < MIN_DATA_POINTS) return false;
        
        constexpr uint8_t POLYNOMIAL_DEGREE = 4;
        auto coeffs = fitter.fitPolynomial(timestamps, values, POLYNOMIAL_DEGREE);
        
        float growth_rate = computeGrowthRate(coeffs, timestamps.back() - timestamps.front());
        bool growth_detected = (growth_rate > GROWTH_THRESHOLD_MIN && 
                              growth_rate < GROWTH_THRESHOLD_MAX);
        
        oled.visualizeGrowthAnalysis(timestamps, values, coeffs, 
                                    growth_detected, growth_rate);
        
        return growth_detected;
    }
};

ExponentialGrowthDetector detector;

void setup() {
    Serial.begin(115200);
    if (!detector.begin()) {
        Serial.println(F("Failed to initialize detector"));
        while (1) yield();
    }
}

void loop() {
    static float time = 0;
    float value = std::sin(0.2f * time) * std::exp(0.1f * time) + 
                  random(-80, 80) / 10.0f;
    
    detector.addDataPoint(time, value);
    detector.detectExponentialGrowth();
    
    time += random(0, 100) / 100.0f;
    delay(200);
}
