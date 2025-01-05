#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <random>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;

class AdvancedPolynomialFitter {
private:
    const int MAX_ITERATIONS = 100;
    const float LEARNING_RATE = 0.001;
    const float CONVERGENCE_THRESHOLD = 1e-6;

    const float L1_LAMBDA = 0.01;
    const float L2_LAMBDA = 0.01;

    enum RegularizationType {
        NONE,
        L1_LASSO,
        L2_RIDGE,
        ELASTIC_NET
    };

    enum LossFunctionType {
        MEAN_SQUARED_ERROR,
        MEAN_ABSOLUTE_ERROR,
        HUBER_LOSS
    };

    float calculateMSE(const std::vector<float>& coeffs,
                       const std::vector<float>& x,
                       const std::vector<float>& y) {
        float totalError = 0.0;
        for (size_t i = 0; i < x.size(); ++i) {
            float prediction = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                prediction += coeffs[j] * pow(x[i], j);
            }
            totalError += pow(prediction - y[i], 2);
        }
        return totalError / x.size();
    }

    std::vector<float> gradientDescentFit(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree,
        RegularizationType regType = L2_RIDGE,
        LossFunctionType lossType = MEAN_SQUARED_ERROR
    ) {
        std::vector<float> coeffs(degree + 1, 0.0);
        float dynamicLearningRate = LEARNING_RATE;
        float prevMSE = calculateMSE(coeffs, x, y);
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(coeffs.size(), 0.0);

            for (size_t j = 0; j < coeffs.size(); ++j) {
                for (size_t i = 0; i < x.size(); ++i) {
                    float prediction = 0.0;
                    for (size_t k = 0; k < coeffs.size(); ++k) {
                        prediction += coeffs[k] * pow(x[i], k);
                    }

                    float error = prediction - y[i];
                    float gradient = 2 * error * pow(x[i], j);

                    switch (regType) {
                        case L1_LASSO:
                            gradient += L1_LAMBDA * (coeffs[j] > 0 ? 1 : -1);
                            break;
                        case L2_RIDGE:
                            gradient += 2 * L2_LAMBDA * coeffs[j];
                            break;
                        case ELASTIC_NET:
                            gradient += L1_LAMBDA * (coeffs[j] > 0 ? 1 : -1) +
                                        2 * L2_LAMBDA * coeffs[j];
                            break;
                        default:
                            break;
                    }

                    gradients[j] += gradient;
                }
                gradients[j] /= x.size();
            }

            for (size_t j = 0; j < coeffs.size(); ++j) {
                coeffs[j] -= dynamicLearningRate * gradients[j];
            }

            float currentMSE = calculateMSE(coeffs, x, y);
            if (currentMSE > prevMSE) {
                dynamicLearningRate *= 0.5;
            } else if (currentMSE < prevMSE) {
                dynamicLearningRate *= 1.1;
            }

            if (abs(currentMSE - prevMSE) < CONVERGENCE_THRESHOLD) {
                break;
            }
            prevMSE = currentMSE;
        }

        return coeffs;
    }

    std::vector<float> simulatedAnnealingFit(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree
    ) {
        std::vector<float> bestCoeffs(degree + 1, 0.0);
        float bestError = std::numeric_limits<float>::max();

        float temperature = 1000.0;
        float coolingRate = 0.003;

        std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> dist(-0.5, 0.5);

        while (temperature > 1.0) {
            std::vector<float> currentCoeffs = bestCoeffs;
            for (auto& coeff : currentCoeffs) {
                coeff += dist(rng) * temperature * 0.1;
            }

            float currentError = calculateMSE(currentCoeffs, x, y);

            float deltaCost = currentError - bestError;
            float acceptanceProbability = exp(-deltaCost / temperature);

            if (deltaCost < 0 || dist(rng) < acceptanceProbability) {
                bestCoeffs = currentCoeffs;
                bestError = currentError;
            }

            temperature *= 1 - coolingRate;
        }

        return bestCoeffs;
    }

public:
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
        SIMULATED_ANNEALING
    };

    std::vector<float> fitPolynomial(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree,
        OptimizationMethod method = GRADIENT_DESCENT
    ) {
        switch (method) {
            case GRADIENT_DESCENT:
                return gradientDescentFit(x, y, degree);
            case SIMULATED_ANNEALING:
                return simulatedAnnealingFit(x, y, degree);
            default:
                return gradientDescentFit(x, y, degree);
        }
    }

    float evaluatePolynomial(
        const std::vector<float>& coeffs,
        float x,
        float& errorBound
    ) {
        float result = 0.0;
        errorBound = 0.0;

        for (size_t i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * pow(x, i);
            errorBound += abs(coeffs[i]) * pow(x, i);
        }

        return result;
    }
};

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    std::vector<float> computeDerivative(const std::vector<float>& coeffs) {
        std::vector<float> derivative(coeffs.size() - 1, 0.0);
        for (size_t i = 1; i < coeffs.size(); ++i) {
            derivative[i-1] = i * coeffs[i];
        }
        return derivative;
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    void begin() {
        Wire.begin();
        if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            for(;;);
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
    }


    void visualizeGrowthAnalysis(
        const std::vector<float>& xData,
        const std::vector<float>& yData,
        const std::vector<float>& coeffs,
        bool growthDetected,
        float growthRate
    ) {
        display.clearDisplay();

        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        std::vector<float> derivative = computeDerivative(coeffs);

        int dataPlotHeight = 40;
        int bottomInfoHeight = 24;

        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, SCREEN_WIDTH-1, xMin, xMax);

            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX, j);
            }

            int y = mapFloat(yFitted, yMin, yMax, dataPlotHeight-1, 0);
            Serial.println(yFitted);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        float derivMin = *std::min_element(derivative.begin(), derivative.end());
        float derivMax = *std::max_element(derivative.begin(), derivative.end());

        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            size_t derivIndex = mapFloat(x, 0, SCREEN_WIDTH-1, 0, derivative.size()-1);

            float derivValue = derivative[derivIndex];

            int y = mapFloat(derivValue, derivMin, derivMax, 
                             SCREEN_HEIGHT-1, dataPlotHeight);

            if (growthDetected && derivValue > 0) {
                display.drawPixel(x, y, SSD1306_WHITE);
            } else {
                display.drawPixel(x, y, SSD1306_WHITE);
            }
        }

        display.setCursor(0, dataPlotHeight + 2);
        display.print(growthDetected ? "GROWTH:" : "NO GROWTH");
        display.print(" R:");
        display.print(growthRate, 2);

        display.setCursor(0, dataPlotHeight + 12);
        display.print("Deg:");
        display.print(coeffs.size()-1);
        display.print(" Pts:");
        display.print(xData.size());

        display.display();
    }

    void displayErrorState(const char* errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
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

public:
    void begin() {
        oledViz.begin();
    }

    void addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);

        if (timestamps.size() > 100) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
    }

    float computeGrowthRate(const std::vector<float>& coeffs,
                              const std::vector<float>& xData) {
        if (xData.empty()) return 0.0;

        float lastX = xData.back();
        float lastValue = 0.0;
        float nextValue = 0.0;

        for (size_t j = 0; j < coeffs.size(); ++j) {
            lastValue += coeffs[j] * pow(lastX, j);
            nextValue += coeffs[j] * pow(lastX + 1, j);
        }

        return (nextValue - lastValue) / lastValue;
    }

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) return false;

        std::vector<int> degrees = {3, 4, 5, 6, 7};
        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> bestCoeffs;
        std::vector<AdvancedPolynomialFitter::OptimizationMethod> methods = {
            AdvancedPolynomialFitter::GRADIENT_DESCENT,
        };

        for (int degree : degrees) {
            for (auto method : methods) {
                std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                    timestamps, values, degree, method
                );

                float errorBound;
                float prediction = polynomialFitter.evaluatePolynomial(
                    coeffs, timestamps.back(), errorBound
                );

                float growthRate = computeGrowthRate(coeffs, timestamps);

                if (growthRate > 0.2 && growthRate < 10.0) {
                    growthDetected = true;
                    if (growthRate > bestGrowthRate) {
                        bestGrowthRate = growthRate;
                        bestCoeffs = coeffs;
                    }
                }
            }
        }

        oledViz.visualizeGrowthAnalysis(
            timestamps, values, bestCoeffs, 
            growthDetected, bestGrowthRate
        );
        return growthDetected;
    }
};

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    growthDetector.begin();
}

void loop() {
    static float time = 0;
    
    float value = 10 * exp(0.2 * time) + static_cast<float>(random(-80, 80)) / 10.0;
    
    growthDetector.addDataPoint(time, value);
    
    growthDetector.detectExponentialGrowth();
    
    time += static_cast<float>(random(0, 10)) / 10.0;
    delay(500);
}
