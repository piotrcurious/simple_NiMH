#include <Arduino.h>
#include <vector>
#include <numeric>
#include <cmath>

class AdvancedPolynomialFitter {
private:
    // Optimization configuration
    const int MAX_ITERATIONS = 100;
    const float LEARNING_RATE = 0.01;
    const float CONVERGENCE_THRESHOLD = 1e-6;

    // Regularization parameters
    const float L1_LAMBDA = 0.01;  // Lasso regularization
    const float L2_LAMBDA = 0.01;  // Ridge regularization

    // Optimization methods enum
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
        SIMULATED_ANNEALING
    };

    // Regularization types
    enum RegularizationType {
        NONE,
        L1_LASSO,
        L2_RIDGE,
        ELASTIC_NET
    };

    // Loss function types
    enum LossFunctionType {
        MEAN_SQUARED_ERROR,
        MEAN_ABSOLUTE_ERROR,
        HUBER_LOSS
    };

    // Helper method: Calculate Mean Squared Error
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

    // Gradient Descent with adaptive learning and regularization
    std::vector<float> gradientDescentFit(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        RegularizationType regType = ELASTIC_NET,
        LossFunctionType lossType = MEAN_SQUARED_ERROR
    ) {
        std::vector<float> coeffs(degree + 1, 0.0);
        
        // Adaptive learning rate
        float dynamicLearningRate = LEARNING_RATE;
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(coeffs.size(), 0.0);
            float prevMSE = calculateMSE(coeffs, x, y);

            // Compute gradients
            for (size_t j = 0; j < coeffs.size(); ++j) {
                for (size_t i = 0; i < x.size(); ++i) {
                    float prediction = 0.0;
                    for (size_t k = 0; k < coeffs.size(); ++k) {
                        prediction += coeffs[k] * pow(x[i], k);
                    }
                    
                    // Gradient computation with different loss functions
                    float error = prediction - y[i];
                    float gradient = 2 * error * pow(x[i], j);

                    // Regularization
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

            // Update coefficients
            for (size_t j = 0; j < coeffs.size(); ++j) {
                coeffs[j] -= dynamicLearningRate * gradients[j];
            }

            // Adaptive learning rate
            float currentMSE = calculateMSE(coeffs, x, y);
            if (currentMSE > prevMSE) {
                dynamicLearningRate *= 0.5;  // Reduce learning rate
            } else if (currentMSE < prevMSE) {
                dynamicLearningRate *= 1.1;  // Slightly increase learning rate
            }

            // Convergence check
            if (abs(currentMSE - prevMSE) < CONVERGENCE_THRESHOLD) {
                break;
            }
        }

        return coeffs;
    }

    // Simulated Annealing for global optimization
    std::vector<float> simulatedAnnealingFit(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        std::vector<float> bestCoeffs(degree + 1, 0.0);
        float bestError = std::numeric_limits<float>::max();
        
        float temperature = 1000.0;
        float coolingRate = 0.003;

        // Random number generator setup (pseudo-random for deterministic behavior)
        unsigned long seed = micros();
        
        while (temperature > 1.0) {
            // Create slightly perturbed coefficients
            std::vector<float> currentCoeffs = bestCoeffs;
            for (auto& coeff : currentCoeffs) {
                // Add random perturbation scaled by temperature
                coeff += ((random(seed) / (float)RAND_MAX) - 0.5) * temperature * 0.1;
            }

            float currentError = calculateMSE(currentCoeffs, x, y);

            // Probabilistic acceptance of worse solutions
            float deltaCost = currentError - bestError;
            float acceptanceProbability = exp(-deltaCost / temperature);
            
            if (deltaCost < 0 || ((random(seed) / (float)RAND_MAX) < acceptanceProbability)) {
                bestCoeffs = currentCoeffs;
                bestError = currentError;
            }

            // Cool down
            temperature *= 1 - coolingRate;
        }

        return bestCoeffs;
    }

public:
    // Main fitting method with multiple optimization strategies
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
                // Fallback to simple least squares if method not supported
                return gradientDescentFit(x, y, degree);
        }
    }

    // Advanced polynomial evaluation with error bounds
    float evaluatePolynomial(
        const std::vector<float>& coeffs, 
        float x, 
        float& errorBound
    ) {
        float result = 0.0;
        errorBound = 0.0;

        for (size_t i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * pow(x, i);
            // Simple error propagation
            errorBound += abs(coeffs[i]) * pow(x, i);
        }

        return result;
    }
};

// Example usage in ESP32 context
class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    std::vector<float> timestamps;
    std::vector<float> values;

public:
    void addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);

        // Limit dataset size
        if (timestamps.size() > 100) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
    }

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) return false;

        // Try different polynomial degrees and optimization methods
        std::vector<int> degrees = {3, 4, 5, 6, 7};
        std::vector<AdvancedPolynomialFitter::OptimizationMethod> methods = {
            AdvancedPolynomialFitter::GRADIENT_DESCENT,
            AdvancedPolynomialFitter::SIMULATED_ANNEALING
        };

        for (int degree : degrees) {
            for (auto method : methods) {
                std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                    timestamps, values, degree, method
                );

                // Analyze coefficients for exponential characteristics
                float errorBound;
                float prediction = polynomialFitter.evaluatePolynomial(
                    coeffs, timestamps.back(), errorBound
                );

                // Potential growth detection logic
                if (prediction > values.back() * 1.5 && errorBound < 0.1) {
                    return true;
                }
            }
        }

        return false;
    }
};

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Example usage
    ExponentialGrowthDetector detector;
    
    // Simulate data collection
    for (int i = 0; i < 50; i++) {
        float timestamp = i;
        float value = exp(0.1 * i) + random(-10, 10) / 10.0;
        detector.addDataPoint(timestamp, value);
    }

    if (detector.detectExponentialGrowth()) {
        Serial.println("Potential Exponential Growth Detected!");
    }

    delay(5000);
}
