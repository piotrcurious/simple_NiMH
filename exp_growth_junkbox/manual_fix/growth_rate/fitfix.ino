#include <Arduino.h>
#include <vector>
#include <numeric>
#include <cmath>

class AdvancedPolynomialFitter {
private:
    // Optimization configuration with adjusted parameters
    const int MAX_ITERATIONS = 1000;
    const double INITIAL_LEARNING_RATE = 0.0001;
    const float CONVERGENCE_THRESHOLD = 1e-6;
    
    // Regularization parameters with better default values
    const float L1_LAMBDA = 0.001;  // Reduced L1 penalty
    const float L2_LAMBDA = 0.0001; // Reduced L2 penalty
    
    enum RegularizationType {
        NONE,
        L1_LASSO,
        L2_RIDGE,
        ELASTIC_NET
    };

    // Improved MSE calculation with numerical stability
    double calculateMSE(const std::vector<float>& coeffs, 
                       const std::vector<float>& x, 
                       const std::vector<float>& y) {
        double mse = 0.0;
        double count = 0.0;
        
        for (size_t i = 0; i < x.size(); ++i) {
            double prediction = evaluatePolynomial(coeffs, x[i]);
            double error = prediction - y[i];
            // Use Kahan summation for better numerical stability
            double temp = mse;
            double y = error * error - temp;
            double t = count + y;
            count = t;
            mse = temp + (t - count);
        }
        
        return mse / x.size();
    }
    
    // New method: Polynomial evaluation with Horner's method
    double evaluatePolynomial(const std::vector<float>& coeffs, float x) {
        double result = coeffs.back();
        for (int i = coeffs.size() - 2; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }
    
    // Improved gradient calculation with better numerical stability
    std::vector<float> calculateGradients(
        const std::vector<float>& coeffs,
        const std::vector<float>& x,
        const std::vector<float>& y,
        RegularizationType regType) {
        
        std::vector<float> gradients(coeffs.size(), 0.0);
        const size_t n = x.size();
        
        for (size_t i = 0; i < n; ++i) {
            double prediction = evaluatePolynomial(coeffs, x[i]);
            double error = prediction - y[i];
            
            // Calculate basis functions using Horner's method
            std::vector<double> basis(coeffs.size());
            basis[0] = 1.0;
            for (size_t j = 1; j < coeffs.size(); ++j) {
                basis[j] = basis[j-1] * x[i];
            }
            
            // Update gradients
            for (size_t j = 0; j < coeffs.size(); ++j) {
                gradients[j] += 2.0 * error * basis[j] / n;
            }
        }
        
        // Add regularization terms
        for (size_t j = 0; j < coeffs.size(); ++j) {
            switch (regType) {
                case L1_LASSO:
                    gradients[j] += L1_LAMBDA * std::copysign(1.0f, coeffs[j]);
                    break;
                case L2_RIDGE:
                    gradients[j] += 2.0f * L2_LAMBDA * coeffs[j];
                    break;
                case ELASTIC_NET:
                    gradients[j] += L1_LAMBDA * std::copysign(1.0f, coeffs[j]) +
                                  2.0f * L2_LAMBDA * coeffs[j];
                    break;
                default:
                    break;
            }
        }
        
        return gradients;
    }

    // Improved gradient descent with adaptive learning rate and momentum
    std::vector<float> gradientDescentFit(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree,
        RegularizationType regType = L2_RIDGE) {
        
        // Initialize coefficients with zero-mean, small variance
        std::vector<float> coeffs(degree + 1);
        for (size_t i = 0; i < coeffs.size(); ++i) {
            coeffs[i] = (random(0, 1000) - 500) / 10000.0f;
        }
        
        // Normalize input data
        float x_mean = 0.0f, x_std = 0.0f;
        for (float val : x) {
            x_mean += val;
        }
        x_mean /= x.size();
        
        for (float val : x) {
            x_std += (val - x_mean) * (val - x_mean);
        }
        x_std = sqrt(x_std / x.size());
        
        std::vector<float> x_norm(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            x_norm[i] = (x[i] - x_mean) / x_std;
        }
        
        // Optimization variables
        double learning_rate = INITIAL_LEARNING_RATE;
        std::vector<float> momentum(coeffs.size(), 0.0f);
        const float momentum_factor = 0.9f;
        
        double prev_mse = std::numeric_limits<double>::max();
        int plateau_count = 0;
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            // Calculate gradients
            std::vector<float> gradients = calculateGradients(coeffs, x_norm, y, regType);
            
            // Update coefficients with momentum
            for (size_t j = 0; j < coeffs.size(); ++j) {
                momentum[j] = momentum_factor * momentum[j] - learning_rate * gradients[j];
                coeffs[j] += momentum[j];
            }
            
            // Adaptive learning rate
            double current_mse = calculateMSE(coeffs, x_norm, y);
            if (current_mse > prev_mse) {
                learning_rate *= 0.5;
                plateau_count++;
            } else if (current_mse < prev_mse - CONVERGENCE_THRESHOLD) {
                learning_rate *= 1.1;
                plateau_count = 0;
            }
            
            // Early stopping
            if (plateau_count > 10 || learning_rate < 1e-10) {
                break;
            }
            
            prev_mse = current_mse;
        }
        
        // Denormalize coefficients
        for (size_t i = 0; i < coeffs.size(); ++i) {
            coeffs[i] *= pow(x_std, i);
            for (int j = 0; j < i; ++j) {
                coeffs[i] *= -x_mean;
            }
        }
        
        return coeffs;
    }

public:
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD
    };
    
    std::vector<float> fitPolynomial(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree,
        OptimizationMethod method = GRADIENT_DESCENT) {
        
        if (x.size() < degree + 1) {
            // Return linear fit if not enough points
            return gradientDescentFit(x, y, 1, L2_RIDGE);
        }
        
        return gradientDescentFit(x, y, degree, L2_RIDGE);
    }
};
