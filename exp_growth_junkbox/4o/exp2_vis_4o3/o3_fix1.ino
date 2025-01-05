// Keep all the existing includes and configuration constants...

struct DataNormalization {
    float xOffset;
    float xScale;
    float yOffset;
    float yScale;
    float meanX;
    float meanY;
    float stdX;
    float stdY;
};

struct PolynomialFit {
    float coefficients[MAX_POLYNOMIAL_DEGREE + 1];
    uint8_t degree;
    float growthRate;
    float error;
    float r2Score;  // Added R² score for fit quality assessment
    DataNormalization norm;
};

class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 100;  // Increased iterations
    static constexpr float INITIAL_LEARNING_RATE = 0.1f;
    static constexpr float MIN_LEARNING_RATE = 1e-6f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6f;
    static constexpr float L2_LAMBDA = 0.0001f;  // Reduced regularization

    DataNormalization normalizeData(float* xNorm, 
                                  float* yNorm,
                                  const float* x,
                                  const float* y,
                                  size_t n) {
        DataNormalization norm = {0};
        
        // Calculate mean and standard deviation
        for (size_t i = 0; i < n; ++i) {
            norm.meanX += x[i];
            norm.meanY += y[i];
        }
        norm.meanX /= n;
        norm.meanY /= n;
        
        for (size_t i = 0; i < n; ++i) {
            float dx = x[i] - norm.meanX;
            float dy = y[i] - norm.meanY;
            norm.stdX += dx * dx;
            norm.stdY += dy * dy;
        }
        norm.stdX = sqrt(norm.stdX / (n - 1));
        norm.stdY = sqrt(norm.stdY / (n - 1));
        
        // Prevent division by zero
        if (norm.stdX < 1e-6f) norm.stdX = 1.0f;
        if (norm.stdY < 1e-6f) norm.stdY = 1.0f;
        
        // Z-score normalization
        for (size_t i = 0; i < n; ++i) {
            xNorm[i] = (x[i] - norm.meanX) / norm.stdX;
            yNorm[i] = (y[i] - norm.meanY) / norm.stdY;
        }
        
        // Store original scale information for denormalization
        norm.xOffset = norm.meanX;
        norm.xScale = norm.stdX;
        norm.yOffset = norm.meanY;
        norm.yScale = norm.stdY;
        
        return norm;
    }

    float evaluateNormalizedPolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        float xPow = x;
        
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float calculateR2Score(const float* y, const float* yPred, size_t n) {
        float meanY = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            meanY += y[i];
        }
        meanY /= n;
        
        float ssTotal = 0.0f;
        float ssResidual = 0.0f;
        
        for (size_t i = 0; i < n; ++i) {
            float diff = y[i] - meanY;
            ssTotal += diff * diff;
            diff = y[i] - yPred[i];
            ssResidual += diff * diff;
        }
        
        return 1.0f - (ssResidual / (ssTotal + 1e-10f));
    }

public:
    PolynomialFit fitPolynomial(const float* x,
                               const float* y,
                               size_t n,
                               uint8_t degree) {
        PolynomialFit result;
        result.degree = degree;

        float xNorm[MAX_DATA_POINTS];
        float yNorm[MAX_DATA_POINTS];
        float yPred[MAX_DATA_POINTS];
        
        result.norm = normalizeData(xNorm, yNorm, x, y, n);

        // Initialize coefficients using scaled random values
        for (uint8_t i = 0; i <= degree; ++i) {
            result.coefficients[i] = (random(1000) - 500) / (1000.0f * (i + 1));
        }

        float learningRate = INITIAL_LEARNING_RATE;
        float prevError = INFINITY;
        float minError = INFINITY;
        float bestCoeffs[MAX_POLYNOMIAL_DEGREE + 1];
        
        // Multiple restarts with momentum
        for (int restart = 0; restart < 5; restart++) {
            float velocities[MAX_POLYNOMIAL_DEGREE + 1] = {0};
            const float momentum = 0.9f;
            
            for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
                float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};
                float currentError = 0.0f;
                
                // Calculate predictions and gradients
                for (size_t i = 0; i < n; ++i) {
                    float prediction = evaluateNormalizedPolynomial(result.coefficients, degree, xNorm[i]);
                    yPred[i] = prediction;
                    float error = prediction - yNorm[i];
                    currentError += error * error;
                    
                    float xPow = 1.0f;
                    for (uint8_t j = 0; j <= degree; ++j) {
                        gradients[j] += 2.0f * error * xPow;
                        xPow *= xNorm[i];
                    }
                }
                
                currentError = sqrt(currentError / n);
                
                // L2 regularization with degree-based scaling
                for (uint8_t j = 0; j <= degree; ++j) {
                    gradients[j] = gradients[j] / n + L2_LAMBDA * result.coefficients[j] * (j + 1);
                }

                // Update with momentum
                for (uint8_t j = 0; j <= degree; ++j) {
                    velocities[j] = momentum * velocities[j] - learningRate * gradients[j];
                    result.coefficients[j] += velocities[j];
                }

                // Adaptive learning rate
                if (currentError > prevError) {
                    learningRate *= 0.5f;
                    if (learningRate < MIN_LEARNING_RATE) {
                        break;
                    }
                } else if (currentError < minError) {
                    minError = currentError;
                    memcpy(bestCoeffs, result.coefficients, sizeof(bestCoeffs));
                }

                if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                    break;
                }
                prevError = currentError;
            }
            
            if (restart < 4) {
                learningRate = INITIAL_LEARNING_RATE;
                for (uint8_t i = 0; i <= degree; ++i) {
                    result.coefficients[i] = (random(1000) - 500) / (1000.0f * (i + 1));
                }
            }
        }

        memcpy(result.coefficients, bestCoeffs, sizeof(bestCoeffs));
        result.error = minError;

        // Calculate R² score
        for (size_t i = 0; i < n; ++i) {
            yPred[i] = evaluateNormalizedPolynomial(bestCoeffs, degree, xNorm[i]);
        }
        result.r2Score = calculateR2Score(yNorm, yPred, n);

        // Calculate growth rate using denormalized values
        float x1 = x[n-1];
        float x2 = x[n-1] + (result.norm.xScale / n);
        float y1 = denormalizeY(evaluateNormalizedPolynomial(bestCoeffs, degree, normalizeX(x1, result.norm)), result.norm);
        float y2 = denormalizeY(evaluateNormalizedPolynomial(bestCoeffs, degree, normalizeX(x2, result.norm)), result.norm);
        
        result.growthRate = (y1 != 0.0f) ? ((y2 - y1) / y1) : 0.0f;
        
        return result;
    }

    float normalizeX(float x, const DataNormalization& norm) {
        return (x - norm.xOffset) / norm.xScale;
    }

    float denormalizeY(float y, const DataNormalization& norm) {
        return y * norm.yScale + norm.yOffset;
    }

    float evaluatePolynomial(const PolynomialFit& fit, float x) {
        float xNorm = normalizeX(x, fit.norm);
        float yNorm = evaluateNormalizedPolynomial(fit.coefficients, fit.degree, xNorm);
        return denormalizeY(yNorm, fit.norm);
    }
};
