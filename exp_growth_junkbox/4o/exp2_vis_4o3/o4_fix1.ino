class RobustPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 500; // Increased iterations
    static constexpr float LEARNING_RATE = 0.01f; // Reduced and constant learning rate
    static constexpr float CONVERGENCE_THRESHOLD = 1e-5f;
    static constexpr float L2_LAMBDA = 0.01f; // Increased regularization

    DataNormalization normalizeData(float* xNorm, float* yNorm, const float* x, const float* y, size_t n) {
        DataNormalization norm = {};
        
        // Calculate mean and std
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
        norm.stdX = sqrt(norm.stdX / n);
        norm.stdY = sqrt(norm.stdY / n);

        if (norm.stdX < 1e-6f) norm.stdX = 1.0f;
        if (norm.stdY < 1e-6f) norm.stdY = 1.0f;

        for (size_t i = 0; i < n; ++i) {
            xNorm[i] = (x[i] - norm.meanX) / norm.stdX;
            yNorm[i] = (y[i] - norm.meanY) / norm.stdY;
        }

        norm.xOffset = norm.meanX;
        norm.xScale = norm.stdX;
        norm.yOffset = norm.meanY;
        norm.yScale = norm.stdY;

        return norm;
    }

    float evaluateNormalizedPolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float calculateR2Score(const float* y, const float* yPred, size_t n) {
        float meanY = 0.0f, ssTotal = 0.0f, ssResidual = 0.0f;

        for (size_t i = 0; i < n; ++i) meanY += y[i];
        meanY /= n;

        for (size_t i = 0; i < n; ++i) {
            ssTotal += (y[i] - meanY) * (y[i] - meanY);
            ssResidual += (y[i] - yPred[i]) * (y[i] - yPred[i]);
        }

        return 1.0f - (ssResidual / (ssTotal + 1e-10f));
    }

public:
    PolynomialFit fitPolynomial(const float* x, const float* y, size_t n, uint8_t degree) {
        PolynomialFit result = {};
        result.degree = degree;

        float xNorm[MAX_DATA_POINTS], yNorm[MAX_DATA_POINTS], yPred[MAX_DATA_POINTS];
        result.norm = normalizeData(xNorm, yNorm, x, y, n);

        // Initialize coefficients to 0
        for (uint8_t i = 0; i <= degree; ++i) {
            result.coefficients[i] = 0.0f;
        }

        float prevError = INFINITY;

        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};
            float error = 0.0f;

            for (size_t i = 0; i < n; ++i) {
                float prediction = evaluateNormalizedPolynomial(result.coefficients, degree, xNorm[i]);
                yPred[i] = prediction;
                float residual = prediction - yNorm[i];
                error += residual * residual;

                float xPower = 1.0f;
                for (uint8_t j = 0; j <= degree; ++j) {
                    gradients[j] += 2.0f * residual * xPower;
                    xPower *= xNorm[i];
                }
            }

            error = sqrt(error / n);

            // Add L2 regularization
            for (uint8_t j = 0; j <= degree; ++j) {
                gradients[j] = gradients[j] / n + L2_LAMBDA * result.coefficients[j];
            }

            // Update coefficients
            for (uint8_t j = 0; j <= degree; ++j) {
                result.coefficients[j] -= LEARNING_RATE * gradients[j];
            }

            if (abs(prevError - error) < CONVERGENCE_THRESHOLD) break;
            prevError = error;
        }

        result.error = prevError;

        // Calculate R² score
        for (size_t i = 0; i < n; ++i) {
            yPred[i] = evaluateNormalizedPolynomial(result.coefficients, degree, xNorm[i]);
        }
        result.r2Score = calculateR2Score(yNorm, yPred, n);

        return result;
    }
};
