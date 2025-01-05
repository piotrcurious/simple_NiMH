class PolynomialFitter {
private:
    static constexpr int MAX_EPOCHS = 1000;
    static constexpr float INITIAL_LEARNING_RATE = 0.01f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6f;
    static constexpr float L2_LAMBDA = 0.001f; // Regularization term
    static constexpr size_t BATCH_SIZE = 10;  // Mini-batch size

    // Helper: Normalize data for better stability
    void normalizeData(const float* x, const float* y, size_t n, 
                       float& xMean, float& xRange, 
                       float& yMean, float& yRange, 
                       float* normX, float* normY) {
        xMean = 0;
        yMean = 0;

        // Calculate means
        for (size_t i = 0; i < n; ++i) {
            xMean += x[i];
            yMean += y[i];
        }
        xMean /= n;
        yMean /= n;

        xRange = 0;
        yRange = 0;

        // Calculate ranges
        for (size_t i = 0; i < n; ++i) {
            xRange = max(xRange, fabs(x[i] - xMean));
            yRange = max(yRange, fabs(y[i] - yMean));
        }

        // Normalize data
        for (size_t i = 0; i < n; ++i) {
            normX[i] = (x[i] - xMean) / xRange;
            normY[i] = (y[i] - yMean) / yRange;
        }
    }

    // Calculate the polynomial using coefficients
    float evaluatePolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    // Compute gradients and error for mini-batch
    float computeGradients(const float* x, const float* y, size_t n, 
                           const float* coeffs, uint8_t degree, 
                           float* gradients) const {
        float totalError = 0;

        // Initialize gradients
        for (uint8_t i = 0; i <= degree; ++i) {
            gradients[i] = 0;
        }

        // Compute gradients and error
        for (size_t i = 0; i < n; ++i) {
            float prediction = evaluatePolynomial(coeffs, degree, x[i]);
            float error = prediction - y[i];
            totalError += error * error;

            // Update gradients
            float xPower = 1.0f;
            for (uint8_t j = 0; j <= degree; ++j) {
                gradients[j] += 2 * error * xPower / n;
                xPower *= x[i];
            }
        }

        // Apply L2 regularization
        for (uint8_t j = 0; j <= degree; ++j) {
            gradients[j] += 2 * L2_LAMBDA * coeffs[j];
        }

        return totalError / n;
    }

public:
    PolynomialFit fit(const float* x, const float* y, size_t n, uint8_t degree) {
        PolynomialFit fitResult;
        fitResult.degree = degree;

        // Allocate normalized data
        float normX[MAX_DATA_POINTS];
        float normY[MAX_DATA_POINTS];

        // Normalize data
        float xMean, xRange, yMean, yRange;
        normalizeData(x, y, n, xMean, xRange, yMean, yRange, normX, normY);

        // Initialize coefficients
        for (uint8_t i = 0; i <= degree; ++i) {
            fitResult.coefficients[i] = 0;
        }

        float learningRate = INITIAL_LEARNING_RATE;
        float prevError = INFINITY;

        // Iterative optimization
        for (int epoch = 0; epoch < MAX_EPOCHS; ++epoch) {
            float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};

            // Process data in batches
            for (size_t i = 0; i < n; i += BATCH_SIZE) {
                size_t batchSize = min(BATCH_SIZE, n - i);
                computeGradients(normX + i, normY + i, batchSize, fitResult.coefficients, degree, gradients);

                // Update coefficients
                for (uint8_t j = 0; j <= degree; ++j) {
                    fitResult.coefficients[j] -= learningRate * gradients[j];
                }
            }

            // Compute total error
            float currentError = computeGradients(normX, normY, n, fitResult.coefficients, degree, gradients);
            if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                break;
            }

            prevError = currentError;
        }

        // Denormalize coefficients
        for (uint8_t j = 0; j <= degree; ++j) {
            fitResult.coefficients[j] *= yRange / pow(xRange, j);
        }
        fitResult.coefficients[0] += yMean - fitResult.coefficients[0] * xMean;

        fitResult.error = prevError;
        return fitResult;
    }
};
