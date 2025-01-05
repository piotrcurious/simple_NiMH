#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h> // For safe data storage

// Configuration constants
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr size_t MAX_DATA_POINTS = 100;  // Increased for larger datasets
constexpr size_t MAX_POLYNOMIAL_DEGREE = 3;
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 10.0f;

struct PolynomialFit {
    float coefficients[MAX_POLYNOMIAL_DEGREE + 1];
    uint8_t degree;
    float growthRate;
    float error;
};

class PolynomialFitter {
private:
    void computeNormalEquations(const float* x, const float* y, size_t n, uint8_t degree, float* coeffs) {
        float X[2 * MAX_POLYNOMIAL_DEGREE + 1] = {0};
        float Y[MAX_POLYNOMIAL_DEGREE + 1] = {0};

        for (size_t i = 0; i < n; ++i) {
            float xPower = 1.0f;
            for (size_t j = 0; j <= 2 * degree; ++j) {
                X[j] += xPower;
                xPower *= x[i];
            }
            xPower = y[i];
            for (size_t j = 0; j <= degree; ++j) {
                Y[j] += xPower;
                xPower *= x[i];
            }
        }

        // Solve normal equations using Gaussian elimination
        float A[MAX_POLYNOMIAL_DEGREE + 1][MAX_POLYNOMIAL_DEGREE + 2] = {0};
        for (uint8_t i = 0; i <= degree; ++i) {
            for (uint8_t j = 0; j <= degree; ++j) {
                A[i][j] = X[i + j];
            }
            A[i][degree + 1] = Y[i];
        }

        for (uint8_t i = 0; i <= degree; ++i) {
            for (uint8_t j = i + 1; j <= degree; ++j) {
                float ratio = A[j][i] / A[i][i];
                for (uint8_t k = 0; k <= degree + 1; ++k) {
                    A[j][k] -= ratio * A[i][k];
                }
            }
        }

        for (int8_t i = degree; i >= 0; --i) {
            coeffs[i] = A[i][degree + 1];
            for (uint8_t j = i + 1; j <= degree; ++j) {
                coeffs[i] -= A[i][j] * coeffs[j];
            }
            coeffs[i] /= A[i][i];
        }
    }

    float evaluatePolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float calculateError(const float* coeffs, uint8_t degree, const float* x, const float* y, size_t n) const {
        float totalError = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            float diff = evaluatePolynomial(coeffs, degree, x[i]) - y[i];
            totalError += diff * diff;
        }
        return totalError / n;
    }

public:
    PolynomialFit fit(const float* x, const float* y, size_t n, uint8_t degree) {
        PolynomialFit result;
        result.degree = degree;

        computeNormalEquations(x, y, n, degree, result.coefficients);
        result.error = calculateError(result.coefficients, degree, x, y, n);

        if (n > 0) {
            float lastX = x[n - 1];
            float lastValue = evaluatePolynomial(result.coefficients, degree, lastX);
            float nextValue = evaluatePolynomial(result.coefficients, degree, lastX + 1.0f);
            result.growthRate = (lastValue != 0.0f) ? (nextValue - lastValue) / lastValue : 0.0f;
        } else {
            result.growthRate = 0.0f;
        }

        return result;
    }
};

class ExponentialGrowthDetector {
private:
    PolynomialFitter fitter;
    AdvancedOLEDVisualizer display;

    float timestamps[MAX_DATA_POINTS];
    float values[MAX_DATA_POINTS];
    size_t dataCount = 0;

public:
    bool begin() {
        return display.begin();
    }

    void addDataPoint(float timestamp, float value) {
        if (dataCount >= MAX_DATA_POINTS) {
            memmove(timestamps, timestamps + 1, (MAX_DATA_POINTS - 1) * sizeof(float));
            memmove(values, values + 1, (MAX_DATA_POINTS - 1) * sizeof(float));
            --dataCount;
        }
        timestamps[dataCount] = timestamp;
        values[dataCount] = value;
        ++dataCount;
    }

    void detectExponentialGrowth() {
        if (dataCount < 10) {
            display.displayErrorState("Need more data");
            return;
        }

        PolynomialFit bestFit;
        bestFit.error = INFINITY;

        for (uint8_t degree = 2; degree <= MAX_POLYNOMIAL_DEGREE; ++degree) {
            PolynomialFit fit = fitter.fit(timestamps, values, dataCount, degree);
            if (fit.error < bestFit.error && fit.growthRate >= MIN_GROWTH_RATE && fit.growthRate <= MAX_GROWTH_RATE) {
                bestFit = fit;
            }
        }

        display.visualizeGrowthAnalysis(timestamps, values, dataCount, bestFit);
    }
};
