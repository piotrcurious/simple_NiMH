#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <cmath>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;

#define MAX_DATASET_WINDOW 50

class AdvancedPolynomialFitter {
private:
    // Solve Ax = b using Cholesky Decomposition
    bool choleskySolve(const std::vector<std::vector<double>>& A, const std::vector<double>& b, std::vector<double>& x) {
        int n = A.size();
        std::vector<std::vector<double>> L(n, std::vector<double>(n, 0.0));

        // Decompose A into L*L^T
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                double sum = 0.0;
                for (int k = 0; k < j; ++k) {
                    sum += L[i][k] * L[j][k];
                }
                if (i == j) {
                    L[i][j] = sqrt(A[i][i] - sum);
                } else {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }

        // Solve L*y = b
        std::vector<double> y(n, 0.0);
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            for (int j = 0; j < i; ++j) {
                sum += L[i][j] * y[j];
            }
            y[i] = (b[i] - sum) / L[i][i];
        }

        // Solve L^T*x = y
        x.assign(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            for (int j = i + 1; j < n; ++j) {
                sum += L[j][i] * x[j];
            }
            x[i] = (y[i] - sum) / L[i][i];
        }

        return true;
    }

    // Fit polynomial using Normal Equations
    std::vector<float> normalEquationFit(const std::vector<float>& x, const std::vector<float>& y, int degree, float l2_lambda = 0.0) {
        int n = x.size();
        int m = degree + 1;

        // Create design matrix X and vector y
        std::vector<std::vector<double>> X(n, std::vector<double>(m, 0.0));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                X[i][j] = pow(x[i], j);
            }
        }

        // Compute A = X^T * X and b = X^T * y
        std::vector<std::vector<double>> A(m, std::vector<double>(m, 0.0));
        std::vector<double> b(m, 0.0);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                b[j] += X[i][j] * y[i];
                for (int k = 0; k < m; ++k) {
                    A[j][k] += X[i][j] * X[i][k];
                }
            }
        }

        // Add L2 regularization
        for (int i = 0; i < m; ++i) {
            A[i][i] += l2_lambda;
        }

        // Solve Ax = b
        std::vector<double> coeffs(m, 0.0);
        choleskySolve(A, b, coeffs);

        // Convert to float for return
        std::vector<float> result(m, 0.0f);
        for (int i = 0; i < m; ++i) {
            result[i] = static_cast<float>(coeffs[i]);
        }

        return result;
    }

public:
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree) {
        return normalEquationFit(x, y, degree, 0.001); // L2 regularization
    }

    float evaluatePolynomial(const std::vector<float>& coeffs, float x) {
        float result = 0.0;
        for (size_t i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }
};
