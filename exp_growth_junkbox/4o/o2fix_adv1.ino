#include <vector>
#include <cmath>

class PolynomialFitter {
public:
    struct FitResult {
        std::vector<float> coefficients; // Polynomial coefficients
        float error;                     // Mean squared error
    };

    FitResult fit(const float* x, const float* y, size_t n, uint8_t degree) {
        FitResult result;
        result.coefficients.resize(degree + 1, 0.0f);

        // Construct Vandermonde matrix and target vector
        std::vector<std::vector<float>> A(n, std::vector<float>(degree + 1));
        std::vector<float> b(n);

        for (size_t i = 0; i < n; ++i) {
            float xi = 1.0f;
            for (uint8_t j = 0; j <= degree; ++j) {
                A[i][j] = xi; // x^j
                xi *= x[i];
            }
            b[i] = y[i];
        }

        // Solve using normal equations: A^T * A * coeffs = A^T * b
        std::vector<std::vector<float>> ATA(degree + 1, std::vector<float>(degree + 1));
        std::vector<float> ATb(degree + 1);

        for (uint8_t i = 0; i <= degree; ++i) {
            for (uint8_t j = 0; j <= degree; ++j) {
                ATA[i][j] = 0;
                for (size_t k = 0; k < n; ++k) {
                    ATA[i][j] += A[k][i] * A[k][j];
                }
            }
            ATb[i] = 0;
            for (size_t k = 0; k < n; ++k) {
                ATb[i] += A[k][i] * b[k];
            }
        }

        // Solve ATA * coeffs = ATb using Gaussian elimination
        result.coefficients = gaussianElimination(ATA, ATb);

        // Compute error
        result.error = computeMeanSquaredError(x, y, result.coefficients, n);
        return result;
    }

private:
    std::vector<float> gaussianElimination(std::vector<std::vector<float>>& A, std::vector<float>& b) {
        uint8_t n = b.size();
        std::vector<float> x(n);

        for (uint8_t i = 0; i < n; ++i) {
            // Pivot
            for (uint8_t j = i + 1; j < n; ++j) {
                if (fabs(A[j][i]) > fabs(A[i][i])) {
                    std::swap(A[i], A[j]);
                    std::swap(b[i], b[j]);
                }
            }

            // Eliminate
            for (uint8_t j = i + 1; j < n; ++j) {
                float factor = A[j][i] / A[i][i];
                for (uint8_t k = i; k < n; ++k) {
                    A[j][k] -= factor * A[i][k];
                }
                b[j] -= factor * b[i];
            }
        }

        // Back-substitution
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (uint8_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }

    float computeMeanSquaredError(const float* x, const float* y, const std::vector<float>& coeffs, size_t n) {
        float mse = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            float prediction = 0.0f;
            float xi = 1.0f;
            for (float c : coeffs) {
                prediction += c * xi;
                xi *= x[i];
            }
            mse += (prediction - y[i]) * (prediction - y[i]);
        }
        return mse / n;
    }
};
