#include "PolynomialFitter.h"
#include <algorithm>
#include <cmath>
#include <numeric>

double PolynomialFitter::calculateMSE(const std::vector<float>& coeffs,
                                       const std::vector<float>& x,
                                       const std::vector<float>& y) {
    double meanSquaredError = 0.0;
    double mean = 0.0;
    double M2 = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        float prediction = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            prediction += coeffs[j] * pow(x[i], j);
        }
        float error = prediction - y[i];
        double squaredError = error * error;
        double delta = squaredError - mean;
        mean += delta / (i + 1);
        M2 += delta * (squaredError - mean);
    }

    meanSquaredError = mean;
    return meanSquaredError;
}

std::vector<float> PolynomialFitter::fitPolynomial(const std::vector<float>& x,
                                                    const std::vector<float>& y,
                                                    int degree, OptimizationMethod method = GRADIENT_DESCENT) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};
    }

    std::vector<float> x_norm = x;
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    std::transform(x.begin(), x.end(), x_norm.begin(), [x_min](double val) { return val - x_min; });

    size_t n = x_norm.size();
    size_t m = degree + 1;

    std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        double xi = 1.0;
        for (size_t j = 0; j < m; ++j) {
            A[i][j] = xi;
            xi *= x_norm[i];
        }
    }

    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += A[i][j] * y[i];
            for (size_t k = 0; k < m; ++k) {
                ATA[j][k] += A[i][j] * A[i][k];
            }
        }
    }

    std::vector<double> coeffs = solveLinearSystem(ATA, ATy);
    return std::vector<float>(coeffs.begin(), coeffs.end());
}

std::vector<double> PolynomialFitter::solveLinearSystem(std::vector<std::vector<double>>& A,
                                                        std::vector<double>& b) {
    size_t n = A.size();

    for (size_t k = 0; k < n; ++k) {
        for (size_t i = k + 1; i < n; ++i) {
            if (fabs(A[i][k]) > fabs(A[k][k])) {
                std::swap(A[k], A[i]);
                std::swap(b[k], b[i]);
            }
        }

        for (size_t i = k + 1; i < n; ++i) {
            double factor = A[i][k] / A[k][k];
            for (size_t j = k; j < n; ++j) {
                A[i][j] -= factor * A[k][j];
            }
            b[i] -= factor * b[k];
        }
    }

    std::vector<double> x(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        x[i] = b[i];
        for (size_t j = i + 1; j < n; ++j) {
            x[i] -= A[i][j] * x[j];
        }
        x[i] /= A[i][i];
    }

    return x;
}
