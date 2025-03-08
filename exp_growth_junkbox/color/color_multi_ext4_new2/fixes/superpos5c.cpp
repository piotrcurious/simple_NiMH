#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

class AdvancedPolynomialFitter {
public:
    std::vector<float> fitPolynomial_superposSMO(
        const std::vector<float>& x, const std::vector<float>& y, int degree);

private:
    std::vector<double> solveQR(std::vector<std::vector<double>>& A, std::vector<double>& b);
    std::pair<double, double> solveLocalSMO(
        const std::vector<double>& coeffs, size_t p, size_t q,
        const std::vector<std::vector<double>>& xPowers, 
        const std::vector<float>& y, const std::vector<size_t>& problemIndices);
};

std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superposSMO(
    const std::vector<float>& x, const std::vector<float>& y, int degree) {

    const size_t n = x.size();
    const size_t m = degree + 1;
    const double tol = 1e-6;
    const int maxIterations = 20;
    const double residualThreshold = 1e-3;

    std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            xPowers[i][j] = xPowers[i][j - 1] * x[i];
        }
    }

    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += xPowers[i][j] * y[i];
            for (size_t k = 0; k <= j; ++k) {
                ATA[j][k] += xPowers[i][j] * xPowers[i][k];
            }
        }
    }
    for (size_t j = 0; j < m; ++j)
        for (size_t k = j + 1; k < m; ++k)
            ATA[j][k] = ATA[k][j];

    std::vector<double> coeffs = solveQR(ATA, ATy);

    for (int iter = 0; iter < maxIterations; ++iter) {
        std::vector<double> residuals(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            double fitted = 0.0;
            for (size_t j = 0; j < m; ++j) {
                fitted += coeffs[j] * xPowers[i][j];
            }
            residuals[i] = y[i] - fitted;
        }

        std::vector<size_t> problemIndices;
        for (size_t i = 0; i < n; ++i) {
            if (std::abs(residuals[i]) > residualThreshold) {
                problemIndices.push_back(i);
            }
        }
        if (problemIndices.empty())
            break;

        bool anyUpdate = false;
        for (size_t p = 0; p < m; ++p) {
            for (size_t q = p + 1; q < m; ++q) {
                std::pair<double, double> updatedPair = solveLocalSMO(
                    coeffs, p, q, xPowers, y, problemIndices);

                if (std::abs(updatedPair.first - coeffs[p]) > tol ||
                    std::abs(updatedPair.second - coeffs[q]) > tol) {
                    coeffs[p] = updatedPair.first;
                    coeffs[q] = updatedPair.second;
                    anyUpdate = true;
                }
            }
        }
        if (!anyUpdate)
            break;
    }

    return std::vector<float>(coeffs.begin(), coeffs.end());
}

std::pair<double, double> AdvancedPolynomialFitter::solveLocalSMO(
    const std::vector<double>& coeffs, size_t p, size_t q,
    const std::vector<std::vector<double>>& xPowers, 
    const std::vector<float>& y, const std::vector<size_t>& problemIndices) {

    double Apq = 0.0, Aq = 0.0, bp = 0.0, bq = 0.0;
    double weightSum = 0.0;

    for (size_t i : problemIndices) {
        double weight = std::exp(-std::abs(y[i] - (coeffs[p] * xPowers[i][p] + coeffs[q] * xPowers[i][q])));
        weightSum += weight;
        Apq += weight * xPowers[i][p] * xPowers[i][q];
        Aq += weight * xPowers[i][q] * xPowers[i][q];
        bp += weight * xPowers[i][p] * y[i];
        bq += weight * xPowers[i][q] * y[i];
    }

    if (std::abs(Apq) < std::numeric_limits<double>::epsilon()) {
        return {coeffs[p], coeffs[q]};
    }

    double det = Aq * Aq - Apq * Apq;
    if (std::abs(det) < std::numeric_limits<double>::epsilon()) {
        return {coeffs[p], coeffs[q]};
    }

    double newP = (Aq * bp - Apq * bq) / det;
    double newQ = (Aq * bq - Apq * bp) / det;

    return {newP, newQ};
}

std::vector<double> AdvancedPolynomialFitter::solveQR(
    std::vector<std::vector<double>>& A, std::vector<double>& b) {
    
    const size_t n = A.size();
    const double eps = std::numeric_limits<double>::epsilon();

    for (size_t k = 0; k < n; ++k) {
        double norm_x = 0.0;
        for (size_t i = k; i < n; ++i) {
            norm_x += A[i][k] * A[i][k];
        }
        norm_x = std::sqrt(std::max(norm_x, eps));

        double alpha = (A[k][k] > 0) ? -norm_x : norm_x;
        double r = std::sqrt(std::max(0.5 * (alpha * alpha - A[k][k] * alpha), eps));

        std::vector<double> v(n, 0.0);
        v[k] = (A[k][k] - alpha) / (2 * r);
        for (size_t i = k + 1; i < n; ++i) {
            v[i] = A[i][k] / (2 * r);
        }

        for (size_t j = k; j < n; ++j) {
            double dot = 0.0;
            for (size_t i = k; i < n; ++i) {
                dot += v[i] * A[i][j];
            }
            for (size_t i = k; i < n; ++i) {
                A[i][j] -= 2 * v[i] * dot;
            }
        }

        double dot_b = 0.0;
        for (size_t i = k; i < n; ++i) {
            dot_b += v[i] * b[i];
        }
        for (size_t i = k; i < n; ++i) {
            b[i] -= 2 * v[i] * dot_b;
        }

        A[k][k] = alpha;
        for (size_t i = k + 1; i < n; ++i) {
            A[i][k] = 0.0;
        }
    }

    std::vector<double> x(n);
    for (int i = n - 1; i >= 0; --i) {
        x[i] = b[i];
        for (size_t j = i + 1; j < n; ++j) {
            x[i] -= A[i][j] * x[j];
        }
        x[i] /= (std::abs(A[i][i]) > eps) ? A[i][i] : eps;
    }
    return x;
}
