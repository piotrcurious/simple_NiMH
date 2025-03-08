#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <utility>

// This is a placeholder for your QR solver.
// Assume that solveQR() is defined as in your original code.
std::vector<double> solveQR(std::vector<std::vector<double>>& A, std::vector<double>& b);

class AdvancedPolynomialFitter {
public:
    enum OptimizationMethod { DEFAULT };

    // Main fitting function using the SMO-inspired multi-step refinement
    std::vector<float> fitPolynomial_superposSMO(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree,
        OptimizationMethod method = DEFAULT
    );

    // Local SMO update for a given pair of coefficients
    std::pair<double, double> solveLocalSMO(
        const std::vector<double>& coeffs,
        size_t p,
        size_t q,
        const std::vector<std::vector<double>>& xPowers,
        const std::vector<float>& y,
        const std::vector<size_t>& problemIndices
    );
};

std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superposSMO(
    const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method) {

    const size_t n = x.size();
    const size_t m = degree + 1;
    const double tol = 1e-6;              // Convergence tolerance for coefficient updates
    const int maxIterations = 20;         // Maximum number of global refinement iterations
    const double residualThreshold = 1e-3; // Threshold to consider a residual as problematic

    // Precompute the Vandermonde matrix (monomial powers)
    std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            xPowers[i][j] = xPowers[i][j - 1] * x[i];
        }
    }

    // Construct initial equation system: ATA and ATy
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
    // Fill the symmetric part
    for (size_t j = 0; j < m; ++j)
        for (size_t k = j + 1; k < m; ++k)
            ATA[j][k] = ATA[k][j];

    // Initial global solution via QR decomposition
    std::vector<double> coeffs = solveQR(ATA, ATy);

    // --- Begin Multi-Step SMO-like Refinement ---
    for (int iter = 0; iter < maxIterations; ++iter) {
        // 1. Compute residuals: r_i = y_i - (Vandermonde * coeffs)_i
        std::vector<double> residuals(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            double fitted = 0.0;
            for (size_t j = 0; j < m; ++j) {
                fitted += coeffs[j] * xPowers[i][j];
            }
            residuals[i] = y[i] - fitted;
        }

        // 2. Identify problematic indices where residual exceeds the threshold
        std::vector<size_t> problemIndices;
        for (size_t i = 0; i < n; ++i) {
            if (std::abs(residuals[i]) > residualThreshold)
                problemIndices.push_back(i);
        }
        if (problemIndices.empty())
            break; // Acceptable solution reached

        // 3. For each pair of coefficients, perform a local SMO update.
        // The local objective is to adjust only coefficients p and q to reduce errors on the problematic points.
        bool anyUpdate = false;
        for (size_t p = 0; p < m; ++p) {
            for (size_t q = p + 1; q < m; ++q) {
                std::pair<double, double> updatedPair = solveLocalSMO(coeffs, p, q, xPowers, y, problemIndices);
                if (std::abs(updatedPair.first - coeffs[p]) > tol ||
                    std::abs(updatedPair.second - coeffs[q]) > tol) {
                    coeffs[p] = updatedPair.first;
                    coeffs[q] = updatedPair.second;
                    anyUpdate = true;
                }
            }
        }
        if (!anyUpdate)
            break;  // No significant updates; terminate refinement
    }
    // --- End Multi-Step Refinement ---

    // Return the final coefficients converted to float.
    return std::vector<float>(coeffs.begin(), coeffs.end());
}

std::pair<double, double> AdvancedPolynomialFitter::solveLocalSMO(
    const std::vector<double>& coeffs,
    size_t p,
    size_t q,
    const std::vector<std::vector<double>>& xPowers,
    const std::vector<float>& y,
    const std::vector<size_t>& problemIndices) {

    // Set up the 2x2 system for the adjustments Δ_p and Δ_q.
    // We aim to minimize:
    //   E_local = sum_{i in problemIndices} w_i * [r_i - Δ_p * xPowers[i][p] - Δ_q * xPowers[i][q]]^2,
    // where r_i = y[i] - (current prediction) and w_i = exp(-|r_i|).
    double A11 = 0.0, A22 = 0.0, A12 = 0.0;
    double b1 = 0.0, b2 = 0.0;

    for (size_t idx : problemIndices) {
        double fitted = 0.0;
        // Compute the current fitted value at index idx using all coefficients
        for (size_t j = 0; j < coeffs.size(); ++j) {
            fitted += coeffs[j] * xPowers[idx][j];
        }
        double r = y[idx] - fitted;
        double w = std::exp(-std::abs(r)); // Weight to focus on large residuals
        double xp = xPowers[idx][p];
        double xq = xPowers[idx][q];
        A11 += w * xp * xp;
        A22 += w * xq * xq;
        A12 += w * xp * xq;
        b1 += w * r * xp;
        b2 += w * r * xq;
    }

    // Solve the local 2x2 linear system:
    //   [A11 A12] [Δ_p] = [b1]
    //   [A12 A22] [Δ_q]   [b2]
    double det = A11 * A22 - A12 * A12;
    double Delta_p = 0.0, Delta_q = 0.0;
    if (std::abs(det) > 1e-12) {
        Delta_p = (A22 * b1 - A12 * b2) / det;
        Delta_q = (A11 * b2 - A12 * b1) / det;
    }
    // Update the coefficients only for indices p and q
    double new_cp = coeffs[p] + Delta_p;
    double new_cq = coeffs[q] + Delta_q;
    return std::make_pair(new_cp, new_cq);
}
