std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(
    const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method) {

    const size_t n = x.size();
    const size_t m = degree + 1;
    const int maxIterations = 10;         // Maximum number of refinement iterations
    const double tol = 1e-6;              // Convergence tolerance for coefficient updates

    // Precompute powers for monomial basis: x^0, x^1, ... x^degree
    std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            xPowers[i][j] = xPowers[i][j - 1] * x[i];
        }
    }

    // Build the global (unweighted) system: ATA and ATy
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
    // Fill the symmetric part of ATA
    for (size_t j = 0; j < m; ++j) {
        for (size_t k = j + 1; k < m; ++k) {
            ATA[j][k] = ATA[k][j];
        }
    }

    // Initial solution via QR decomposition on the global system
    std::vector<double> coeffs = solveQR(ATA, ATy);

    // Multi-step refinement using the superposition principle
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Compute residuals at each data point using the current coefficients
        std::vector<double> residuals(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            double fitted_value = 0.0;
            for (size_t j = 0; j < m; ++j) {
                fitted_value += coeffs[j] * xPowers[i][j];
            }
            residuals[i] = y[i] - fitted_value;
        }

        // Build an alternative weighted system using the residuals
        std::vector<std::vector<double>> ATA_alt(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy_alt(m, 0.0);
        for (size_t i = 0; i < n; ++i) {
            // Exponential weight: points with high residuals are down-weighted.
            // You can experiment with the weight function if needed.
            double weight = std::exp(-std::abs(residuals[i]));
            for (size_t j = 0; j < m; ++j) {
                ATy_alt[j] += weight * xPowers[i][j] * y[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA_alt[j][k] += weight * xPowers[i][j] * xPowers[i][k];
                }
            }
        }
        // Fill the symmetric part of the alternative matrix
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA_alt[j][k] = ATA_alt[k][j];
            }
        }

        // Solve the new weighted system via QR decomposition
        std::vector<double> refined_coeffs = solveQR(ATA_alt, ATy_alt);

        // Check for convergence: if the maximum change is below tolerance, break
        double maxChange = 0.0;
        for (size_t j = 0; j < m; ++j) {
            maxChange = std::max(maxChange, std::abs(refined_coeffs[j] - coeffs[j]));
        }
        coeffs = refined_coeffs;  // update coefficients for the next iteration

        if (maxChange < tol) {
            break;  // convergence achieved
        }
    }

    // Convert the final double coefficients to float before returning
    std::vector<float> result(coeffs.begin(), coeffs.end());
    return result;
}
