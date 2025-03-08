std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(
    const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method) {

    const size_t n = x.size();
    const size_t m = degree + 1;
    const double tolerance = 1e-6;  // Convergence threshold
    const int max_iterations = 10;  // Maximum refinement steps

    // Precompute x powers
    std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            xPowers[i][j] = xPowers[i][j - 1] * x[i];
        }
    }

    // Initial equation set
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

    // Fill symmetric part
    for (size_t j = 0; j < m; ++j) {
        for (size_t k = j + 1; k < m; ++k) {
            ATA[j][k] = ATA[k][j];
        }
    }

    // Solve initial system
    std::vector<double> coeffs = solveQR(ATA, ATy);

    // **Multi-Step Superposition Refinement**
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        // Compute residuals
        std::vector<double> residuals(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            double fitted_value = 0.0;
            for (size_t j = 0; j < m; ++j) {
                fitted_value += coeffs[j] * xPowers[i][j];
            }
            residuals[i] = y[i] - fitted_value;
        }

        // Check for convergence (small residuals)
        double max_residual = 0.0;
        for (double r : residuals) max_residual = std::max(max_residual, std::abs(r));
        if (max_residual < tolerance) break;  // Converged

        // Construct new alternative equation set
        std::vector<std::vector<double>> ATA_alt(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy_alt(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            double weight = std::exp(-std::abs(residuals[i]));  // Suppresses large oscillations
            for (size_t j = 0; j < m; ++j) {
                ATy_alt[j] += weight * xPowers[i][j] * (y[i] - residuals[i] / 2.0);  // Superposition correction
                for (size_t k = 0; k <= j; ++k) {
                    ATA_alt[j][k] += weight * xPowers[i][j] * xPowers[i][k];
                }
            }
        }

        // Fill symmetric part
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA_alt[j][k] = ATA_alt[k][j];
            }
        }

        // Solve refined system
        std::vector<double> refined_coeffs = solveQR(ATA_alt, ATy_alt);

        // Compute change in coefficients
        double max_change = 0.0;
        for (size_t j = 0; j < m; ++j) {
            max_change = std::max(max_change, std::abs(refined_coeffs[j] - coeffs[j]));
        }
        coeffs = refined_coeffs;

        // If coefficients stabilize, stop iterations
        if (max_change < tolerance) break;
    }

    // Convert coefficients to float and return
    return std::vector<float>(coeffs.begin(), coeffs.end());
}
