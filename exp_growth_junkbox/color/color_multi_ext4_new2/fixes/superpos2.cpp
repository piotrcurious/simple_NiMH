std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(
    const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method ) 
{
    const size_t n = x.size();
    const size_t m = degree + 1;

    // Precompute powers of x for efficiency (monomial basis)
    std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            xPowers[i][j] = xPowers[i][j - 1] * x[i];
        }
    }

    // Build normal equations for the global fit: ATA * coeffs = ATy
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
    // Fill in the symmetric part
    for (size_t j = 0; j < m; ++j) {
        for (size_t k = j + 1; k < m; ++k) {
            ATA[j][k] = ATA[k][j];
        }
    }

    // Solve the initial system with QR decomposition
    std::vector<double> coeffs = solveQR(ATA, ATy);

    // ---- Begin iterative superposition correction ----
    const int maxIterations = 10;  // Maximum number of superposition iterations
    const double tol = 1e-6;       // Tolerance for convergence
    double damping = 0.5;          // Damping factor to control the correction magnitude

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Compute the residuals: r[i] = y[i] - p(x[i])
        std::vector<double> residuals(n, 0.0);
        double residualNorm = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double p_val = 0.0;
            for (size_t j = 0; j < m; ++j) {
                p_val += coeffs[j] * xPowers[i][j];
            }
            residuals[i] = y[i] - p_val;
            residualNorm += residuals[i] * residuals[i];
        }
        residualNorm = std::sqrt(residualNorm);
        if (residualNorm < tol) break; // Converged

        // Build a new normal equation system for the residual polynomial.
        // This system will provide the correction coefficients.
        std::vector<std::vector<double>> ATA_res(m, std::vector<double>(m, 0.0));
        std::vector<double> ATres(m, 0.0);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATres[j] += xPowers[i][j] * residuals[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA_res[j][k] += xPowers[i][j] * xPowers[i][k];
                }
            }
        }
        // Fill symmetric matrix for ATA_res
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA_res[j][k] = ATA_res[k][j];
            }
        }

        // Solve for the correction polynomial using QR decomposition
        std::vector<double> correction = solveQR(ATA_res, ATres);

        // Update the coefficients with a damping factor
        double maxUpdate = 0.0;
        for (size_t j = 0; j < m; ++j) {
            double update = damping * correction[j];
            coeffs[j] += update;
            maxUpdate = std::max(maxUpdate, std::abs(update));
        }

        // If the maximum update is small, we consider the process converged
        if (maxUpdate < tol) break;
    }
    // ---- End iterative superposition correction ----

    // Convert the coefficients from double to float before returning
    std::vector<float> result(coeffs.begin(), coeffs.end());
    return result;
}
