std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(
    const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method) {

    const size_t n = x.size();
    const size_t m = degree + 1;

    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);
    std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            xPowers[i][j] = xPowers[i][j - 1] * x[i];
        }
    }

    // Compute initial A^T * A and A^T * y
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += xPowers[i][j] * y[i];
            for (size_t k = 0; k <= j; ++k) {
                ATA[j][k] += xPowers[i][j] * xPowers[i][k];
            }
        }
    }

    // Fill symmetric part of the matrix
    for (size_t j = 0; j < m; ++j) {
        for (size_t k = j + 1; k < m; ++k) {
            ATA[j][k] = ATA[k][j];
        }
    }

    std::vector<double> coeffs = solveQR(ATA, ATy);
    
    // **Step 2: Compute Residuals & Construct Alternative Equation Set**
    std::vector<double> residuals(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        double fitted_value = 0.0;
        for (size_t j = 0; j < m; ++j) {
            fitted_value += coeffs[j] * xPowers[i][j];
        }
        residuals[i] = y[i] - fitted_value; // Compute residual at each point
    }

    // Modify equation set using weighted residuals
    std::vector<std::vector<double>> ATA_alt(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy_alt(m, 0.0);

    for (size_t i = 0; i < n; ++i) {
        double weight = std::exp(-std::abs(residuals[i])); // Exponential weighting to suppress oscillations
        for (size_t j = 0; j < m; ++j) {
            ATy_alt[j] += weight * xPowers[i][j] * y[i];
            for (size_t k = 0; k <= j; ++k) {
                ATA_alt[j][k] += weight * xPowers[i][j] * xPowers[i][k];
            }
        }
    }

    // Fill symmetric part of modified matrix
    for (size_t j = 0; j < m; ++j) {
        for (size_t k = j + 1; k < m; ++k) {
            ATA_alt[j][k] = ATA_alt[k][j];
        }
    }

    // **Step 3: Solve the alternative equation set using QR decomposition**
    std::vector<double> refined_coeffs = solveQR(ATA_alt, ATy_alt);

    // Convert coefficients to float and return
    std::vector<float> result(refined_coeffs.begin(), refined_coeffs.end());
    return result;
}
