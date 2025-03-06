std::vector<double> AdvancedPolynomialFitter::convertOrthogonalToMonomial(
    const std::vector<double>& orthogonal_coeffs, int degree) {
    
    size_t n = std::min(orthogonal_coeffs.size(), static_cast<size_t>(degree + 1));
    if (n == 0) return {};  // Handle empty input case

    std::vector<double> monomial_coeffs(n, 0.0);

    // Transformation matrix for Chebyshev to monomial basis
    std::vector<std::vector<double>> chebyshev_to_monomial(n, std::vector<double>(n, 0.0));

    // T_0(x) = 1
    chebyshev_to_monomial[0][0] = 1.0;

    // T_1(x) = x
    if (n > 1) {
        chebyshev_to_monomial[1][1] = 1.0;
    }

    // Generate higher-order Chebyshev polynomials using recurrence relation
    for (size_t i = 2; i < n; ++i) {
        for (size_t j = 0; j < n - 1; ++j) {  // Limit to prevent overflow
            chebyshev_to_monomial[i][j + 1] += 2.0 * chebyshev_to_monomial[i - 1][j];  // 2x*T_{n-1}
        }
        for (size_t j = 0; j < n; ++j) {
            chebyshev_to_monomial[i][j] -= chebyshev_to_monomial[i - 2][j];  // -T_{n-2}
        }
    }

    // Convert coefficients
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            monomial_coeffs[j] += orthogonal_coeffs[i] * chebyshev_to_monomial[i][j];
        }
    }

    return monomial_coeffs;
}
