#include <vector>
#include <numeric>

std::vector<double> AdvancedPolynomialFitter::convertOrthogonalToMonomial(
    const std::vector<double>& orthogonal_coeffs, int degree) {

    size_t n = orthogonal_coeffs.size();
    if (degree >= 0 && static_cast<size_t>(degree) < n) {
        n = degree + 1; // Adjust n based on degree if it's smaller
    }

    std::vector<std::vector<double>> chebyshev_monomial_coeffs(n);

    // Initialize base cases for T_0(x) = 1 and T_1(x) = x
    chebyshev_monomial_coeffs[0] = {1.0};        // T_0(x) = 1
    if (n > 1) chebyshev_monomial_coeffs[1] = {0.0, 1.0}; // T_1(x) = x

    // Generate monomial coefficients for Chebyshev polynomials up to degree n-1
    for (size_t i = 2; i < n; ++i) {
        chebyshev_monomial_coeffs[i].resize(i + 1, 0.0); // Initialize with 0s up to degree i
        const std::vector<double>& prev_coeffs_1 = chebyshev_monomial_coeffs[i - 1];
        const std::vector<double>& prev_coeffs_2 = chebyshev_monomial_coeffs[i - 2];

        // T_i(x) = 2x*T_{i-1}(x) - T_{i-2}(x)

        // Contribution from 2x*T_{i-1}(x)
        for (size_t j = 0; j < prev_coeffs_1.size(); ++j) {
            if (j + 1 < chebyshev_monomial_coeffs[i].size()) {
                chebyshev_monomial_coeffs[i][j + 1] += 2.0 * prev_coeffs_1[j];
            }
        }
        // Contribution from -T_{i-2}(x)
        for (size_t j = 0; j < prev_coeffs_2.size(); ++j) {
            chebyshev_monomial_coeffs[i][j] -= prev_coeffs_2[j];
        }
    }

    std::vector<double> monomial_coeffs(n, 0.0);
    // Accumulate monomial coefficients
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < chebyshev_monomial_coeffs[i].size(); ++j) {
            monomial_coeffs[j] += orthogonal_coeffs[i] * chebyshev_monomial_coeffs[i][j];
        }
    }

    return monomial_coeffs;
}
