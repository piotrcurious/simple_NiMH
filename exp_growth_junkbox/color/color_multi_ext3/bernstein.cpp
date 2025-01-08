std::vector<float> AdvancedPolynomialFitter::fitWithBernstein(const std::vector<float>& x, const std::vector<float>& y, int degree) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {}; // Invalid input
    }

    // Normalize x for numerical stability
    std::vector<float> x_norm(x.size());
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    std::transform(x.begin(), x.end(), x_norm.begin(), [x_min, x_max](float val) {
        return (val - x_min) / (x_max - x_min);
    });

    size_t n = x_norm.size();
    size_t m = degree + 1;

    // Construct Bernstein basis matrix
    std::vector<std::vector<double>> B(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            B[i][j] = binomialCoefficient(degree, j) * 
                      std::pow(x_norm[i], j) * 
                      std::pow(1 - x_norm[i], degree - j);
        }
    }

    // Solve for coefficients using least squares
    std::vector<std::vector<double>> BTB(m, std::vector<double>(m, 0.0));
    std::vector<double> BTy(m, 0.0);

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            BTy[j] += B[i][j] * y[i];
            for (size_t k = 0; k < m; ++k) {
                BTB[j][k] += B[i][j] * B[i][k];
            }
        }
    }

    // Solve the system BTB * coeffs = BTy
    std::vector<double> coeffs = solveLinearSystem(BTB, BTy);

    // Convert coefficients to float
    return std::vector<float>(coeffs.begin(), coeffs.end());
}

// Utility: Binomial coefficient
double AdvancedPolynomialFitter::binomialCoefficient(int n, int k) {
    if (k > n) return 0;
    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1) / double(i);
    }
    return result;
}

// Utility: Solve linear system
std::vector<double> AdvancedPolynomialFitter::solveLinearSystem(const std::vector<std::vector<double>>& A, const std::vector<double>& b) {
    // Implement Gaussian elimination or LU decomposition
    // (Code omitted for brevity)
}
