// Evaluate the fitted polynomial at a given point
float AdvancedPolynomialFitter::evaluateBernstein(const std::vector<float>& coeffs, float x, float x_min, float x_max) {
    // Normalize x to the range [0, 1]
    float x_norm = (x - x_min) / (x_max - x_min);

    int degree = coeffs.size() - 1;
    float result = 0.0f;

    // Evaluate the polynomial using the Bernstein basis
    for (int j = 0; j <= degree; ++j) {
        float basis = binomialCoefficient(degree, j) * 
                      std::pow(x_norm, j) * 
                      std::pow(1 - x_norm, degree - j);
        result += coeffs[j] * basis;
    }

    return result;
}

// Evaluate the fitted polynomial over a range of points
std::vector<float> AdvancedPolynomialFitter::evaluateBernsteinRange(const std::vector<float>& coeffs, const std::vector<float>& x_range, float x_min, float x_max) {
    std::vector<float> results(x_range.size());
    for (size_t i = 0; i < x_range.size(); ++i) {
        results[i] = evaluateBernstein(coeffs, x_range[i], x_min, x_max);
    }
    return results;
}
