#include <algorithm> // For std::transform

float calculateMSE(const std::vector<double>& coeffs, 
                   const std::vector<double>& x, 
                   const std::vector<double>& y) {
    // Normalization
    std::vector<double> x_norm = x;
    std::vector<double> y_norm = y;
    double x_max = *std::max_element(x.begin(), x.end());
    double y_max = *std::max_element(y.begin(), y.end());
    std::transform(x.begin(), x.end(), x_norm.begin(), [x_max](double val) { return val / x_max; });
    std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });

    double meanSquaredError = 0.0;
    double mean = 0.0;
    double M2 = 0.0;

    for (size_t i = 0; i < x_norm.size(); ++i) {
        double prediction = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            prediction += coeffs[j] * pow(x_norm[i], j);
        }
        double error = prediction - y_norm[i];
        double squaredError = error * error;
        double delta = squaredError - mean;
        mean += delta / (i + 1);
        M2 += delta * (squaredError - mean);
    }

    meanSquaredError = mean;
    return meanSquaredError;
}
