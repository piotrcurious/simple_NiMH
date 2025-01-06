float calculateMSE(const std::vector<float>& coeffs, 
                   const std::vector<float>& x, 
                   const std::vector<float>& y) {
    float meanSquaredError = 0.0;
    float mean = 0.0;
    float M2 = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        float prediction = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            prediction += coeffs[j] * pow(x[i], j);
        }
        float error = prediction - y[i];
        float squaredError = error * error;
        float delta = squaredError - mean;
        mean += delta / (i + 1);
        M2 += delta * (squaredError - mean);
    }

    meanSquaredError = mean;
    return meanSquaredError;
}
