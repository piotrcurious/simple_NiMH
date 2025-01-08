void ExponentialGrowthDetector::extendNormalizedDataset(float backwardTimeWindow, float forwardTimeWindow, size_t predictionCount) {
    if (timestamps.size() < MIN_DATASET_WINDOW || values.size() < 3) return;

    // Normalize timestamps
    timestamps_norm = timestamps;
    double timestamps_min = *std::min_element(timestamps.begin(), timestamps.end());
    std::transform(timestamps.begin(), timestamps.end(), timestamps_norm.begin(), [timestamps_min](double val) { return val - timestamps_min; });

    // Fit a 3rd-degree polynomial to the data
    std::vector<float> coeffs = polynomialFitter.fitPolynomial(timestamps_norm, values, 3, AdvancedPolynomialFitter::LEVENBERG_MARQUARDT);

    // Determine the backward time range
    float latestTimeNorm = timestamps_norm.back();
    float startTimeNorm = latestTimeNorm - backwardTimeWindow;

    // Find indices within the backward window
    auto startIt = std::lower_bound(timestamps_norm.begin(), timestamps_norm.end(), startTimeNorm);
    size_t startIndex = std::distance(timestamps_norm.begin(), startIt);

    if (startIndex >= timestamps_norm.size() - 1) return;

    // Extend the dataset with predictions
    float timeStep = forwardTimeWindow / static_cast<float>(predictionCount);
    float currentTime = timestamps.back();

    for (size_t i = 0; i < predictionCount; ++i) {
        currentTime += timeStep;

        // Predict the value using the polynomial
        float currentTimeNorm = currentTime - timestamps_min;
        float predictedValue = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            predictedValue += coeffs[j] * pow(currentTimeNorm, j);
        }

        // Append the prediction to the dataset
        timestamps.push_back(currentTime);
        values.push_back(predictedValue);
    }

    // Ensure dataset size doesn't exceed maximum limit
    while (timestamps.size() > MAX_DATASET_WINDOW) {
        timestamps.erase(timestamps.begin());
        values.erase(values.begin());
    }
}
