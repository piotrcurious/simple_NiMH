bool detectExponentialGrowth() {
    if (timestamps.size() < 10) return false;

    std::vector<uint8_t> degrees = {4};
    float bestGrowthRate = 0.0;
    bool growthDetected = false;
    std::vector<float> bestCoeffs;
    std::vector<AdvancedPolynomialFitter::OptimizationMethod> methods = {
        AdvancedPolynomialFitter::GRADIENT_DESCENT,
    };

    for (uint8_t degree : degrees) {
        for (auto method : methods) {
            std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                timestamps, values, degree, method
            );
            bestCoeffs = coeffs;  // Ensure there is some fit in coeffs sent to visualizer

            // Compute growth rate over backward time window and predict future
            float backwardTimeWindow = 5.0;  // Example: 5 units
            float forwardTimeWindow = 2.0;   // Example: 2 units
            float growthRate = computeGrowthRate(coeffs, timestamps, backwardTimeWindow, forwardTimeWindow);

            // Sophisticated growth detection criteria
            if (growthRate > 0.2 && growthRate < 10.0) {
                growthDetected = true;
                if (growthRate > bestGrowthRate) {
                    bestGrowthRate = growthRate;
                    bestCoeffs = coeffs;
                }
            }
        }
    }

    // Visualize results
    oledViz.visualizeGrowthAnalysis(
        timestamps, values, bestCoeffs,
        growthDetected, bestGrowthRate
    );
    return growthDetected;
}
