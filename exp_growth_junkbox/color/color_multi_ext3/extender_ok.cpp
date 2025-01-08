// Efficient polynomial evaluation using Horner's method
float ExponentialGrowthDetector::evaluatePolynomial(const std::vector<float>& coeffs, float x) {
    if (coeffs.empty()) return 0.0f;
    
    float result = coeffs.back();  // Start with highest degree coefficient
    
    // Traverse coefficients from highest to lowest degree
    for (int i = coeffs.size() - 2; i >= 0; --i) {
        result = result * x + coeffs[i];
    }
    
    return result;
}

void ExponentialGrowthDetector::extendNormalizedDataset(
    std::vector<float>& norm_timestamps,
    std::vector<float>& norm_values,
    float backwardTimeWindow,
    float forwardTimeWindow,
    size_t numPredictions,
    uint8_t polyDegree = 3) {
    
    if (norm_timestamps.empty() || norm_timestamps.size() != norm_values.size()) {
        return;
    }

    // Create temporary datasets for the backward window
    std::vector<float> window_timestamps;
    std::vector<float> window_values;
    window_timestamps.reserve(norm_timestamps.size());  // Optimize memory allocation
    window_values.reserve(norm_timestamps.size());
    
    // Get the cutoff time for backward window
    float lastTime = norm_timestamps.back();
    float startTime = lastTime - backwardTimeWindow;
    
    // Fill temporary datasets with points within backward window
    for (size_t i = 0; i < norm_timestamps.size(); ++i) {
        if (norm_timestamps[i] >= startTime) {
            window_timestamps.push_back(norm_timestamps[i]);
            window_values.push_back(norm_values[i]);
        }
    }
    
    // Ensure we have enough points for the requested polynomial degree
    if (window_timestamps.size() <= polyDegree) {
        #ifdef DEBUG_MODE
            Serial.println("Warning: Not enough points for requested polynomial degree");
        #endif
        polyDegree = window_timestamps.size() - 1;
    }
    
    // Fit polynomial to the windowed data
    std::vector<float> windowCoeffs = polynomialFitter.fitPolynomial(
        window_timestamps,
        window_values,
        polyDegree,
        AdvancedPolynomialFitter::LEVENBERG_MARQUARDT
    );
    
    // Pre-allocate space for new points
    norm_timestamps.reserve(norm_timestamps.size() + numPredictions);
    norm_values.reserve(norm_values.size() + numPredictions);
    
    // Generate predictions
    float timeStep = forwardTimeWindow / static_cast<float>(numPredictions);
    
    // Extend the normalized datasets with predictions
    for (size_t i = 1; i <= numPredictions; ++i) {
        float futureTime = lastTime + (i * timeStep);
        float predictedValue = evaluatePolynomial(windowCoeffs, futureTime);
        
        // Add predictions to the normalized datasets
        norm_timestamps.push_back(futureTime);
        norm_values.push_back(predictedValue);
    }
    
    #ifdef DEBUG_MODE
        Serial.print("Polynomial degree used: ");
        Serial.println(polyDegree);
        Serial.print("Window points used for fit: ");
        Serial.println(window_timestamps.size());
        Serial.print("Predictions added: ");
        Serial.println(numPredictions);
    #endif
}
