void ExponentialGrowthDetector::extendNormalizedDataset(
    const std::vector<float>& coeffs,
    float backwardTimeWindow,
    float forwardTimeWindow,
    size_t numPredictions,
    std::vector<float>& extended_timestamps,
    std::vector<float>& extended_values) {
    
    if (timestamps.empty() || coeffs.size() != 4) { // Ensure 3rd degree polynomial
        return;
    }

    // Initialize output vectors with existing normalized data
    extended_timestamps = timestamps_norm;
    extended_values = values;

    // Calculate the time step for predictions based on forward window
    float timeStep = forwardTimeWindow / static_cast<float>(numPredictions);
    float lastTimestamp = timestamps_norm.back();

    // Generate future timestamps and predict corresponding values
    for (size_t i = 1; i <= numPredictions; ++i) {
        float futureTime = lastTimestamp + (i * timeStep);
        
        // Calculate predicted value using 3rd degree polynomial
        float predictedValue = coeffs[0] + 
                             coeffs[1] * futureTime +
                             coeffs[2] * pow(futureTime, 2) +
                             coeffs[3] * pow(futureTime, 3);

        // Add new points to extended datasets
        extended_timestamps.push_back(futureTime);
        extended_values.push_back(predictedValue);
    }

    // Calculate fit error for the historical data
    float sumSquaredError = 0.0;
    size_t backwardPoints = 0;
    float backwardStartTime = lastTimestamp - backwardTimeWindow;

    for (size_t i = 0; i < timestamps_norm.size(); ++i) {
        if (timestamps_norm[i] >= backwardStartTime) {
            float fitted_value = coeffs[0] + 
                               coeffs[1] * timestamps_norm[i] +
                               coeffs[2] * pow(timestamps_norm[i], 2) +
                               coeffs[3] * pow(timestamps_norm[i], 3);
            
            sumSquaredError += pow(values[i] - fitted_value, 2);
            backwardPoints++;
        }
    }

    // Calculate RMSE (Root Mean Square Error)
    float rmse = (backwardPoints > 0) ? 
                 sqrt(sumSquaredError / static_cast<float>(backwardPoints)) : 0.0;

    // Store fit quality metrics as class members or print them
    fitQuality = {
        .rmse = rmse,
        .backwardPoints = backwardPoints,
        .predictedPoints = numPredictions
    };

    #ifdef DEBUG_MODE
        Serial.print("Polynomial Fit RMSE: ");
        Serial.println(rmse, 6);
        Serial.print("Backward window points: ");
        Serial.println(backwardPoints);
        Serial.print("Generated predictions: ");
        Serial.println(numPredictions);
    #endif
}
