float computeGrowthRate(const std::vector<float>& coeffs, 
                        const std::vector<float>& xData, 
                        float backwardTimeWindow, 
                        float forwardTimeWindow) {
    if (xData.size() < 2 || coeffs.empty()) return 0.0;

    // Determine the time range for the backward window
    float latestTime = xData.back();
    float startTime = latestTime - backwardTimeWindow;

    // Find indices within the backward window
    auto startIt = std::lower_bound(xData.begin(), xData.end(), startTime);
    size_t startIndex = std::distance(xData.begin(), startIt);

    if (startIndex >= xData.size() - 1) return 0.0;

    // Calculate average growth rate over the backward time window
    float firstValue = 0.0, lastValue = 0.0;
    for (size_t j = 0; j < coeffs.size(); ++j) {
        firstValue += coeffs[j] * pow(xData[startIndex], j);
        lastValue += coeffs[j] * pow(xData.back(), j);
    }

    if (firstValue <= 0) return 0.0;  // Avoid division by zero or invalid growth rate

    float growthRate = (lastValue - firstValue) / firstValue;

    // Predict future value for the forward time window
    float futureTime = latestTime + forwardTimeWindow;
    float predictedValue = 0.0;
    for (size_t j = 0; j < coeffs.size(); ++j) {
        predictedValue += coeffs[j] * pow(futureTime, j);
    }

    Serial.print("Growth Rate: ");
    Serial.println(growthRate, 4);
    Serial.print("Predicted Value (");
    Serial.print(forwardTimeWindow);
    Serial.print(" units forward): ");
    Serial.println(predictedValue, 4);

    return growthRate;
}
