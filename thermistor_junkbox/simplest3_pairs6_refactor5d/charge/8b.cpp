int estimateSaturationDutyCycle() {
    if (internalResistanceDataPairs.size() < 2) {
        Serial.println("Not enough data to estimate saturation duty cycle (pairs).");
        return MAX_CHARGE_DUTY_CYCLE; // Default to max if not enough data
    }
    if (internalResistanceData.size() < 2) {
        Serial.println("Not enough data to estimate saturation duty cycle (loaded/unloaded).");
        return MAX_CHARGE_DUTY_CYCLE; // Default to max if not enough data
    }

    float resistanceThresholdMultiplier = 1.5f; // Default value

    // Estimate resistance threshold multiplier based on loaded/unloaded data
    if (internalResistanceData.size() >= 2) {
        float resistanceLowCurrent = -1.0f;
        float resistanceHighCurrent = -1.0f;

        // Resistance at approximately lowest current (last element as it's sorted)
        if (!internalResistanceData.empty()) {
            resistanceLowCurrent = internalResistanceData.back().resistance;
        }

        // Resistance at approximately highest current (first element)
        if (!internalResistanceData.empty()) {
            resistanceHighCurrent = internalResistanceData.front().resistance;
        }

        if (resistanceHighCurrent > 0.001f && resistanceLowCurrent > 0.001f) {
            float resistanceRatio = resistanceLowCurrent / resistanceHighCurrent;
            Serial.printf("Loaded/Unloaded Resistance Ratio (Low Curr/High Curr): %.3f\n", resistanceRatio);
            // Adjust multiplier based on the ratio (example mapping)
            if (resistanceRatio > 2.0f) {
                resistanceThresholdMultiplier = 2.0f;
            } else if (resistanceRatio > 1.5f) {
                resistanceThresholdMultiplier = 1.8f;
            } else {
                resistanceThresholdMultiplier = 1.6f;
            }
            Serial.printf("Estimated Resistance Threshold Multiplier: %.3f\n", resistanceThresholdMultiplier);
        } else {
            Serial.println("Could not reliably estimate resistance threshold multiplier.");
        }
    }

    int potentialSaturationDutyCycle = MAX_CHARGE_DUTY_CYCLE;

    for (size_t i = 1; i < internalResistanceDataPairs.size(); ++i) {
        float currentPair = internalResistanceDataPairs[i].current;
        float resistancePair = internalResistanceDataPairs[i].resistance;
        int dutyCyclePair = internalResistanceDataPairs[i].dutyCycle;

        // Find a corresponding loaded/unloaded data point (approximate current match)
        for (const auto& luData : internalResistanceData) {
            if (std::abs(luData.current - currentPair) < 0.02f) { // Tolerance for current matching
                if (resistancePair > luData.resistance * resistanceThresholdMultiplier && resistancePair > 0.01f && luData.resistance > 0.005f) {
                    potentialSaturationDutyCycle = dutyCyclePair;
                    Serial.printf("Potential saturation detected at Duty Cycle: %d (Pair Resistance: %.3f, LU Resistance: %.3f, Multiplier: %.3f)\n", dutyCyclePair, resistancePair, luData.resistance, resistanceThresholdMultiplier);
                    return potentialSaturationDutyCycle; // Return the first potential saturation point
                }
                break; // Move to the next pair data point
            }
        }
    }

    Serial.printf("Saturation duty cycle not explicitly detected, using MAX_CHARGE_DUTY_CYCLE as fallback (%d).\n", MAX_CHARGE_DUTY_CYCLE);
    return MAX_CHARGE_DUTY_CYCLE; // Fallback if no clear saturation point is found
}
