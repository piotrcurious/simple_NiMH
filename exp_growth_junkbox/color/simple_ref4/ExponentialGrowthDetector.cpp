#include "ExponentialGrowthDetector.h"

void ExponentialGrowthDetector::begin() {
    tftViz.begin();
}

void ExponentialGrowthDetector::addDataPoint(float timestamp, float value) {
    // Implementation as provided
}

bool ExponentialGrowthDetector::detectExponentialGrowth() {
    // Implementation as provided
}

float ExponentialGrowthDetector::computeGrowthRate(const std::vector<float>& coeffs, const std::vector<float>& xData,
                                                   float backwardTimeWindow, float forwardTimeWindow) {
    // Implementation as provided
}
