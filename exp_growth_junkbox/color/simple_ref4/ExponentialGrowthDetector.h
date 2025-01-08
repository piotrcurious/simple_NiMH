#ifndef EXPONENTIAL_GROWTH_DETECTOR_H
#define EXPONENTIAL_GROWTH_DETECTOR_H

#include <Arduino.h>
#include <vector>
#include "AdvancedPolynomialFitter.h"
#include "AdvancedTFTVisualizer.h"

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    AdvancedTFTVisualizer tftViz;
    std::vector<float> timestamps;
    std::vector<float> values;

public:
    void begin();
    void addDataPoint(float timestamp, float value);
    bool detectExponentialGrowth();
    float computeGrowthRate(const std::vector<float>& coeffs, const std::vector<float>& xData,
                            float backwardTimeWindow, float forwardTimeWindow);
};

#endif
