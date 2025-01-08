#ifndef EXPONENTIAL_GROWTH_DETECTOR_H
#define EXPONENTIAL_GROWTH_DETECTOR_H

#include <Arduino.h>
#include "AdvancedPolynomialFitter.hpp"
#include <vector>
#include <numeric>
#include <cmath>
class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    //    AdvancedOLEDVisualizer oledViz; // TODO : define ifdef logic
//    AdvancedTFTVisualizer tftViz;
//    std::vector<float> timestamps;
//    std::vector<float> values;

public:

        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> newCoeffs;
        std::vector<float> lastCoeffs;
        std::vector<float> bestCoeffs;

        
    std::vector<float> timestamps;
    std::vector<float> values;
    void begin();
    void addDataPoint(float timestamp, float value);
    bool detectExponentialGrowth();
    float computeGrowthRate(const std::vector<float>& coeffs, const std::vector<float>& xData,
                            float backwardTimeWindow, float forwardTimeWindow);
};

#endif
