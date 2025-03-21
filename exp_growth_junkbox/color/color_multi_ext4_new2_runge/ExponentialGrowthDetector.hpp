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
double normalizeTime(double t, double tMax);

public:
        float predictedValue = 0.0;
        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> newCoeffs;
        std::vector<float> lastCoeffs;
        std::vector<float> bestCoeffs;

        
    std::vector<float> timestamps;
    std::vector<float> timestamps_norm;
    std::vector<float> values;
    void begin();
    void addDataPoint(float timestamp, float value);
    void detectExponentialGrowth();
    float computeGrowthRate(const std::vector<float>& coeffs, const std::vector<float>& xData,
                            float backwardTimeWindow, float forwardTimeWindow);
};

#endif
