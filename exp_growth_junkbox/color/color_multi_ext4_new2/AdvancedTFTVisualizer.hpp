#ifndef ADVANCED_TFT_VISUALIZER_H
#define ADVANCED_TFT_VISUALIZER_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <vector>

class AdvancedTFTVisualizer {
private:
    TFT_eSPI display;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    double normalizeTime(double t, double tMax);
public:
    AdvancedTFTVisualizer();
    void begin();
    void visualizeGrowthAnalysis(const std::vector<float>& xData, const std::vector<float>& yData,
                                 const std::vector<float>& coeffs, bool growthDetected, float growthRate);
};

#endif
