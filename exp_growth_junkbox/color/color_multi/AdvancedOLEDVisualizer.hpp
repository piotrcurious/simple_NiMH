#ifndef ADVANCED_OLED_VISUALIZER_H
#define ADVANCED_OLED_VISUALIZER_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

public:
    AdvancedOLEDVisualizer();
    void begin();
    void visualizeGrowthAnalysis(const std::vector<float>& xData, const std::vector<float>& yData,
                                 const std::vector<float>& coeffs, bool growthDetected, float growthRate);
    void displayErrorState(const String& errorMsg);
};

#endif
