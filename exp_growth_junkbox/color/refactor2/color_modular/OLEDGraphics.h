#ifdef USE_OLED
#ifndef OLED_GRAPHICS_H
#define OLED_GRAPHICS_H

#include "GraphicsInterface.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class OLEDGraphics : public GraphicsInterface {
private:
    Adafruit_SSD1306 display;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

public:
    OLEDGraphics();
    void begin() override;
    void clear() override;
    void drawPixel(int x, int y, int color) override;
    void drawText(int x, int y, const std::string& text) override;
    void display() override;
    void visualizeGrowthAnalysis(
        const std::vector<float>& xData,
        const std::vector<float>& yData,
        const std::vector<float>& coeffs,
        bool growthDetected,
        float growthRate
    ) override;
    void displayErrorState(const std::string& errorMsg) override;
};

#endif // OLED_GRAPHICS_H
#endif // #ifdef USE_OLED
