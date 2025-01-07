#ifndef TFT_GRAPHICS_H
#define TFT_GRAPHICS_H

#include "GraphicsInterface.h"
#include <TFT_eSPI.h>

class TFTGraphics : public GraphicsInterface {
private:
    TFT_eSPI display;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

public:
    TFTGraphics();
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

#endif // TFT_GRAPHICS_H
