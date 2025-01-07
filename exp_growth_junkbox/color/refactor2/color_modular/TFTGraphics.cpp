#ifdef USE_ESPI
#include "TFTGraphics.h"

TFTGraphics::TFTGraphics()
    : tft() {}

void TFTGraphics::begin() {
    tft.init();
    tft.setRotation(1); // Landscape mode
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text on black background
    tft.setTextSize(1);
    Serial.println("TFT_eSPI initialized");
}

void TFTGraphics::clear() {
    tft.fillScreen(TFT_BLACK);
}

void TFTGraphics::drawPixel(int x, int y, int color) {
    tft.drawPixel(x, y, color ? TFT_WHITE : TFT_BLACK);
}

void TFTGraphics::drawText(int x, int y, const std::string& text) {
    tft.setCursor(x, y);
    tft.print(text.c_str());
}

void TFTGraphics::display() {
    // No-op for TFT_eSPI as updates are immediate
}

void TFTGraphics::visualizeGrowthAnalysis(
    const std::vector<float>& xData,
    const std::vector<float>& yData,
    const std::vector<float>& coeffs,
    bool growthDetected,
    float growthRate
) {
    clear();

    // Display header
    drawText(10, 10, "Growth Analysis");
    drawText(10, 30, growthDetected ? "Growth: Yes" : "Growth: No");
    drawText(10, 50, "Rate: " + std::to_string(growthRate) + "x");

    // Define plot area
    int plotX = 20;
    int plotY = 70;
    int plotWidth = tft.width() - 40;
    int plotHeight = tft.height() - 90;

    // Draw plot boundaries
    tft.drawRect(plotX, plotY, plotWidth, plotHeight, TFT_WHITE);

    float minX = *std::min_element(xData.begin(), xData.end());
    float maxX = *std::max_element(xData.begin(), xData.end());
    float minY = *std::min_element(yData.begin(), yData.end());
    float maxY = *std::max_element(yData.begin(), yData.end());

    // Plot data points
    for (size_t i = 0; i < xData.size(); ++i) {
        int px = mapFloat(xData[i], minX, maxX, plotX, plotX + plotWidth);
        int py = mapFloat(yData[i], minY, maxY, plotY + plotHeight, plotY);
        drawPixel(px, py, 1);
    }

    // Plot the fitted curve if coefficients are available
    if (!coeffs.empty()) {
        for (int x = 0; x < plotWidth; ++x) {
            float realX = mapFloat(x, 0, plotWidth, minX, maxX);
            float realY = 0;
            for (size_t i = 0; i < coeffs.size(); ++i) {
                realY += coeffs[i] * pow(realX, i);
            }
            int py = mapFloat(realY, minY, maxY, plotY + plotHeight, plotY);
            if (py >= plotY && py <= plotY + plotHeight) {
                drawPixel(x + plotX, py, 1);
            }
        }
    }
}

void TFTGraphics::displayErrorState(const std::string& errorMsg) {
    clear();
    drawText(10, 10, "ERROR:");
    drawText(10, 30, errorMsg);
}

float TFTGraphics::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif // #ifdef USE_ESPI
