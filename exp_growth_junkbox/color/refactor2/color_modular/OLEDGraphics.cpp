#ifdef USE_OLED
#include "OLEDGraphics.h"

OLEDGraphics::OLEDGraphics()
    : display(128, 64, &SPI, -1, -1, -1) {} // Update pins if needed

void OLEDGraphics::begin() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        while (true); // Halt if initialization fails
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    Serial.println("OLED initialized");
}

void OLEDGraphics::clear() {
    display.clearDisplay();
}

void OLEDGraphics::drawPixel(int x, int y, int color) {
    display.drawPixel(x, y, color ? SSD1306_WHITE : SSD1306_BLACK);
}

void OLEDGraphics::drawText(int x, int y, const std::string& text) {
    display.setCursor(x, y);
    display.print(text.c_str());
}

void OLEDGraphics::display() {
    display.display();
}

void OLEDGraphics::visualizeGrowthAnalysis(
    const std::vector<float>& xData,
    const std::vector<float>& yData,
    const std::vector<float>& coeffs,
    bool growthDetected,
    float growthRate
) {
    clear();

    // Display header
    drawText(0, 0, "Growth Analysis");
    drawText(0, 10, growthDetected ? "Growth: Yes" : "Growth: No");
    drawText(0, 20, "Rate: " + std::to_string(growthRate) + "x");

    // Map data to the display area
    int plotWidth = display.width();
    int plotHeight = display.height() - 30; // Leave space for labels
    int plotX = 0;
    int plotY = 30;

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
            float realX = mapFloat(x, plotX, plotX + plotWidth, minX, maxX);
            float realY = 0;
            for (size_t i = 0; i < coeffs.size(); ++i) {
                realY += coeffs[i] * pow(realX, i);
            }
            int py = mapFloat(realY, minY, maxY, plotY + plotHeight, plotY);
            if (py >= plotY && py < plotY + plotHeight) {
                drawPixel(x, py, 1);
            }
        }
    }

    display();
}

void OLEDGraphics::displayErrorState(const String& errorMsg) {
    clear();
    drawText(0, 0, "ERROR:");
    drawText(0, 10, errorMsg);
    display();
}

float OLEDGraphics::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif //#ifdef USE_OLED
