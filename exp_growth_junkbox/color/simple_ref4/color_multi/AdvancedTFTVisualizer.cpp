#include "AdvancedTFTVisualizer.h"
#include "config.h"

AdvancedTFTVisualizer::AdvancedTFTVisualizer() : display(TFT_eSPI()) {}

//private :
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

//public :

void AdvancedTFTVisualizer::begin() {
        display.init();
        display.setRotation(1);  // Landscape mode
        display.fillScreen(TFT_BLACK);
    }

void AdvancedTFTVisualizer::visualizeGrowthAnalysis(const std::vector<float>& xData,
                                 const std::vector<float>& yData,
                                 const std::vector<float>& coeffs,
                                 bool growthDetected,
                                 float growthRate) {
        display.fillScreen(TFT_BLACK);

        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, TFT_SCREEN_WIDTH - 1);
            int y = mapFloat(yData[i], yMin, yMax, TFT_SCREEN_HEIGHT - 40, 0);
            display.drawPixel(x, y, TFT_WHITE);
        }

        for (int x = 0; x < TFT_SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, TFT_SCREEN_WIDTH - 1, xMin, xMax);

            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX - xMin, j);
            }

            int y = mapFloat(yFitted, yMin, yMax, TFT_SCREEN_HEIGHT - 40, 0);
            display.drawPixel(x, y, TFT_GREEN);
        }

        display.setCursor(10, TFT_SCREEN_HEIGHT - 30);
        display.setTextColor(TFT_WHITE);
        display.printf("Growth: %s", growthDetected ? "YES" : "NO");
        display.setCursor(10, TFT_SCREEN_HEIGHT - 20);
        display.printf("Rate: %.2f", growthRate);
        //display.display();
    }
