#include "AdvancedOLEDVisualizer.hpp"
#include "config.h"

AdvancedOLEDVisualizer::AdvancedOLEDVisualizer()
    : display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

// private
    // Improved mapping with float precision
    float AdvancedOLEDVisualizer::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Compute polynomial derivative
//    std::vector<float> AdvancedOLEDVisualizer::computeDerivative(const std::vector<float>& coeffs) {
//        std::vector<float> derivative(coeffs.size() - 1, 0.0);
//        for (size_t i = 1; i < coeffs.size(); ++i) {
//            derivative.push_back(i * coeffs[i]);
//        }
//        return derivative;
//    }

//public

   void AdvancedOLEDVisualizer::begin() {
        //Wire.begin();
        if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            for(;;);
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
    }

    void AdvancedOLEDVisualizer::visualizeGrowthAnalysis(
        const std::vector<float>& xData, 
        const std::vector<float>& yData, 
        const std::vector<float>& coeffs,
        bool growthDetected,
        float growthRate
    ) {
        display.clearDisplay();

        // Find data ranges
        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        // Compute derivative
        //std::vector<float> derivative = computeDerivative(coeffs);

        // Split screen into three regions
        int dataPlotHeight = 40;  // Main data plot
        int bottomInfoHeight = 24;  // Bottom info area

        // Plot original data points
        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, OLED_SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Plot fitted polynomial curve
        for (int x = 0; x < OLED_SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, OLED_SCREEN_WIDTH-1, xMin, xMax);
            
            // Evaluate polynomial
            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX-xMin, j);
            }
            
            // Map fitted y to screen
            int y = mapFloat(yFitted, yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Bottom info area
        //display.drawFastHLine(0, dataPlotHeight, SCREEN_WIDTH, SSD1306_WHITE);
        
        // Compact info display
        display.setCursor(0, dataPlotHeight + 2);
        display.print(growthDetected ? "GROWTH:" : "NO GROWTH");
        display.print(" R:");
        display.print(growthRate, 2);
        
        // Degree and visualization indicator
        display.setCursor(0, dataPlotHeight + 12);
        display.print("Deg:");
        display.print(coeffs.size()-1);
        display.print(" Pts:");
        display.print(xData.size());
        display.display();
    }

    void AdvancedOLEDVisualizer::displayErrorState(const String& errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("ERROR:");
        display.println(errorMsg);
        display.display();
    }
