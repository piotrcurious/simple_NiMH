#include "AdvancedTFTVisualizer.hpp"
#include "config.h"

AdvancedTFTVisualizer::AdvancedTFTVisualizer() : display(TFT_eSPI()) {}

//private :
float AdvancedTFTVisualizer::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
// Add normalization helper functions
double AdvancedTFTVisualizer::normalizeTime(double t, double tMax) {
    return t / tMax;
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
        float xMax = *std::max_element(xData.begin(), xData.end())+GROWTH_EST_FORWARD_TIME_WINDOW;
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        
        float margin_yMin=yMin-(yMax-yMin);
        float margin_yMax=yMax+(yMax-yMin);

// plot data
        for (size_t i = 0; i < xData.size(); ++i) {
            uint16_t x = mapFloat(xData[i], xMin, xMax, 0, TFT_SCREEN_WIDTH - 1);
//            uint16_t y = mapFloat(yData[i], yMin, yMax, TFT_SCREEN_HEIGHT - 1, 0);
            uint16_t y = mapFloat(yData[i], margin_yMin, margin_yMax, TFT_SCREEN_HEIGHT - 1, 0);

            display.drawPixel(x, y, TFT_WHITE);
        }
//plot prediction

        float futureX = mapFloat(xMax-GROWTH_EST_FORWARD_TIME_WINDOW,xMin,xMax,0,TFT_SCREEN_WIDTH-1);
        display.drawLine(futureX,0,futureX,TFT_SCREEN_HEIGHT,TFT_BLUE);
        float pastX = mapFloat(xMax-GROWTH_EST_BACKWARD_TIME_WINDOW-GROWTH_EST_FORWARD_TIME_WINDOW,xMin,xMax,0,TFT_SCREEN_WIDTH-1);
        display.drawLine(pastX,0,pastX,TFT_SCREEN_HEIGHT,TFT_RED);

        
        float y = mapFloat(yMin, margin_yMin, margin_yMax, TFT_SCREEN_HEIGHT - 1, 0);
        display.drawLine(pastX,y,futureX,y,TFT_DARKGREEN);
        y = mapFloat(yMax, margin_yMin, margin_yMax, TFT_SCREEN_HEIGHT - 1, 0);
        display.drawLine(pastX,y,futureX,y,TFT_DARKGREEN);

        uint16_t lastY = 0 ; 
        for (int x = 0; x < TFT_SCREEN_WIDTH; ++x) {
//            float dataX = mapFloat(x, 0, TFT_SCREEN_WIDTH - 1, xMin, xMax+GROWTH_EST_FORWARD_TIME_WINDOW);
            float dataX = mapFloat(x, 0, TFT_SCREEN_WIDTH - 1, xMin, xMax);

            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
#ifdef REVERSED_NORMALIZATION 
               yFitted += coeffs[j] * pow(normalizeTime(dataX - xMax),xMax), j);
#else
               yFitted += coeffs[j] * pow(normalizeTime((dataX-xMin),xMax-xMin-GROWTH_EST_FORWARD_TIME_WINDOW), j);
#endif // #ifdef REVERSED_NORMALIZATION 


            }
            uint16_t y = 0; 
            if (yFitted != NAN && lastY != NAN) {
//               y = mapFloat(yFitted, yMin, yMax, TFT_SCREEN_HEIGHT - 1, 0);
               y = mapFloat(yFitted, margin_yMin, margin_yMax, TFT_SCREEN_HEIGHT - 1, 0);
            if (dataX < xMax-GROWTH_EST_FORWARD_TIME_WINDOW) {
            display.drawPixel(x, y, TFT_GREEN);
            if (growthDetected){
            display.drawPixel(x, y, TFT_CYAN);             
            }
            
            }else {
            if (y>0 && y< TFT_SCREEN_HEIGHT) {
            display.drawPixel(x, y, TFT_YELLOW);
            display.drawLine(x-1,lastY,x,y,TFT_WHITE);
            }            
            }
            
            }
            lastY = y;

        }

        display.setCursor(10, TFT_SCREEN_HEIGHT - 30);
        display.setTextColor(TFT_WHITE);
        display.printf("Growth: %s", growthDetected ? "YES" : "NO");
        display.setCursor(10, TFT_SCREEN_HEIGHT - 20);
        display.printf("Rate: %.2f", growthRate);

              // Degree and visualization indicator
        display.setCursor(10, TFT_SCREEN_HEIGHT - 12);
        display.printf("Deg:");
        display.print(coeffs.size()-1);
        display.print(" Pts:");
        display.print(xData.size());
        
        //display.display();
    }
