#include <Arduino.h>
#include "GraphicsInterface.h"

// Select the display backend
#define USE_OLED
//#define USE_ADATFT
//#define USE_ESPI

#ifdef USE_OLED
#include "OLEDGraphics.h"
#endif 

#ifdef USE_ESPI
#include "TFTGraphics.h"
#endif


#include "PolynomialFitter.h"

#define MAX_DATASET_WINDOW 100
#define GROWTH_EST_BACKWARD_TIME_WINDOW 5
#define GROWTH_EST_FORWARD_TIME_WINDOW 2

class ExponentialGrowthDetector {
private:
    PolynomialFitter polynomialFitter;
    std::vector<float> timestamps;
    std::vector<float> values;
    GraphicsInterface* graphics;

public:
    ExponentialGrowthDetector(GraphicsInterface* gfx) : graphics(gfx) {}
    //ExponentialGrowthDetector()  {}
    void begin() { graphics->begin(); }

    void addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);

        // Limit dataset size
        if (timestamps.size() > MAX_DATASET_WINDOW) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
    }

float computeGrowthRate(const std::vector<float>& coeffs, 
                        const std::vector<float>& xData, 
                        float backwardTimeWindow, 
                        float forwardTimeWindow) {
    if (xData.size() < 2 || coeffs.empty()) return 0.0;

    // Determine the time range for the backward window
    float latestTime = xData.back();
    float startTime = latestTime - backwardTimeWindow;

    // Find indices within the backward window
    auto startIt = std::lower_bound(xData.begin(), xData.end(), startTime);
    size_t startIndex = std::distance(xData.begin(), startIt);

    if (startIndex >= xData.size() - 1) return 0.0;

    // Calculate average growth rate over the backward time window
    float firstValue = 0.0, lastValue = 0.0;
    for (size_t j = 0; j < coeffs.size(); ++j) {
        firstValue += coeffs[j] * pow(xData[startIndex], j);
        lastValue += coeffs[j] * pow(xData.back(), j);
    }

    if (firstValue <= 0) return 0.0;  // Avoid division by zero or invalid growth rate

    float growthRate = (lastValue - firstValue) / firstValue;

    // Predict future value for the forward time window
    float futureTime = latestTime + forwardTimeWindow;
    float predictedValue = 0.0;
    for (size_t j = 0; j < coeffs.size(); ++j) {
        predictedValue += coeffs[j] * pow(futureTime, j);
    }

    Serial.print("Growth Rate: ");
    Serial.println(growthRate, 4);
    Serial.print("Predicted Value (");
    Serial.print(forwardTimeWindow);
    Serial.print(" units forward): ");
    Serial.println(predictedValue, 4);

    return growthRate;
}

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) return false;

        // Try different polynomial degrees and optimization methods
//        std::vector<int> degrees = {3, 4, 5, 6, 7};
        //std::vector<uint8_t> degrees = {1,2,3,4,5,6,7};
        std::vector<uint8_t> degrees = {16};

        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> newCoeffs;
        std::vector<float> lastCoeffs;
        std::vector<float> bestCoeffs;
        
        std::vector<PolynomialFitter::OptimizationMethod> methods = {
            PolynomialFitter::GRADIENT_DESCENT,
            //AdvancedPolynomialFitter::LEVENBERG_MARQUARDT,
            //AdvancedPolynomialFitter::NELDER_MEAD,
        };


       // Normalize timestamps
        std::vector<float> timestamps_norm = timestamps;
        double timestamps_min = *std::min_element(timestamps.begin(), timestamps.end());
   //     Serial.print("timestamps:");
   //     Serial.print(timestamps_min);
        double timestamps_max = *std::max_element(timestamps.begin(), timestamps.end());
   //     Serial.print(" timestamps:");
   //     Serial.println(timestamps_max);

       // double y_max = *std::max_element(y.begin(), y.end());
        std::transform(timestamps.begin(), timestamps.end(), timestamps_norm.begin(), [timestamps_min](double val) { return val - timestamps_min; });
        //std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });
//        Serial.println(timestamps_norm[0]); 
//        Serial.println(timestamps_norm[timestamps_norm.size()-1]);

        for (uint8_t degree : degrees) {
            for (auto method : methods) {
//                std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                  newCoeffs = polynomialFitter.fitPolynomial(
                    timestamps, values, degree, method
                );
                bestCoeffs = newCoeffs; // ensure there is some fit in coeffs sent to visualizer

                // Compute growth rate over backward time window and predict future
            float backwardTimeWindow = GROWTH_EST_BACKWARD_TIME_WINDOW;  // Example: 5 units
            float forwardTimeWindow = GROWTH_EST_FORWARD_TIME_WINDOW;    // Example: 2 units
            float growthRate = computeGrowthRate(newCoeffs, timestamps_norm, backwardTimeWindow, forwardTimeWindow);
               bestGrowthRate = growthRate;
                // Sophisticated growth detection criteria
            if (growthRate > 0.2 && growthRate < 10.0) {
                growthDetected = true;
               if (growthRate > bestGrowthRate) {
                    bestGrowthRate = growthRate;
//                    bestCoeffs = coeffs;
                }
            } else {
              growthDetected = false;  
            }
               // choose polynomial with best fit. 
                

                    
                // Potential growth detection logic
               // if (prediction > values.back() * 1.5 && errorBound < 0.1) {
                    //return true;
                //}
            }
            
            // choose polynomial with best fit. 
            
            
        }
//         Visualize results
        graphics->visualizeGrowthAnalysis(
            //timestamps, values, bestCoeffs, 
            timestamps, values, bestCoeffs, 
            growthDetected, bestGrowthRate
        );
        return growthDetected;
    }

};

// Instantiate the appropriate graphics backend
#ifdef USE_OLED
OLEDGraphics graphicsBackend;
#endif 

#ifdef USE_ESPI
TFTGraphics graphicsBackend;
#endif

ExponentialGrowthDetector growthDetector(&graphicsBackend);
//ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    growthDetector.begin();
}

void loop() {
    static float time = 0;
    float value = 40 * sin(0.1 * time) + random(-80, 80) / 100.0;
    time += random(0, 1000) / 1000.0;
    growthDetector.addDataPoint(time, value);
    growthDetector.detectExponentialGrowth();
    delay(200);
}
