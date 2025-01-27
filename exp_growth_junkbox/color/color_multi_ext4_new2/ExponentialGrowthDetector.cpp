#include "ExponentialGrowthDetector.hpp"
#include "config.h"
#include <vector>
#include <numeric>
#include <cmath>

void ExponentialGrowthDetector::begin() {
  //        oledViz.begin(); // TODO : ifdef logic
//    tftViz.begin();
}

void ExponentialGrowthDetector::addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);

        // Limit dataset size
        if (timestamps.size() > MAX_DATASET_WINDOW) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
}

void ExponentialGrowthDetector::detectExponentialGrowth() {
        if (timestamps.size() < MIN_DATASET_WINDOW) {
          // no data yet       
          return;}

        // Try different polynomial degrees and optimization methods
//        std::vector<int> degrees = {3, 4, 5, 6, 7};
        //std::vector<uint8_t> degrees = {1,2,3,4,5,6,7};
        //std::vector<uint8_t> degrees = {5};

//        float bestGrowthRate = 0.0;
//        bool growthDetected = false;
//        std::vector<float> newCoeffs;
//        std::vector<float> lastCoeffs;
//        std::vector<float> bestCoeffs;
        
        std::vector<AdvancedPolynomialFitter::OptimizationMethod> methods = {
            //AdvancedPolynomialFitter::GRADIENT_DESCENT,    // TODO: implemented , in another file
            //AdvancedPolynomialFitter::LEVENBERG_MARQUARDT,   // implemented, works
            //AdvancedPolynomialFitter::NELDER_MEAD,         // not implemented
            AdvancedPolynomialFitter::NONE,                // default
            
        };

       // Normalize timestamps
//        std::vector<float> timestamps_norm = timestamps;
        timestamps_norm = timestamps;
        double timestamps_min = *std::min_element(timestamps.begin(), timestamps.end());
   //     Serial.print("timestamps:");
   //     Serial.print(timestamps_min);
        double timestamps_max = *std::max_element(timestamps.begin(), timestamps.end());
   //     Serial.print(" timestamps:");
   //     Serial.println(timestamps_max);

       // double y_max = *std::max_element(y.begin(), y.end());
        std::transform(timestamps.begin(), timestamps.end(), timestamps_norm.begin(), [timestamps_min,timestamps_max](double val) { return (val - timestamps_min)/(timestamps_max-timestamps_min); });
        //std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });
//        Serial.println(timestamps_norm[0]); 
//        Serial.println(timestamps_norm[timestamps_norm.size()-1]);

        uint8_t degree = 9; // polynomial degree

        if (true || timestamps.size() >= MIN_DATASET_WINDOW && timestamps.size() < MAX_DATASET_WINDOW/2 ) {
//        if ( timestamps.size() >= MIN_DATASET_WINDOW && timestamps.size() < MAX_DATASET_WINDOW/2 ) {

          // enough data to perform initial fit
          // after enough data formed initial fit , switch to optimization.       
        //for (uint8_t degree : degrees) {
            for (auto method : methods) {
//                std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                  newCoeffs = polynomialFitter.fitPolynomial(timestamps_norm, values, degree, method);
                bestCoeffs = newCoeffs; // ensure there is some fit in coeffs sent to visualizer                    
            }
            // choose polynomial with best fit.                         
        //}
      
        } else {
          // enough data to incrementally update fit
          newCoeffs = polynomialFitter.levenbergMarquardt(bestCoeffs, timestamps_norm, values, degree);
        }

        
//            float backwardTimeWindow = GROWTH_EST_BACKWARD_TIME_WINDOW;  // Example: 5 units
//            float forwardTimeWindow = GROWTH_EST_FORWARD_TIME_WINDOW;    // Example: 2 units
//            float growthRate = computeGrowthRate(newCoeffs, timestamps_norm, backwardTimeWindow, forwardTimeWindow);
//               bestGrowthRate = growthRate;
//                // Sophisticated growth detection criteria
//            if (growthRate > 0.2 && growthRate < 10.0) {
//                growthDetected = true;
//               if (growthRate > bestGrowthRate) {
//                    bestGrowthRate = growthRate;
//                    bestCoeffs = coeffs;
//                }
//            } else {
//              growthDetected = false;  
//            }


        return; 
    }

double ExponentialGrowthDetector::normalizeTime(double t, double tMax) {
    return t / tMax;
}

float ExponentialGrowthDetector::computeGrowthRate(const std::vector<float>& coeffs, const std::vector<float>& xData,
                                                   float backwardTimeWindow, float forwardTimeWindow) {
    if (xData.size() < 2 || coeffs.empty()) return 0.0;

    // Determine the time range for the backward window
    float latestTime = xData.back();
    float startTime = latestTime - backwardTimeWindow;

    float xMax = *std::max_element(xData.begin(), xData.end());
    float xMin = *std::min_element(xData.begin(), xData.end());
    // Find indices within the backward window
    auto startIt = std::lower_bound(xData.begin(), xData.end(), startTime);
    size_t startIndex = std::distance(xData.begin(), startIt);

    if (startIndex >= xData.size() - 1) return 0.0;

    // Calculate average growth rate over the backward time window
    float firstValue = 0.0, lastValue = 0.0;
    for (size_t j = 0; j < coeffs.size(); ++j) {
        firstValue += coeffs[j] * pow(normalizeTime(xData[startIndex]-xMin,xMax-xMin), j);
        lastValue += coeffs[j] * pow(normalizeTime(xData.back()-xMin,xMax-xMin), j);
    }

    if (firstValue <= 0) return 0.0;  // Avoid division by zero or invalid growth rate

    float growthRate = (lastValue - firstValue) / firstValue;

    // Predict future value for the forward time window
    float futureTime = latestTime + forwardTimeWindow;
    predictedValue = 0.0;
    for (size_t j = 0; j < coeffs.size(); ++j) {
        predictedValue += coeffs[j] * pow(normalizeTime(futureTime-xMin,xMax-xMin), j);
    }
    Serial.print("tStart:");
    Serial.print(normalizeTime(xData[startIndex]-xMin,xMax-xMin));
    Serial.print(" tEnd:");
    Serial.print(normalizeTime(xData.back()-xMin,xMax-xMin));
    
    Serial.print(" Growth Rate: ");
    Serial.println(growthRate, 4);
    Serial.print("Predicted Value (");
    Serial.print(forwardTimeWindow);
    Serial.print(" units forward): ");
    Serial.println(predictedValue, 4);

    return growthRate;
                                                    
}
