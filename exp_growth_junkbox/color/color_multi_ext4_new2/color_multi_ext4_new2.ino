#include "config.h"
#include <vector>
#include <numeric>
#include <cmath>
#include "ExponentialGrowthDetector.hpp"

#ifdef USE_OLED
#include "AdvancedOLEDVisualizer.hpp"
AdvancedOLEDVisualizer oledViz;
#endif //#ifdef USE_OLED

#ifdef USE_ESPI
#include "AdvancedTFTVisualizer.hpp"
AdvancedTFTVisualizer tftViz;
#endif //#ifdef USE_ESPI

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
     // Initialize growth detector and OLED
    growthDetector.begin();

#ifdef USE_OLED
    oledViz.begin();
#endif //#ifdef USE_OLED

#ifdef USE_ESPI
    tftViz.begin();
#endif //#ifdef USE_ESPI

      
}
float exp_step = 0 ; 
float value = 0 ;
void loop() {
    // Simulate exponential-like data collection
    static float time = 0;
    
    // Simulated exponential growth with noise
   // float value = 1 * sin(0.2*time)*exp(0.1 *time)) + random(-80, 80) / 10.0;

    float time_delta = random(0,+1000)/1000.0;
    float ride = 1*sin(0.05*time) ;

    if (ride >0) {exp_step += time_delta; 
    } else { exp_step = 0 ;}
    
    value = 60* sin(0.12*time)+6*exp(exp_step*0.06)+ random(-80, 80) / 100.0;
    time += time_delta;
    growthDetector.addDataPoint(time, value);
    // Detect growth and visualize
    growthDetector.detectExponentialGrowth();

            float backwardTimeWindow = GROWTH_EST_BACKWARD_TIME_WINDOW;  // Example: 5 units
            float forwardTimeWindow = GROWTH_EST_FORWARD_TIME_WINDOW;    // Example: 2 units
            float growthRate = growthDetector.computeGrowthRate(growthDetector.newCoeffs, growthDetector.timestamps, backwardTimeWindow, forwardTimeWindow);
               growthDetector.bestGrowthRate = growthRate;
                // Sophisticated growth detection criteria
            if (growthRate > 0.5 && growthRate < 10.0) {
                growthDetector.growthDetected = true;
               if (growthRate > growthDetector.bestGrowthRate) {
                    growthDetector.bestGrowthRate = growthRate;
//                    bestCoeffs = coeffs;
                }
            } else {
              growthDetector.growthDetected = false;  
            }


#ifdef USE_OLED
    oledViz.visualizeGrowthAnalysis(growthDetector.timestamps, growthDetector.values, growthDetector.bestCoeffs, growthDetector.growthDetected, growthDetector.bestGrowthRate);
#endif //#ifdef USE_OLED

#ifdef USE_ESPI
    tftViz.visualizeGrowthAnalysis(growthDetector.timestamps, growthDetector.values, growthDetector.bestCoeffs, growthDetector.growthDetected, growthDetector.bestGrowthRate);
#endif //#ifdef USE_ESPI
    
    //delay(500);
}
