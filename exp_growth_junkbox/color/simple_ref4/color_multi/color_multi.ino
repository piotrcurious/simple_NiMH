#include "config.h"
#include <vector>
#include <numeric>
#include <cmath>
#include "ExponentialGrowthDetector.h"

ExponentialGrowthDetector growthDetector;
AdvancedTFTVisualizer tftViz;
void setup() {
    Serial.begin(115200);
     // Initialize growth detector and OLED
    growthDetector.begin();
      //        oledViz.begin(); // TODO : ifdef logic
    tftViz.begin();
}
float exp_step = 0 ; 
float value = 0 ;
void loop() {
    // Simulate exponential-like data collection
    static float time = 0;
    
    // Simulated exponential growth with noise
   // float value = 1 * sin(0.2*time)*exp(0.1 *time)) + random(-80, 80) / 10.0;

    float time_delta = random(0,+1000)/1000.0;
    float ride = 1*sin(0.2*time) ;

    if (ride >0) {exp_step += time_delta; 
    } else { exp_step = 0 ;}
    
    value = 60* sin(0.1*time)+10*exp(exp_step*0.2)+ random(-80, 80) / 100.0;
    time += time_delta;
    growthDetector.addDataPoint(time, value);
    // Detect growth and visualize
    growthDetector.detectExponentialGrowth();

      //oledViz.visualizeGrowthAnalysis(timestamps, values, bestCoeffs, growthDetected, bestGrowthRate); // TODO: ifdef logic
    
    tftViz.visualizeGrowthAnalysis(growthDetector.timestamps, growthDetector.values, growthDetector.bestCoeffs, growthDetector.growthDetected, growthDetector.bestGrowthRate);
    
    delay(200);
}
