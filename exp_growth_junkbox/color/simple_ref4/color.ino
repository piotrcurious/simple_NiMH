#include "ExponentialGrowthDetector.h"

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    growthDetector.begin();
}

void loop() {
    // Add logic to add data points and detect growth
}
