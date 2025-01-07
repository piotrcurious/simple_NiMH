#include <Arduino.h>
#include "GraphicsInterface.h"

// Select the display backend
#define USE_OLED

#ifdef USE_OLED
#include "graphics/OLEDGraphics.h"
#else
#include "graphics/TFTGraphics.h"
#endif

#include "utils/PolynomialFitter.h"

class ExponentialGrowthDetector {
private:
    PolynomialFitter polynomialFitter;
    std::vector<float> timestamps;
    std::vector<float> values;
    GraphicsInterface* graphics;

public:
    ExponentialGrowthDetector(GraphicsInterface* gfx) : graphics(gfx) {}
    void begin() { graphics->begin(); }
    void addDataPoint(float timestamp, float value);
    bool detectExponentialGrowth();
};

// Instantiate the appropriate graphics backend
#ifdef USE_OLED
OLEDGraphics graphicsBackend;
#else
TFTGraphics graphicsBackend;
#endif

ExponentialGrowthDetector growthDetector(&graphicsBackend);

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
