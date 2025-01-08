// constants.h
#pragma once

// Screen definitions
#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
#define OLED_SCREEN_ADDRESS 0x3C

constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;

#define TFT_SCREEN_WIDTH 320
#define TFT_SCREEN_HEIGHT 240

// Analysis parameters
#define MAX_DATASET_WINDOW 100 
#define GROWTH_EST_BACKWARD_TIME_WINDOW 5.0 // seconds
#define GROWTH_EST_FORWARD_TIME_WINDOW  2.0 // seconds

// polynomial_fitter.h
#pragma once
#include <vector>
#include <numeric>
#include <cmath>

class AdvancedPolynomialFitter {
public:
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
    };

    double calculateMSE(const std::vector<float>& coeffs, 
                       const std::vector<float>& x, 
                       const std::vector<float>& y);
    
    std::vector<float> fitPolynomial(const std::vector<float>& x, 
                                    const std::vector<float>& y, 
                                    int degree,
                                    OptimizationMethod method = GRADIENT_DESCENT);

private:
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A, 
                                         std::vector<double>& b);
};

// polynomial_fitter.cpp
#include "polynomial_fitter.h"

double AdvancedPolynomialFitter::calculateMSE(const std::vector<float>& coeffs, 
                                             const std::vector<float>& x, 
                                             const std::vector<float>& y) {
    double meanSquaredError = 0.0;
    double mean = 0.0;
    double M2 = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        float prediction = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            prediction += coeffs[j] * pow(x[i], j);
        }
        float error = prediction - y[i];
        double squaredError = error * error;
        double delta = squaredError - mean;
        mean += delta / (i + 1);
        M2 += delta * (squaredError - mean);
    }

    return mean;
}

// ... Rest of AdvancedPolynomialFitter implementation ...

// tft_visualizer.h
#pragma once
#include <TFT_eSPI.h>
#include <vector>
#include "constants.h"

class AdvancedTFTVisualizer {
private:
    TFT_eSPI display;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

public:
    AdvancedTFTVisualizer();
    void begin();
    void visualizeGrowthAnalysis(const std::vector<float>& xData,
                                const std::vector<float>& yData,
                                const std::vector<float>& coeffs,
                                bool growthDetected,
                                float growthRate);
};

// tft_visualizer.cpp
#include "tft_visualizer.h"

// ... Implementation of AdvancedTFTVisualizer methods ...

// oled_visualizer.h
#pragma once
#include <Adafruit_SSD1306.h>
#include <vector>
#include "constants.h"

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    std::vector<float> computeDerivative(const std::vector<float>& coeffs);

public:
    AdvancedOLEDVisualizer();
    void begin();
    void visualizeGrowthAnalysis(const std::vector<float>& xData,
                                const std::vector<float>& yData,
                                const std::vector<float>& coeffs,
                                bool growthDetected,
                                float growthRate);
    void displayErrorState(const String& errorMsg);
};

// oled_visualizer.cpp
#include "oled_visualizer.h"

// ... Implementation of AdvancedOLEDVisualizer methods ...

// growth_detector.h
#pragma once
#include <vector>
#include "polynomial_fitter.h"
#include "tft_visualizer.h"
#include "oled_visualizer.h"

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    std::vector<float> timestamps;
    std::vector<float> values;
    AdvancedTFTVisualizer tftViz;
    
    float computeGrowthRate(const std::vector<float>& coeffs,
                           const std::vector<float>& xData,
                           float backwardTimeWindow,
                           float forwardTimeWindow);

public:
    void begin();
    void addDataPoint(float timestamp, float value);
    bool detectExponentialGrowth();
};

// growth_detector.cpp
#include "growth_detector.h"

// ... Implementation of ExponentialGrowthDetector methods ...

// main.ino
#include "growth_detector.h"

ExponentialGrowthDetector growthDetector;
float exp_step = 0;
float value = 0;

void setup() {
    Serial.begin(115200);
    growthDetector.begin();
}

void loop() {
    static float time = 0;
    float time_delta = random(0, +1000)/1000.0;
    float ride = 1*sin(0.2*time);

    if (ride > 0) {
        exp_step += time_delta;
    } else {
        exp_step = 0;
    }
    
    value = 60*sin(0.1*time) + 10*exp(exp_step*0.2) + random(-80, 80)/100.0;
    time += time_delta;
    
    growthDetector.addDataPoint(time, value);
    growthDetector.detectExponentialGrowth();
    
    delay(200);
}
