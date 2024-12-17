#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class OLEDPolynomialVisualizer {
private:
    Adafruit_SSD1306 display;
    
    // Scaling and mapping helpers
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Color palette for visualization
    enum ColorScheme {
        MONOCHROME,
        GRADIENT,
        HEAT_MAP
    };

public:
    OLEDPolynomialVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

    void begin() {
        Wire.begin();
        if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            for(;;); // Don't proceed, loop forever
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
    }

    void visualizePolynomialFit(
        const std::vector<float>& xData, 
        const std::vector<float>& yData, 
        const std::vector<float>& coeffs,
        ColorScheme colorScheme = MONOCHROME
    ) {
        display.clearDisplay();

        // Find data ranges
        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        // Plot original data points
        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, SCREEN_HEIGHT-1, 0);
            
            // Color based on scheme
            switch (colorScheme) {
                case MONOCHROME:
                    display.drawPixel(x, y, SSD1306_WHITE);
                    break;
                case GRADIENT:
                    // Gradient coloring based on position
                    display.drawPixel(x, y, 
                        i % 2 ? SSD1306_WHITE : SSD1306_BLACK
                    );
                    break;
                case HEAT_MAP:
                    // Heat map coloring based on value
                    uint8_t intensity = map(yData[i], yMin, yMax, 0, 255);
                    display.drawPixel(x, y, 
                        intensity > 128 ? SSD1306_WHITE : SSD1306_BLACK
                    );
                    break;
            }
        }

        // Plot fitted polynomial curve
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            // Map screen x to data x
            float dataX = mapFloat(x, 0, SCREEN_WIDTH-1, xMin, xMax);
            
            // Evaluate polynomial
            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX, j);
            }
            
            // Map fitted y to screen
            int y = mapFloat(yFitted, yMin, yMax, SCREEN_HEIGHT-1, 0);
            
            // Draw fitted curve
            display.drawPixel(x, y, SSD1306_INVERSE);
        }

        // Display polynomial degree and key stats
        display.setCursor(0,0);
        display.print("Deg:");
        display.print(coeffs.size()-1);
        display.print(" R:");
        display.print(yMax-yMin);

        display.display();
    }

    void displayGrowthDetectionStatus(bool detected, float growthRate) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);

        if (detected) {
            display.println("GROWTH DETECTED!");
            display.print("Rate: ");
            display.print(growthRate, 2);
        } else {
            display.println("No Significant");
            display.println("Growth");
        }

        display.display();
    }
};

class AdvancedPolynomialFitter {
    // [Previous implementation remains the same]
    // ... (include all methods from previous artifact)
};

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    OLEDPolynomialVisualizer oledViz;
    
    std::vector<float> timestamps;
    std::vector<float> values;

public:
    void begin() {
        oledViz.begin();
    }

    void addDataPoint(float timestamp, float value) {
        timestamps.push_back(timestamp);
        values.push_back(value);

        // Limit dataset size
        if (timestamps.size() > 100) {
            timestamps.erase(timestamps.begin());
            values.erase(values.begin());
        }
    }

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) {
            oledViz.displayGrowthDetectionStatus(false, 0);
            return false;
        }

        // Try different polynomial degrees
        std::vector<int> degrees = {3, 4, 5, 6, 7};
        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> bestCoeffs;

        for (int degree : degrees) {
            std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                timestamps, values, degree
            );

            // Visualize current polynomial fit
            oledViz.visualizePolynomialFit(
                timestamps, values, coeffs, 
                OLEDPolynomialVisualizer::GRADIENT
            );

            // Compute growth characteristics
            float lastValue = values.back();
            float predictedNextValue = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                predictedNextValue += coeffs[j] * pow(timestamps.back() + 1, j);
            }

            float growthRate = (predictedNextValue - lastValue) / lastValue;

            // Growth detection criteria
            if (growthRate > 0.2 && growthRate < 5.0) {
                growthDetected = true;
                if (growthRate > bestGrowthRate) {
                    bestGrowthRate = growthRate;
                    bestCoeffs = coeffs;
                }
            }
        }

        // Display final growth status
        oledViz.displayGrowthDetectionStatus(growthDetected, bestGrowthRate);

        return growthDetected;
    }
};

// Pin Definitions (adjust for your specific ESP32 board)
#define SCL_PIN 22
#define SDA_PIN 21

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C with specific pins
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Initialize growth detector and OLED
    growthDetector.begin();
}

void loop() {
    // Simulate exponential-like data collection
    static float time = 0;
    
    // Simulated exponential growth with noise
    float value = 10 * exp(0.2 * time) + random(-50, 50) / 10.0;
    
    growthDetector.addDataPoint(time, value);
    
    // Detect growth and visualize
    growthDetector.detectExponentialGrowth();
    
    time += 0.5;
    delay(500);
}
