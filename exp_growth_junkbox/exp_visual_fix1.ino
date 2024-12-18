#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    // Improved mapping with float precision
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Compute polynomial derivative
    std::vector<float> computeDerivative(const std::vector<float>& coeffs) {
        std::vector<float> derivative;
        for (size_t i = 1; i < coeffs.size(); ++i) {
            derivative.push_back(i * coeffs[i]);
        }
        return derivative;
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

    void begin() {
        Wire.begin();
        if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            for(;;);
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
    }

    void visualizeGrowthAnalysis(
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
        std::vector<float> derivative = computeDerivative(coeffs);

        // Split screen into three regions
        int dataPlotHeight = 40;  // Main data plot
        int bottomInfoHeight = 24;  // Bottom info area

        // Plot original data points
        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Plot fitted polynomial curve
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, SCREEN_WIDTH-1, xMin, xMax);
            
            // Evaluate polynomial
            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX, j);
            }
            
            // Map fitted y to screen
            int y = mapFloat(yFitted, yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_INVERSE);
        }

        // Plot derivative in bottom region
        float derivMin = *std::min_element(derivative.begin(), derivative.end());
        float derivMax = *std::max_element(derivative.begin(), derivative.end());
        
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            // Map screen x to derivative index
            size_t derivIndex = map(x, 0, SCREEN_WIDTH-1, 0, derivative.size()-1);
            
            // Evaluate derivative
            float derivValue = derivative[derivIndex];
            
            // Map derivative to bottom region
            int y = mapFloat(derivValue, derivMin, derivMax, 
                             SCREEN_HEIGHT-1, dataPlotHeight);
            
            // Highlight growth regions
            if (growthDetected && derivValue > 0) {
                display.drawPixel(x, y, SSD1306_WHITE);
            } else {
                display.drawPixel(x, y, SSD1306_INVERSE);
            }
        }

        // Bottom info area
        display.drawFastHLine(0, dataPlotHeight, SCREEN_WIDTH, SSD1306_WHITE);
        
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

    void displayErrorState(const String& errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("ERROR:");
        display.println(errorMsg);
        display.display();
    }
};

class AdvancedGrowthDetector {
private:
    std::vector<float> timestamps;
    std::vector<float> values;
    AdvancedOLEDVisualizer oledViz;

    float computeGrowthRate(const std::vector<float>& coeffs, 
                             const std::vector<float>& xData) {
        if (xData.empty()) return 0.0;

        float lastX = xData.back();
        float lastValue = 0.0;
        float nextValue = 0.0;

        // Compute last and next values
        for (size_t j = 0; j < coeffs.size(); ++j) {
            lastValue += coeffs[j] * pow(lastX, j);
            nextValue += coeffs[j] * pow(lastX + 1, j);
        }

        // Compute growth rate
        return (nextValue - lastValue) / lastValue;
    }

    std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        // Simple polynomial fitting (could be replaced with more advanced method)
        std::vector<float> coeffs(degree + 1, 0.0);
        
        // Simplified least squares approximation
        for (size_t j = 0; j <= degree; ++j) {
            float sum = 0.0;
            for (size_t i = 0; i < x.size(); ++i) {
                float yPred = 0.0;
                for (size_t k = 0; k <= degree; ++k) {
                    yPred += (j == k ? 1 : 0) * pow(x[i], k);
                }
                sum += y[i] * pow(x[i], j);
            }
            coeffs[j] = sum;
        }

        return coeffs;
    }

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
            oledViz.displayErrorState("Insufficient Data");
            return false;
        }

        // Try different polynomial degrees
        std::vector<int> degrees = {3, 4, 5, 6, 7};
        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> bestCoeffs;

        for (int degree : degrees) {
            std::vector<float> coeffs = fitPolynomial(
                timestamps, values, degree
            );

            // Compute growth rate
            float growthRate = computeGrowthRate(coeffs, timestamps);

            // Sophisticated growth detection criteria
            if (growthRate > 0.2 && growthRate < 5.0) {
                growthDetected = true;
                if (growthRate > bestGrowthRate) {
                    bestGrowthRate = growthRate;
                    bestCoeffs = coeffs;
                }
            }
        }

        // Visualize results
        oledViz.visualizeGrowthAnalysis(
            timestamps, values, bestCoeffs, 
            growthDetected, bestGrowthRate
        );

        return growthDetected;
    }
};

// Pin Definitions (adjust for your specific ESP32 board)
#define SCL_PIN 22
#define SDA_PIN 21

AdvancedGrowthDetector growthDetector;

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
