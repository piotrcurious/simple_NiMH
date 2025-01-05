#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>  // Include CircularBuffer library for safer data storage

// Configuration constants
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr size_t MAX_DATA_POINTS = 30;  // Reduced further to prevent stack overflow
constexpr size_t MAX_POLYNOMIAL_DEGREE = 3;  // Reduced max degree
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 10.0f;

// Static array for polynomial coefficients to avoid heap fragmentation
struct PolynomialFit {
    float coefficients[MAX_POLYNOMIAL_DEGREE + 1];
    uint8_t degree;
    float growthRate;
    float error;
};

class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 50;  // Reduced iterations
    static constexpr float LEARNING_RATE = 0.01f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-5f;
    static constexpr float L2_LAMBDA = 0.001f;

    // Evaluate polynomial using Horner's method for better stability
    float evaluatePolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float calculateError(const float* coeffs, 
                        uint8_t degree,
                        const float* x,
                        const float* y,
                        size_t n) const {
        float totalError = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            float prediction = evaluatePolynomial(coeffs, degree, x[i]);
            float diff = prediction - y[i];
            totalError += diff * diff;
        }
        return totalError / n;
    }

public:
    PolynomialFit fitPolynomial(const float* x,
                               const float* y,
                               size_t n,
                               uint8_t degree) {
        PolynomialFit result;
        result.degree = degree;
        
        // Initialize coefficients to 0
        for (uint8_t i = 0; i <= degree; ++i) {
            result.coefficients[i] = 0.0f;
        }

        float learningRate = LEARNING_RATE;
        float prevError = calculateError(result.coefficients, degree, x, y, n);
        
        // Gradient descent with early stopping
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};
            
            // Calculate gradients
            for (size_t i = 0; i < n; ++i) {
                float prediction = evaluatePolynomial(result.coefficients, degree, x[i]);
                float error = prediction - y[i];
                float xPow = 1.0f;
                
                for (uint8_t j = 0; j <= degree; ++j) {
                    gradients[j] += 2.0f * error * xPow / n;
                    xPow *= x[i];
                }
            }

            // Update coefficients
            for (uint8_t j = 0; j <= degree; ++j) {
                gradients[j] += 2.0f * L2_LAMBDA * result.coefficients[j];
                result.coefficients[j] -= learningRate * gradients[j];
            }

            float currentError = calculateError(result.coefficients, degree, x, y, n);
            if (currentError > prevError) {
                learningRate *= 0.5f;
            }

            if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                break;
            }
            prevError = currentError;
        }

        result.error = prevError;
        
        // Calculate growth rate
        if (n > 0) {
            float lastX = x[n-1];
            float lastValue = evaluatePolynomial(result.coefficients, degree, lastX);
            float nextValue = evaluatePolynomial(result.coefficients, degree, lastX + 1.0f);
            result.growthRate = (lastValue != 0.0f) ? (nextValue - lastValue) / lastValue : 0.0f;
        } else {
            result.growthRate = 0.0f;
        }

        return result;
    }
};

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        if (in_max == in_min) return out_min;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    bool begin() {
        Wire.begin();
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            return false;
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        return true;
    }

    void visualizeGrowthAnalysis(const float* xData,
                                const float* yData,
                                size_t dataSize,
                                const PolynomialFit& fit) {
        if (dataSize < 2) {
            displayErrorState("Insufficient data");
            return;
        }

        display.clearDisplay();

        // Find min/max values
        float xMin = xData[0], xMax = xData[0];
        float yMin = yData[0], yMax = yData[0];
        for (size_t i = 1; i < dataSize; ++i) {
            xMin = min(xMin, xData[i]);
            xMax = max(xMax, xData[i]);
            yMin = min(yMin, yData[i]);
            yMax = max(yMax, yData[i]);
        }

        // Add margin to y-axis
        float yMargin = (yMax - yMin) * 0.1f;
        yMin -= yMargin;
        yMax += yMargin;

        const int plotHeight = SCREEN_HEIGHT - 24;
        
        // Draw data points
        for (size_t i = 0; i < dataSize; ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, plotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Draw fitted curve - use less points to save memory
        constexpr int CURVE_POINTS = 32;
        int16_t lastX = -1, lastY = -1;
        
        for (int i = 0; i < CURVE_POINTS; ++i) {
            float x = mapFloat(i, 0, CURVE_POINTS-1, xMin, xMax);
            float y = fit.coefficients[fit.degree];
            
            for (int j = fit.degree - 1; j >= 0; --j) {
                y = y * x + fit.coefficients[j];
            }

            int16_t screenX = mapFloat(x, xMin, xMax, 0, SCREEN_WIDTH-1);
            int16_t screenY = mapFloat(y, yMin, yMax, plotHeight-1, 0);
            
            if (lastX >= 0 && screenX - lastX < 5) {  // Only draw if points are close
                display.drawLine(lastX, lastY, screenX, screenY, SSD1306_WHITE);
            }
            
            lastX = screenX;
            lastY = screenY;
        }

        // Display statistics
        display.setCursor(0, plotHeight + 2);
        display.print(F("Growth:"));
        display.print(fit.growthRate >= MIN_GROWTH_RATE ? F("Yes") : F("No"));
        display.print(F(" R:"));
        display.print(fit.growthRate, 1);

        display.setCursor(0, plotHeight + 12);
        display.print(F("N:"));
        display.print(dataSize);

        display.display();
    }

    void displayErrorState(const char* errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("ERROR:"));
        display.println(errorMsg);
        display.display();
    }
};

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer display;
    
    // Use static arrays instead of vectors
    float timestamps[MAX_DATA_POINTS];
    float values[MAX_DATA_POINTS];
    size_t dataCount = 0;

public:
    bool begin() {
        return display.begin();
    }

    void addDataPoint(float timestamp, float value) {
        if (dataCount >= MAX_DATA_POINTS) {
            // Shift data left
            for (size_t i = 1; i < MAX_DATA_POINTS; ++i) {
                timestamps[i-1] = timestamps[i];
                values[i-1] = values[i];
            }
            dataCount = MAX_DATA_POINTS - 1;
        }
        
        timestamps[dataCount] = timestamp;
        values[dataCount] = value;
        dataCount++;
    }

    bool detectExponentialGrowth() {
        if (dataCount < 10) {
            display.displayErrorState("Need more data");
            return false;
        }

        PolynomialFit bestFit;
        bestFit.error = INFINITY;
        bestFit.growthRate = 0;
        
        // Try different polynomial degrees
        for (uint8_t degree = 2; degree <= MAX_POLYNOMIAL_DEGREE; ++degree) {
            PolynomialFit currentFit = fitter.fitPolynomial(
                timestamps, values, dataCount, degree
            );
            
            if (currentFit.error < bestFit.error && 
                currentFit.growthRate >= MIN_GROWTH_RATE && 
                currentFit.growthRate <= MAX_GROWTH_RATE) {
                bestFit = currentFit;
            }
        }

        display.visualizeGrowthAnalysis(timestamps, values, dataCount, bestFit);
        return bestFit.growthRate >= MIN_GROWTH_RATE;
    }

    void reset() {
        dataCount = 0;
    }
};

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    if (!growthDetector.begin()) {
        Serial.println(F("Display init failed"));
        while (1) delay(100);
    }
}

void loop() {
    static float time = 0.0f;
    
    // Generate sample data with exponential growth and noise
    float value = 10.0f * exp(0.2f * time) + random(-80, 81) / 10.0f;
    
    growthDetector.addDataPoint(time, value);
    growthDetector.detectExponentialGrowth();
    
    time += random(0, 11) / 10.0f;
    delay(500);
}
