#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <TFT_eSPI.h>

#include <vector>
#include <numeric>
#include <cmath>

#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
//#define OLED_RESET -1
#define OLED_SCREEN_ADDRESS 0x3C

  constexpr uint8_t OLED_DC = 16 ;   
//#define OLED_DC     16
  constexpr uint8_t OLED_CS = 5 ; 
//#define OLED_CS     5
  constexpr uint8_t OLED_RESET = 17;
//#define OLED_RESET  17
  constexpr uint32_t OLED_BITRATE = 80000000 ;

#define TFT_SCREEN_WIDTH 320
#define TFT_SCREEN_HEIGHT 240

#define MAX_DATASET_WINDOW 100 
#define GROWTH_EST_BACKWARD_TIME_WINDOW 5.0 // seconds
#define GROWTH_EST_FORWARD_TIME_WINDOW  2.0 // seconds



class AdvancedPolynomialFitter {
public:
    // Optimization methods enum
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
    };

    // calculate using Welford's method
    double calculateMSE(const std::vector<float>& coeffs, 
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

    meanSquaredError = mean;
    //Serial.println(meanSquaredError);
    return meanSquaredError;
}


    // Fit a polynomial to the data using the normal equation
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method = GRADIENT_DESCENT) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

       // Normalize x and y
        std::vector<float> x_norm = x;
        double x_min = *std::min_element(x.begin(), x.end());
        Serial.print("xmin:");
        Serial.print(x_min);
        double x_max = *std::max_element(x.begin(), x.end());
        Serial.print(" xmax:");
        Serial.println(x_max);

       // double y_max = *std::max_element(y.begin(), y.end());
        std::transform(x.begin(), x.end(), x_norm.begin(), [x_min](double val) { return val - x_min; });
        //std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });
        Serial.println(x_norm[0]); 
        Serial.println(x_norm[x_norm.size()-1]);

        size_t n = x_norm.size();
        size_t m = degree + 1;

        // Construct the Vandermonde matrix
        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A[i][j] = xi;
                xi *= x_norm[i];
            }
        }

        // Construct the normal equation: (A^T * A) * coeffs = A^T * y
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += A[i][j] * y[i];
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += A[i][j] * A[i][k];
                }
            }
        }

        // Solve the normal equation using Gaussian elimination
        std::vector<double> coeffs = solveLinearSystem(ATA, ATy);

        // Convert coefficients to float
        std::vector<float> result(coeffs.begin(), coeffs.end());
        return result;
    }

private:
    // Solve a linear system using Gaussian elimination
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b) {
        size_t n = A.size();

        // Forward elimination
        for (size_t k = 0; k < n; ++k) {
            // Pivot for numerical stability
            for (size_t i = k + 1; i < n; ++i) {
                if (fabs(A[i][k]) > fabs(A[k][k])) {
                    std::swap(A[k], A[i]);
                    std::swap(b[k], b[i]);
                }
            }

            for (size_t i = k + 1; i < n; ++i) {
                double factor = A[i][k] / A[k][k];
                for (size_t j = k; j < n; ++j) {
                    A[i][j] -= factor * A[k][j];
                }
                b[i] -= factor * b[k];
            }
        }

        // Back substitution
        std::vector<double> x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }
};
//------------------- graphics

class AdvancedTFTVisualizer {
//  #include <TFT_eSPI.h>
private:
    TFT_eSPI display;

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedTFTVisualizer() : display(TFT_eSPI()) {}

    void begin() {
        display.init();
        display.setRotation(1);  // Landscape mode
        display.fillScreen(TFT_BLACK);
    }

    void visualizeGrowthAnalysis(const std::vector<float>& xData,
                                 const std::vector<float>& yData,
                                 const std::vector<float>& coeffs,
                                 bool growthDetected,
                                 float growthRate) {
        display.fillScreen(TFT_BLACK);

        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, TFT_SCREEN_WIDTH - 1);
            int y = mapFloat(yData[i], yMin, yMax, TFT_SCREEN_HEIGHT - 40, 0);
            display.drawPixel(x, y, TFT_WHITE);
        }

        for (int x = 0; x < TFT_SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, TFT_SCREEN_WIDTH - 1, xMin, xMax);

            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX - xMin, j);
            }

            int y = mapFloat(yFitted, yMin, yMax, TFT_SCREEN_HEIGHT - 40, 0);
            display.drawPixel(x, y, TFT_GREEN);
        }

        display.setCursor(10, TFT_SCREEN_HEIGHT - 30);
        display.setTextColor(TFT_WHITE);
        display.printf("Growth: %s", growthDetected ? "YES" : "NO");
        display.setCursor(10, TFT_SCREEN_HEIGHT - 20);
        display.printf("Rate: %.2f", growthRate);
        //display.display();
    }
};


//----OLED

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;

    // Improved mapping with float precision
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Compute polynomial derivative
    std::vector<float> computeDerivative(const std::vector<float>& coeffs) {
        std::vector<float> derivative(coeffs.size() - 1, 0.0);
        for (size_t i = 1; i < coeffs.size(); ++i) {
            derivative.push_back(i * coeffs[i]);
        }
        return derivative;
    }

public:
//    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}
      AdvancedOLEDVisualizer() : display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    void begin() {
        Wire.begin();
        if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_SCREEN_ADDRESS)) {
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
        //std::vector<float> derivative = computeDerivative(coeffs);

        // Split screen into three regions
        int dataPlotHeight = 40;  // Main data plot
        int bottomInfoHeight = 24;  // Bottom info area

        // Plot original data points
        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, OLED_SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Plot fitted polynomial curve
        for (int x = 0; x < OLED_SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, OLED_SCREEN_WIDTH-1, xMin, xMax);
            
            // Evaluate polynomial
            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX-xMin, j);
            }
            
            // Map fitted y to screen
            int y = mapFloat(yFitted, yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Bottom info area
        //display.drawFastHLine(0, dataPlotHeight, SCREEN_WIDTH, SSD1306_WHITE);
        
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

// Example usage in ESP32 context
class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter polynomialFitter;
    std::vector<float> timestamps;
    std::vector<float> values;
//    AdvancedOLEDVisualizer oledViz;
    AdvancedTFTVisualizer  tftViz;   // so two can be used at the same time 

public:
    void begin() {
//        oledViz.begin();
        tftViz.begin();

    }

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
        std::vector<uint8_t> degrees = {9};

        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> newCoeffs;
        std::vector<float> lastCoeffs;
        std::vector<float> bestCoeffs;
        
        std::vector<AdvancedPolynomialFitter::OptimizationMethod> methods = {
            AdvancedPolynomialFitter::GRADIENT_DESCENT,
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
        // Visualize results
        //oledViz.visualizeGrowthAnalysis(timestamps, values, bestCoeffs, growthDetected, bestGrowthRate);
        tftViz.visualizeGrowthAnalysis(timestamps, values, bestCoeffs, growthDetected, bestGrowthRate);

        return growthDetected;
    }
};

   ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    // Initialize growth detector and OLED
    growthDetector.begin();
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
    
    delay(200);
}
