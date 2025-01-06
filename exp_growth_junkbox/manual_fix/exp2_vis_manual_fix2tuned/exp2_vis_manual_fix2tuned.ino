#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <cmath>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
//#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

  constexpr uint8_t OLED_DC = 16 ;   
//#define OLED_DC     16
  constexpr uint8_t OLED_CS = 5 ; 
//#define OLED_CS     5
  constexpr uint8_t OLED_RESET = 17;
//#define OLED_RESET  17
  constexpr uint32_t OLED_BITRATE = 80000000 ;

#define MAX_DATASET_WINDOW 50 

class AdvancedPolynomialFitter {
private:
    // Optimization configuration
    const int MAX_ITERATIONS = 500;
    const double LEARNING_RATE = 0.000000001;
    const float CONVERGENCE_THRESHOLD = 1e-6;
//    const float CONVERGENCE_THRESHOLD = 1e-4;
//    const float CONVERGENCE_THRESHOLD = 1e-3;

    // Regularization parameters
    const float L1_LAMBDA = 0.01;  // Lasso regularization
    const float L2_LAMBDA = 0.001;  // Ridge regularization

 

    // Regularization types
    enum RegularizationType {
        NONE,
        L1_LASSO,
        L2_RIDGE,
        ELASTIC_NET
    };

    // Loss function types
    enum LossFunctionType {
        MEAN_SQUARED_ERROR,
        MEAN_ABSOLUTE_ERROR,
        HUBER_LOSS
    };

/* prone to overflow
    // Helper method: Calculate Mean Squared Error
    float calculateMSE(const std::vector<float>& coeffs, 
                       const std::vector<float>& x, 
                       const std::vector<float>& y) {
        double totalError = 0.0;
        for (size_t i = 0; i < x.size(); ++i) {
            float prediction = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                prediction += coeffs[j] * pow(x[i], j);
            }
            totalError += pow(prediction - y[i], 2);
        }
//        Serial.println(totalError);
        return totalError / x.size();
    }
*/


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


/*
double calculateMSE(const std::vector<float>& coeffs, 
                   const std::vector<float>& x, 
                   const std::vector<float>& y) {
    // Normalization
    std::vector<float> x_norm = x;
    std::vector<float> y_norm = y;
    double x_max = *std::max_element(x.begin(), x.end());
    double y_max = *std::max_element(y.begin(), y.end());
    std::transform(x.begin(), x.end(), x_norm.begin(), [x_max](double val) { return val / x_max; });
    std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });

    double meanSquaredError = 0.0;
    double mean = 0.0;
    double M2 = 0.0;

    for (size_t i = 0; i < x_norm.size(); ++i) {
        double prediction = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            prediction += coeffs[j] * pow(x_norm[i], j);
        }
        double error = prediction - y_norm[i];
        double squaredError = error * error;
        double delta = squaredError - mean;
        mean += delta / (i + 1);
        M2 += delta * (squaredError - mean);
    }

    meanSquaredError = mean;
    return meanSquaredError;
}
*/

 // Gradient Descent with adaptive learning and regularization
    std::vector<float> gradientDescentFit(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        RegularizationType regType = L2_RIDGE,
        LossFunctionType lossType = MEAN_SQUARED_ERROR
    ) {
        std::vector<float> coeffs(degree + 1, 0.0);

        // Normalize x and y
        std::vector<float> x_norm = x;
        //std::vector<float> y_norm = y;
//        double x_min = *std::min_element(x.begin(), x.end());
//        Serial.println(x_min);
//        double x_max = *std::max_element(x.begin(), x.end());
//        Serial.println(x_max);
//        Serial.print("orig:");
//        Serial.println(x[0]); 
//        Serial.println(x[x.size()-1]);
  
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
        // Adaptive learning rate
        double dynamicLearningRate = LEARNING_RATE;
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(degree + 1, 0.0);
         //   double prevMSE = calculateMSE(coeffs, x_norm, y);

            // Compute gradients
            for (size_t j = 0; j < degree; ++j) {
                for (size_t i = 0; i < x_norm.size(); ++i) {
                    float prediction = 0.0;
                    for (size_t k = 0; k < degree; ++k) {
                        prediction += coeffs[k] * pow(x_norm[i], k);
                    }
                    float error = prediction - y[i];
                    double gradient = 2 * error * pow(x_norm[i], j);

                    // Regularization
                    switch (regType) {
                        case L1_LASSO:
                            gradient += L1_LAMBDA * (coeffs[j] > 0 ? 1 : -1);
                            break;
                        case L2_RIDGE:
                            gradient += 2 * L2_LAMBDA * coeffs[j];
                            break;
                        case ELASTIC_NET:
                            gradient += L1_LAMBDA * (coeffs[j] > 0 ? 1 : -1) + 
                                        2 * L2_LAMBDA * coeffs[j];
                            break;
                        default:
                            break;
                    }
                    gradients[j] += gradient;
                }
                gradients[j] /= x_norm.size();
            }

            // Update coefficients
            for (size_t j = 0; j < degree; ++j) {
                coeffs[j] -= dynamicLearningRate * gradients[j];
  //              coeffs[j] =+ dynamicLearningRate * 0.0001;
                
            }

            // Adaptive learning rate
       //     double currentMSE = calculateMSE(coeffs, x_norm, y);
       //     if (currentMSE > prevMSE) {
       //         dynamicLearningRate *= 0.5;  // Reduce learning rate
       //     } else if (currentMSE < prevMSE) {
       //         dynamicLearningRate *= 1.05;  // Slightly increase learning rate
       //     }

        }
       
        return coeffs;
    }


 // Simulated Annealing for global optimization
    std::vector<float> simulatedAnnealingFit(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        std::vector<float> bestCoeffs(degree + 1, 0.0);
        float bestError = std::numeric_limits<float>::max();

        // Normalize x and y
        std::vector<float> x_norm = x;
        std::vector<float> y_norm = y;
        double x_max = *std::max_element(x.begin(), x.end());
        double y_max = *std::max_element(y.begin(), y.end());
        std::transform(x.begin(), x.end(), x_norm.begin(), [x_max](double val) { return val / x_max; });
        std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });
        
        float temperature = 1000.0;
        float coolingRate = 0.003;

        // Random number generator setup (pseudo-random for deterministic behavior)
        unsigned long seed = micros();
        
        while (temperature > 1.0) {
            // Create slightly perturbed coefficients
            std::vector<float> currentCoeffs = bestCoeffs;
            for (auto& coeff : currentCoeffs) {
                // Add random perturbation scaled by temperature
                coeff += ((random(seed) / (float)RAND_MAX) - 0.5) * temperature * 0.1;
            }

            float currentError = calculateMSE(currentCoeffs, x_norm, y_norm);

            // Probabilistic acceptance of worse solutions
            float deltaCost = currentError - bestError;
            float acceptanceProbability = exp(-deltaCost / temperature);
            
            if (deltaCost < 0 || ((random(seed) / (float)RAND_MAX) < acceptanceProbability)) {
                bestCoeffs = currentCoeffs;
                bestError = currentError;
            }

            // Cool down
            temperature *= 1 - coolingRate;
        }

        return bestCoeffs;
    }

public:
    // Optimization methods enum
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
        SIMULATED_ANNEALING
    };
    
    // Main fitting method with multiple optimization strategies
    std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        OptimizationMethod method = GRADIENT_DESCENT
    ) {
        switch (method) {
            case GRADIENT_DESCENT:
                return gradientDescentFit(x, y, degree);
            case SIMULATED_ANNEALING:
                return simulatedAnnealingFit(x, y, degree);
            default:
                // Fallback to simple least squares if method not supported
                return gradientDescentFit(x, y, degree);
        }
    }

    // Advanced polynomial evaluation with error bounds
    float evaluatePolynomial(
        const std::vector<float>& coeffs, 
        float x, 
        float& errorBound
    ) {
        float result = 0.0;
        errorBound = 0.0;

        for (size_t i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * pow(x, i);
            // Simple error propagation
           // errorBound += abs(coeffs[i]) * pow(x, i);
        }

        return result;
    }
};

//------------------- graphics


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
      AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

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
        //std::vector<float> derivative = computeDerivative(coeffs);

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
                yFitted += coeffs[j] * pow(dataX-xMin, j);
            }
            
            // Map fitted y to screen
            int y = mapFloat(yFitted, yMin, yMax, dataPlotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }

        // Plot derivative in bottom region
/*
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
                display.drawPixel(x, y-1, SSD1306_WHITE);
                display.drawPixel(x, y,   SSD1306_WHITE);
            }
        }
*/
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
        yield();
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
    AdvancedOLEDVisualizer oledViz;

public:
    void begin() {
        oledViz.begin();
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

    bool detectExponentialGrowth() {
        if (timestamps.size() < 10) return false;

        // Try different polynomial degrees and optimization methods
//        std::vector<int> degrees = {3, 4, 5, 6, 7};
        std::vector<uint8_t> degrees = {4};

        float bestGrowthRate = 0.0;
        bool growthDetected = false;
        std::vector<float> bestCoeffs;
        std::vector<AdvancedPolynomialFitter::OptimizationMethod> methods = {
            AdvancedPolynomialFitter::GRADIENT_DESCENT,
            //AdvancedPolynomialFitter::LEVENBERG_MARQUARDT,
            //AdvancedPolynomialFitter::NELDER_MEAD,
            //AdvancedPolynomialFitter::SIMULATED_ANNEALING
        };

        for (uint8_t degree : degrees) {
            for (auto method : methods) {
                std::vector<float> coeffs = polynomialFitter.fitPolynomial(
                    timestamps, values, degree, method
                );
                bestCoeffs = coeffs; // ensure there is some fit in coeffs sent to visualizer
                // Analyze coefficients for exponential characteristics
                float errorBound;
                float prediction = polynomialFitter.evaluatePolynomial(
                    coeffs, timestamps.back()-timestamps[0], errorBound
                );

                // Compute growth rate
                float growthRate = computeGrowthRate(coeffs, timestamps);

                // Sophisticated growth detection criteria
            if (growthRate > 0.2 && growthRate < 10.0) {
                growthDetected = true;
               if (growthRate > bestGrowthRate) {
                    bestGrowthRate = growthRate;
                    bestCoeffs = coeffs;
                }
            }
                    
                // Potential growth detection logic
               // if (prediction > values.back() * 1.5 && errorBound < 0.1) {
                    //return true;
                //}
            }
        }
        // Visualize results
        oledViz.visualizeGrowthAnalysis(
            //timestamps, values, bestCoeffs, 
            timestamps, values, bestCoeffs, 
            growthDetected, bestGrowthRate
        );
        return growthDetected;
    }
};

   ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    // Initialize growth detector and OLED
    growthDetector.begin();
}

void loop() {
    // Simulate exponential-like data collection
    static float time = 0;
    
    // Simulated exponential growth with noise
    float value = 1 * sin(0.2*time)*exp(0.1 * time) + random(-80, 80) / 10.0;
    
    growthDetector.addDataPoint(time, value);
    yield();
    // Detect growth and visualize
    growthDetector.detectExponentialGrowth();
    
    time += random(0,+100)/100.0;
    delay(200);
}

/*
void loop() {
    // Example usage
 
    
    // Simulate data collection
    for (int i = 0; i < 50; i++) {
        float timestamp = i;
        float value = exp(0.1 * i) + random(-10, 10) / 10.0;
        detector.addDataPoint(timestamp, value);
    }

    if (detector.detectExponentialGrowth()) {
        Serial.println("Potential Exponential Growth Detected!");
    }

    delay(5000);
}
*/
