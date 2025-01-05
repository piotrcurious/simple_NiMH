#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuration constants (unchanged from previous version)
constexpr uint16_t SCREEN_WIDTH = 128;
constexpr uint16_t SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint8_t OLED_DC = 16;
constexpr uint8_t OLED_CS = 5;
constexpr uint8_t OLED_RESET = 17;
constexpr uint32_t OLED_BITRATE = 80000000;
constexpr size_t MAX_DATA_POINTS = 200;
constexpr size_t MAX_POLYNOMIAL_DEGREE = 5;
constexpr float MIN_GROWTH_RATE = 0.2f;
constexpr float MAX_GROWTH_RATE = 60.0f;

#define LED_BUILTIN 2 

// Keep all the existing includes and configuration constants...

struct DataNormalization {
    float xOffset;
    float xScale;
    float yOffset;
    float yScale;
    float meanX;
    float meanY;
    float stdX;
    float stdY;
};

struct PolynomialFit {
    float coefficients[MAX_POLYNOMIAL_DEGREE + 1];
    uint8_t degree;
    float growthRate;
    float error;
    float r2Score;  // Added R² score for fit quality assessment
    DataNormalization norm;
};

class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_EPOCHS = 50000;
    static constexpr float INITIAL_LEARNING_RATE = 0.001f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6f;
    static constexpr float L2_LAMBDA = 0.0001f; // Regularization term
    static constexpr size_t BATCH_SIZE = 10;  // Mini-batch size

/*
    // Evaluate polynomial using Horner's method for better stability
    float evaluatePolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }
*/
    // Helper: Normalize data for better stability
    void normalizeData(const float* x, const float* y, size_t n, 
                       float& xMean, float& xRange, 
                       float& yMean, float& yRange, 
                       float* normX, float* normY) {
        xMean = 0;
        yMean = 0;

        // Calculate means
        for (size_t i = 0; i < n; ++i) {
            xMean += x[i];
            yMean += y[i];
        }
        xMean /= n;
        yMean /= n;

        xRange = 0;
        yRange = 0;

        // Calculate ranges
        for (size_t i = 0; i < n; ++i) {
            xRange = max(xRange, fabs(x[i] - xMean));
            yRange = max(yRange, fabs(y[i] - yMean));
        }

        // Normalize data
        for (size_t i = 0; i < n; ++i) {
            normX[i] = (x[i] - xMean) / xRange;
            normY[i] = (y[i] - yMean) / yRange;
        }
    }


    // Compute gradients and error for mini-batch
    float computeGradients(float* x, float* y, size_t n, 
                           float* coeffs, uint8_t degree, 
                           float* gradients) {
        float totalError = 0;

        // Initialize gradients
        for (uint8_t i = 0; i <= degree; ++i) {
            gradients[i] = 0;
        }

        // Compute gradients and error
        for (size_t i = 0; i < n; ++i) {
            float prediction = evaluatePolynomial(coeffs, degree, x[i]);
            float error = prediction - y[i];
            totalError += error * error;

            // Update gradients
            float xPower = 1.0f;
            for (uint8_t j = 0; j <= degree; ++j) {
                gradients[j] += 2 * error * xPower / n;
                xPower *= x[i];
            }
        }

        // Apply L2 regularization
        for (uint8_t j = 0; j <= degree; ++j) {
            gradients[j] += 2 * L2_LAMBDA * coeffs[j];
        }

        return totalError / n;
    }

public:
    // Calculate the polynomial using coefficients
    float evaluatePolynomial(float* coeffs, uint8_t degree, float x)  {
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    
    PolynomialFit fitPolynomial(const float* x, const float* y, size_t n, uint8_t degree) {
        PolynomialFit fitResult;
        fitResult.degree = degree;

        // Allocate normalized data
        float normX[MAX_DATA_POINTS];
        float normY[MAX_DATA_POINTS];

        // Normalize data
        float xMean, xRange, yMean, yRange;
        normalizeData(x, y, n, xMean, xRange, yMean, yRange, normX, normY);

        // Initialize coefficients
        for (uint8_t i = 0; i <= degree; ++i) {
            fitResult.coefficients[i] = 0;
        }

        float learningRate = INITIAL_LEARNING_RATE;
        float prevError = INFINITY;

        // Iterative optimization
        for (int epoch = 0; epoch < MAX_EPOCHS; ++epoch) {
            float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};

            // Process data in batches
            for (size_t i = 0; i < n; i += BATCH_SIZE) {
                size_t batchSize = min(BATCH_SIZE, n - i);
                computeGradients(normX + i, normY + i, batchSize, fitResult.coefficients, degree, gradients);

                // Update coefficients
                for (uint8_t j = 0; j <= degree; ++j) {
                    fitResult.coefficients[j] -= learningRate * gradients[j];
                }
            }

            // Compute total error
            float currentError = computeGradients(normX, normY, n, fitResult.coefficients, degree, gradients);
            if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                break;
            }

            prevError = currentError;
        }

        // Denormalize coefficients
        for (uint8_t j = 0; j <= degree; ++j) {
            fitResult.coefficients[j] *= yRange / pow(xRange, j);
        }
        fitResult.coefficients[0] += yMean - fitResult.coefficients[0] * xMean;

        fitResult.error = prevError;
        return fitResult;
    }
};

/*
class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 1000;  // Increased iterations
    static constexpr float INITIAL_LEARNING_RATE = 0.01f;
    static constexpr float MIN_LEARNING_RATE = 1e-6f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6f;
    static constexpr float L2_LAMBDA = 0.1f;  // Reduced regularization

    DataNormalization normalizeData(float* xNorm, 
                                  float* yNorm,
                                  const float* x,
                                  const float* y,
                                  size_t n) {
        DataNormalization norm = {0};
        
        // Calculate mean and standard deviation
        for (size_t i = 0; i < n; ++i) {
            norm.meanX += x[i];
            norm.meanY += y[i];
        }
        norm.meanX /= n;
        norm.meanY /= n;
        
        for (size_t i = 0; i < n; ++i) {
            float dx = x[i] - norm.meanX;
            float dy = y[i] - norm.meanY;
            norm.stdX += dx * dx;
            norm.stdY += dy * dy;
        }
        norm.stdX = sqrt(norm.stdX / (n - 1));
        norm.stdY = sqrt(norm.stdY / (n - 1));
        
        // Prevent division by zero
        if (norm.stdX < 1e-6f) norm.stdX = 1.0f;
        if (norm.stdY < 1e-6f) norm.stdY = 1.0f;
        
        // Z-score normalization
        for (size_t i = 0; i < n; ++i) {
            xNorm[i] = (x[i] - norm.meanX) / norm.stdX;
            yNorm[i] = (y[i] - norm.meanY) / norm.stdY;
        }
        
        // Store original scale information for denormalization
        norm.xOffset = norm.meanX;
        norm.xScale = norm.stdX;
        norm.yOffset = norm.meanY;
        norm.yScale = norm.stdY;
        
        return norm;
    }

    float evaluateNormalizedPolynomial(const float* coeffs, uint8_t degree, float x) const {
        float result = coeffs[degree];
        float xPow = x;
        
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float calculateR2Score(const float* y, const float* yPred, size_t n) {
        float meanY = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            meanY += y[i];
        }
        meanY /= n;
        
        float ssTotal = 0.0f;
        float ssResidual = 0.0f;
        
        for (size_t i = 0; i < n; ++i) {
            float diff = y[i] - meanY;
            ssTotal += diff * diff;
            diff = y[i] - yPred[i];
            ssResidual += diff * diff;
        }
        
        return 1.0f - (ssResidual / (ssTotal + 1e-10f));
    }

public:
    PolynomialFit fitPolynomial(const float* x,
                               const float* y,
                               size_t n,
                               uint8_t degree) {
        PolynomialFit result;
        result.degree = degree;

        float xNorm[MAX_DATA_POINTS];
        float yNorm[MAX_DATA_POINTS];
        float yPred[MAX_DATA_POINTS];
        
        result.norm = normalizeData(xNorm, yNorm, x, y, n);

        // Initialize coefficients using scaled random values
        for (uint8_t i = 0; i <= degree; ++i) {
            result.coefficients[i] = (random(1000) - 500) / (0.1f * (i + 1));
        }

        float learningRate = INITIAL_LEARNING_RATE;
        float prevError = INFINITY;
        float minError = INFINITY;
        float bestCoeffs[MAX_POLYNOMIAL_DEGREE + 1];
        
        // Multiple restarts with momentum
        for (int restart = 0; restart < 5; restart++) {
            float velocities[MAX_POLYNOMIAL_DEGREE + 1] = {0};
            const float momentum = 0.9f;
            
            for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
                float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};
                float currentError = 0.0f;
                
                // Calculate predictions and gradients
                for (size_t i = 0; i < n; ++i) {
                    float prediction = evaluateNormalizedPolynomial(result.coefficients, degree, xNorm[i]);
                    yPred[i] = prediction;
                    float error = prediction - yNorm[i];
                    currentError += error * error;
                    
                    float xPow = 1.0f;
                    for (uint8_t j = 0; j <= degree; ++j) {
                        gradients[j] += 2.0f * error * xPow;
                        xPow *= xNorm[i];
                    }
                }
                
                currentError = sqrt(currentError / n);
                
                // L2 regularization with degree-based scaling
                for (uint8_t j = 0; j <= degree; ++j) {
                    gradients[j] = gradients[j] / n + L2_LAMBDA * result.coefficients[j] * (j + 1);
                }

                // Update with momentum
                for (uint8_t j = 0; j <= degree; ++j) {
                    velocities[j] = momentum * velocities[j] - learningRate * gradients[j];
                    result.coefficients[j] += velocities[j];
                }

                // Adaptive learning rate
                if (currentError > prevError) {
                    learningRate *= 0.5f;
                    if (learningRate < MIN_LEARNING_RATE) {
                        break;
                    }
                } else if (currentError < minError) {
                    minError = currentError;
                    memcpy(bestCoeffs, result.coefficients, sizeof(bestCoeffs));
                }

                if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                    break;
                }
                prevError = currentError;
            }
            
            if (restart < 4) {
                learningRate = INITIAL_LEARNING_RATE;
                for (uint8_t i = 0; i <= degree; ++i) {
                    result.coefficients[i] = (random(1000) - 500) / (1000.0f * (i + 1));
                }
            }
        }

        memcpy(result.coefficients, bestCoeffs, sizeof(bestCoeffs));
        result.error = minError;

        // Calculate R² score
        for (size_t i = 0; i < n; ++i) {
            yPred[i] = evaluateNormalizedPolynomial(bestCoeffs, degree, xNorm[i]);
        }
        result.r2Score = calculateR2Score(yNorm, yPred, n);

        // Calculate growth rate using denormalized values
        float x1 = x[n-1];
        float x2 = x[n-1] + (result.norm.xScale / n);
        float y1 = denormalizeY(evaluateNormalizedPolynomial(bestCoeffs, degree, normalizeX(x1, result.norm)), result.norm);
        float y2 = denormalizeY(evaluateNormalizedPolynomial(bestCoeffs, degree, normalizeX(x2, result.norm)), result.norm);
        
        result.growthRate = (y1 != 0.0f) ? ((y2 - y1) / y1) : 0.0f;
        
        return result;
    }

    float normalizeX(float x, const DataNormalization& norm) {
        return (x - norm.xOffset) / norm.xScale;
    }

    float denormalizeY(float y, const DataNormalization& norm) {
        return y * norm.yScale + norm.yOffset;
    }

    float evaluatePolynomial(const PolynomialFit& fit, float x) {
        float xNorm = normalizeX(x, fit.norm);
        float yNorm = evaluateNormalizedPolynomial(fit.coefficients, fit.degree, xNorm);
        return denormalizeY(yNorm, fit.norm);
    }
};
*/


/*
struct DataNormalization {
    float xOffset;
    float xScale;
    float yOffset;
    float yScale;
};

struct PolynomialFit {
    float coefficients[MAX_POLYNOMIAL_DEGREE + 1];
    uint8_t degree;
    float growthRate;
    float error;
    DataNormalization norm;
};

class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 50;
    static constexpr float LEARNING_RATE = 0.01f;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-5f;
    static constexpr float L2_LAMBDA = 0.001f;

    DataNormalization normalizeData(float* xNorm, 
                                  float* yNorm,
                                  const float* x,
                                  const float* y,
                                  size_t n) {
        DataNormalization norm;
        
        // Find min/max for x and y
        norm.xOffset = x[0];
        norm.xScale = x[0];
        norm.yOffset = y[0];
        norm.yScale = y[0];
        
        for (size_t i = 1; i < n; ++i) {
            norm.xOffset = min(norm.xOffset, x[i]);
            norm.xScale = max(norm.xScale, x[i]);
            norm.yOffset = min(norm.yOffset, y[i]);
            norm.yScale = max(norm.yScale, y[i]);
        }
        
        // Calculate scales
        norm.xScale = norm.xScale - norm.xOffset;
        norm.yScale = norm.yScale - norm.yOffset;
        
        if (norm.xScale < 1e-6f) norm.xScale = 1.0f;
        if (norm.yScale < 1e-6f) norm.yScale = 1.0f;
        
        // Normalize data to [-1, 1] range
        for (size_t i = 0; i < n; ++i) {
            xNorm[i] = 2.0f * (x[i] - norm.xOffset) / norm.xScale - 1.0f;
            yNorm[i] = 2.0f * (y[i] - norm.yOffset) / norm.yScale - 1.0f;
        }
        
        return norm;
    }

    float evaluateNormalizedPolynomial(const float* coeffs, uint8_t degree, float x) const {
        // Use Horner's method for better numerical stability
        float result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float denormalizeY(float y, const DataNormalization& norm) {
        return (y + 1.0f) * 0.5f * norm.yScale + norm.yOffset;
    }

    float normalizeX(float x, const DataNormalization& norm) {
        return 2.0f * (x - norm.xOffset) / norm.xScale - 1.0f;
    }

public:
    PolynomialFit fitPolynomial(const float* x,
                               const float* y,
                               size_t n,
                               uint8_t degree) {
        PolynomialFit result;
        result.degree = degree;

        // Allocate temporary arrays for normalized data
        float xNorm[MAX_DATA_POINTS];
        float yNorm[MAX_DATA_POINTS];
        
        // Normalize input data
        result.norm = normalizeData(xNorm, yNorm, x, y, n);

        // Initialize coefficients with small random values
        for (uint8_t i = 0; i <= degree; ++i) {
            result.coefficients[i] = (random(1000) - 500) / 10000.0f;
        }

        float learningRate = LEARNING_RATE;
        float prevError = INFINITY;
        float minError = INFINITY;
        float bestCoeffs[MAX_POLYNOMIAL_DEGREE + 1];
        
        // Gradient descent with multiple restarts
        for (int restart = 0; restart < 3; restart++) {
            for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
                float gradients[MAX_POLYNOMIAL_DEGREE + 1] = {0};
                float currentError = 0.0f;
                
                // Calculate gradients and error
                for (size_t i = 0; i < n; ++i) {
                    float prediction = evaluateNormalizedPolynomial(result.coefficients, degree, xNorm[i]);
                    float error = prediction - yNorm[i];
                    currentError += error * error;
                    
                    // Update gradients using Horner's method
                    float xPow = 1.0f;
                    for (uint8_t j = 0; j <= degree; ++j) {
                        gradients[j] += 2.0f * error * xPow;
                        xPow *= xNorm[i];
                    }
                }
                
                currentError = sqrt(currentError / n);  // RMSE
                
                // Add L2 regularization
                for (uint8_t j = 0; j <= degree; ++j) {
                    gradients[j] = gradients[j] / n + 2.0f * L2_LAMBDA * result.coefficients[j];
                }

                // Update coefficients
                for (uint8_t j = 0; j <= degree; ++j) {
                    result.coefficients[j] -= learningRate * gradients[j];
                }

                // Adaptive learning rate
                if (currentError > prevError) {
                    learningRate *= 0.5f;
                } else if (currentError < minError) {
                    minError = currentError;
                    memcpy(bestCoeffs, result.coefficients, sizeof(bestCoeffs));
                }

                if (abs(currentError - prevError) < CONVERGENCE_THRESHOLD) {
                    break;
                }
                prevError = currentError;
            }
            
            // Prepare for next restart if needed
            if (restart < 2) {
                learningRate = LEARNING_RATE;
                for (uint8_t i = 0; i <= degree; ++i) {
                    result.coefficients[i] = (random(1000) - 500) / 10000.0f;
                }
            }
        }

        // Use the best coefficients found
        memcpy(result.coefficients, bestCoeffs, sizeof(bestCoeffs));
        result.error = minError;

        // Calculate growth rate using denormalized values
        float x1 = x[n-1];
        float x2 = x[n-1] + (result.norm.xScale / n);  // Small time step
        
        float y1 = evaluateNormalizedPolynomial(result.coefficients, degree, normalizeX(x1, result.norm));
        float y2 = evaluateNormalizedPolynomial(result.coefficients, degree, normalizeX(x2, result.norm));
        
        y1 = denormalizeY(y1, result.norm);
        y2 = denormalizeY(y2, result.norm);
        
        result.growthRate = (y1 != 0.0f) ? ((y2 - y1) / y1) : 0.0f;
        
        return result;
    }

    float evaluatePolynomial(const PolynomialFit& fit, float x)  {
        float xNorm = normalizeX(x, fit.norm);
        float yNorm = evaluateNormalizedPolynomial(fit.coefficients, fit.degree, xNorm);
       // float denormY = denormalizeY(yNorm, fit.norm);
        return denormalizeY(yNorm, fit.norm);
       // return denormY;
    }
};
*/

// AdvancedOLEDVisualizer class implementation remains mostly the same,
// but update the visualization method:

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;
    AdvancedPolynomialFitter fitter;  // Add fitter for polynomial evaluation

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

    void visualizeGrowthAnalysis(float* xData,
                                float* yData,
                                size_t dataSize,
                                PolynomialFit& fit) {
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

        const int plotHeight = SCREEN_HEIGHT - 24;
       // Draw data points
        for (size_t i = 0; i < dataSize; ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH-1);
            int y = mapFloat(yData[i], yMin, yMax, plotHeight-1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }


        // Evaluate fitted curve at extremes to ensure proper y-axis scaling
        float yFitMin = fitter.evaluatePolynomial(fit.coefficients, fit.degree, xMin);
        float yFitMax = fitter.evaluatePolynomial(fit.coefficients, fit.degree, xMax);
        yMin = min(yMin, min(yFitMin, yFitMax));
        yMax = max(yMax, max(yFitMin, yFitMax));

        // Add margin to y-axis
        float yMargin = (yMax - yMin) * 0.1f;
        yMin -= yMargin;
        yMax += yMargin;

       
        
 
        // Draw fitted curve
        int16_t lastX = -1, lastY = -1;
        for (int i = 0; i < SCREEN_WIDTH; ++i) {
            float x = mapFloat(i, 0, SCREEN_WIDTH-1, xMin, xMax);
            float y = fitter.evaluatePolynomial(fit.coefficients,fit.degree, x);            
            int16_t screenY = mapFloat(y, yMin, yMax, plotHeight-1, 0);
            
            if (lastX >= 0) {
                display.drawLine(lastX, lastY, i, screenY, SSD1306_WHITE);
            }
            
            lastX = i;
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
        display.print(F(" E:"));
        display.print(fit.error, 2);

        display.display();
    }

    void displayErrorState(const char* errorMsg) {
        //display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("ERROR:"));
        display.println(errorMsg);
        display.display();
    }
};

// ExponentialGrowthDetector class remains mostly the same but with updated fit quality checking:

class ExponentialGrowthDetector {
private:
    AdvancedPolynomialFitter fitter;
    AdvancedOLEDVisualizer display;
    float timestamps[MAX_DATA_POINTS];
    float values[MAX_DATA_POINTS];
    size_t dataCount = 0;
    
    static constexpr float MAX_ACCEPTABLE_ERROR = 0.01f;  // Maximum acceptable normalized error

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
        if (dataCount < 6) {
            display.displayErrorState("Need more data");
            return false;
        }

        PolynomialFit bestFit;
        bestFit.error = INFINITY;
        bestFit.growthRate = 0;
        
        // Try different polynomial degrees
        for (uint8_t degree = 3; degree <= MAX_POLYNOMIAL_DEGREE; ++degree) {
            PolynomialFit currentFit = fitter.fitPolynomial(
                timestamps, values, dataCount, degree
            );
            
            if (currentFit.error < bestFit.error && 
                currentFit.error < MAX_ACCEPTABLE_ERROR &&
                currentFit.growthRate >= MIN_GROWTH_RATE && 
                currentFit.growthRate <= MAX_GROWTH_RATE) {
                bestFit = currentFit;
            }
        }

        // If no good fit was found, try with fewer points
        if (bestFit.error == INFINITY && dataCount > 15) {
            size_t reducedCount = 15;  // Try with last 15 points
            float* recentTimestamps = &timestamps[dataCount - reducedCount];
            float* recentValues = &values[dataCount - reducedCount];
            
            for (uint8_t degree = 2; degree <= MAX_POLYNOMIAL_DEGREE; ++degree) {
                PolynomialFit currentFit = fitter.fitPolynomial(
                    recentTimestamps, recentValues, reducedCount, degree
                );
                
                if (currentFit.error < bestFit.error && 
                    currentFit.error < MAX_ACCEPTABLE_ERROR &&
                    currentFit.growthRate >= MIN_GROWTH_RATE && 
                    currentFit.growthRate <= MAX_GROWTH_RATE) {
                    bestFit = currentFit;
                }
            }
        }

        display.visualizeGrowthAnalysis(timestamps, values, dataCount, bestFit);

        // If still no good fit, display error
        if (bestFit.error == INFINITY) {
            display.displayErrorState("Poor fit quality");
            return false;
        }

//        display.visualizeGrowthAnalysis(timestamps, values, dataCount, bestFit);
        return bestFit.growthRate >= MIN_GROWTH_RATE;
    }

    void reset() {
        dataCount = 0;
    }
};

ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));  // Better random seed initialization
    pinMode(LED_BUILTIN,OUTPUT);
    
    if (!growthDetector.begin()) {
        Serial.println(F("Display init failed"));
        while (1) {
            delay(100);
            // Blink LED or provide other visual feedback
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
}

void loop() {
    static float time = 0.0f;
    static uint32_t lastUpdateTime = 0;
    const uint32_t UPDATE_INTERVAL = 500;  // 500ms between updates
    
    uint32_t currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = currentTime;
        
        // Generate sample data with exponential growth and controlled noise
        float baseValue = 1.0f * exp(0.2f * time);
        float noiseAmount = baseValue * 0.1f;  // 10% noise
        float noise = (random(-100, 101) / 100.0f) * noiseAmount;
        float value = baseValue + noise;
        
        // Ensure value stays positive
        value = max(value, 0.1f);
        
        growthDetector.addDataPoint(time, value);
        
        if (!growthDetector.detectExponentialGrowth()) {
            Serial.println(F("No growth detected or poor fit"));
        }
        
        // Increment time with small random variation
        time += 0.1f + (random(0, 100) / 100.0f);  // 0.08 to 0.12 increment
        
        // Print debug info to Serial
        Serial.print(F("Time: "));
        Serial.print(time);
        Serial.print(F(" Value: "));
        Serial.println(value);
    }
    
    // Allow other tasks to run
    yield();
}




  
