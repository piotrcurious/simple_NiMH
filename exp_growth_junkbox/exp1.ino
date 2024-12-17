#include <Arduino.h>
#include <Eigen.h>  // Linear algebra library for polynomial fitting
#include <vector>

using namespace Eigen;

class ExponentialGrowthDetector {
private:
    // Configuration parameters
    const int MAX_DATASET_SIZE = 100;
    const int POLYNOMIAL_DEGREES[8] = {3, 4, 5, 6, 7, 8, 9, 10};
    
    // Threshold configuration for growth detection
    struct GrowthThreshold {
        int duration;      // Time steps
        float minGrowth;   // Minimum growth rate threshold
        float maxGrowth;   // Maximum growth rate threshold
    };

    // Data structure for timestamp/value pairs
    struct DataPoint {
        unsigned long timestamp;
        float value;
    };

    // Thresholds table
    GrowthThreshold growthThresholdTable[8] = {
        {2,  0.1,  0.5},   // 2-step duration
        {4,  0.2,  1.0},   // 4-step duration
        {8,  0.3,  1.5},   // 8-step duration
        {16, 0.4,  2.0},   // 16-step duration
        {32, 0.5,  2.5},   // 32-step duration
        {64, 0.6,  3.0},   // 64-step duration
        {128, 0.7, 3.5},   // 128-step duration
        {256, 0.8, 4.0}    // 256-step duration
    };

    std::vector<DataPoint> rollingDataset;

    // Polynomial fitting method using Eigen
    VectorXd fitPolynomial(const std::vector<float>& values, int degree) {
        int n = values.size();
        MatrixXd X(n, degree + 1);
        VectorXd Y(n);

        // Construct design matrix
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= degree; j++) {
                X(i, j) = pow(i, j);
            }
            Y(i) = values[i];
        }

        // Solve using least squares
        return (X.transpose() * X).ldlt().solve(X.transpose() * Y);
    }

    // Derivative of polynomial coefficients
    VectorXd polynomialDerivative(const VectorXd& coeffs) {
        VectorXd derivative(coeffs.size() - 1);
        for (int i = 1; i < coeffs.size(); i++) {
            derivative[i-1] = i * coeffs[i];
        }
        return derivative;
    }

public:
    void addDataPoint(unsigned long timestamp, float value) {
        if (rollingDataset.size() >= MAX_DATASET_SIZE) {
            rollingDataset.erase(rollingDataset.begin());
        }
        rollingDataset.push_back({timestamp, value});
    }

    bool detectExponentialGrowth() {
        if (rollingDataset.size() < 8) return false;

        std::vector<float> values;
        for (const auto& point : rollingDataset) {
            values.push_back(point.value);
        }

        for (int degreeIndex = 0; degreeIndex < 8; degreeIndex++) {
            int degree = POLYNOMIAL_DEGREES[degreeIndex];
            VectorXd coeffs = fitPolynomial(values, degree);
            VectorXd derivative = polynomialDerivative(coeffs);

            // Analyze derivative to detect growth characteristics
            float maxDerivative = derivative.maxCoeff();
            float minDerivative = derivative.minCoeff();

            // Check against threshold table
            GrowthThreshold& threshold = growthThresholdTable[degreeIndex];
            
            if (maxDerivative > threshold.minGrowth && 
                maxDerivative < threshold.maxGrowth && 
                rollingDataset.size() >= threshold.duration) {
                return true;
            }
        }
        return false;
    }

    float getGrowthRate() {
        // Implement complex growth rate calculation
        return 0.0;  // Placeholder
    }
};

// Global instance
ExponentialGrowthDetector growthDetector;

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Example data injection (replace with actual sensor data)
    unsigned long currentTime = millis();
    float sensorValue = random(100, 1000) / 100.0;
    
    growthDetector.addDataPoint(currentTime, sensorValue);
    
    if (growthDetector.detectExponentialGrowth()) {
        Serial.println("Exponential Growth Detected!");
    }
    
    delay(1000);  // Sample interval
}
