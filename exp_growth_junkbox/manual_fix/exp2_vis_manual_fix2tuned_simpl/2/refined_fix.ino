// [Previous includes and constants remain the same]

class AdvancedPolynomialFitter {
    // [Previous enums and constants remain the same]

private:
    double calculateMSE(const std::vector<float>& coeffs, 
                       const std::vector<float>& x, 
                       const std::vector<float>& y) const {
        double mean = 0.0;
        const size_t n = x.size();
        
        for (size_t i = 0; i < n; ++i) {
            float prediction = 0.0f;
            float x_power = 1.0f;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                prediction += coeffs[j] * x_power;
                x_power *= x[i];
            }
            float error = prediction - y[i];
            double delta = (error * error - mean);
            mean += delta / (i + 1);
        }
        
        return mean;
    }

    std::vector<float> gradientDescentFit(const std::vector<float>& x, 
                                         const std::vector<float>& y, 
                                         size_t degree,
                                         RegularizationType regType = RegularizationType::L2_RIDGE) {
        // Normalize x values
        float x_min = *std::min_element(x.begin(), x.end());
        std::vector<float> x_norm(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            x_norm[i] = x[i] - x_min;
        }
        
        // Initialize coefficients
        std::vector<float> coeffs(degree + 1, 0.0f);
        double learning_rate = LEARNING_RATE;
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(degree + 1, 0.0f);
            
            // Compute gradients for each coefficient
            for (size_t i = 0; i < x_norm.size(); ++i) {
                // Calculate prediction
                float prediction = 0.0f;
                float x_power = 1.0f;
                for (size_t j = 0; j <= degree; ++j) {
                    prediction += coeffs[j] * x_power;
                    x_power *= x_norm[i];
                }
                
                // Calculate error
                float error = prediction - y[i];
                
                // Update gradients
                x_power = 1.0f;
                for (size_t j = 0; j <= degree; ++j) {
                    gradients[j] += error * x_power;
                    x_power *= x_norm[i];
                }
            }
            
            // Apply gradients with regularization
            for (size_t j = 0; j <= degree; ++j) {
                gradients[j] = (2.0f * gradients[j]) / x_norm.size();
                
                // Add regularization term
                if (regType == RegularizationType::L2_RIDGE) {
                    gradients[j] += 2.0f * L2_LAMBDA * coeffs[j];
                }
                
                // Update coefficient
                coeffs[j] -= learning_rate * gradients[j];
            }
            
            // Adaptive learning rate
            learning_rate *= 0.999f;  // Gradually reduce learning rate
        }
        
        return coeffs;
    }

public:
    // [Previous public methods remain the same]
};

class AdvancedOLEDVisualizer {
    // [Previous members and methods remain the same]

    void visualizeGrowthAnalysis(const std::vector<float>& x_data, 
                                const std::vector<float>& y_data,
                                const std::vector<float>& coeffs,
                                bool growth_detected,
                                float growth_rate) {
        if (x_data.empty() || y_data.empty()) return;

        display.clearDisplay();
        
        // Find data ranges
        float x_min = *std::min_element(x_data.begin(), x_data.end());
        float x_max = *std::max_element(x_data.begin(), x_data.end());
        float y_min = *std::min_element(y_data.begin(), y_data.end());
        float y_max = *std::max_element(y_data.begin(), y_data.end());
        
        // Add some padding to y range
        float y_padding = (y_max - y_min) * 0.1f;
        y_min -= y_padding;
        y_max += y_padding;
        
        constexpr int PLOT_HEIGHT = 40;
        
        // Plot data points
        for (size_t i = 0; i < x_data.size(); ++i) {
            int x = mapFloat(x_data[i], x_min, x_max, 0, SCREEN_WIDTH - 1);
            int y = mapFloat(y_data[i], y_min, y_max, PLOT_HEIGHT - 1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }
        
        // Plot fitted curve
        for (int i = 0; i < SCREEN_WIDTH; ++i) {
            float x = mapFloat(i, 0, SCREEN_WIDTH - 1, x_min, x_max);
            float x_normalized = x - x_min;
            
            // Evaluate polynomial
            float y = 0.0f;
            float x_power = 1.0f;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                y += coeffs[j] * x_power;
                x_power *= x_normalized;
            }
            
            int plot_y = mapFloat(y, y_min, y_max, PLOT_HEIGHT - 1, 0);
            if (plot_y >= 0 && plot_y < PLOT_HEIGHT) {
                display.drawPixel(i, plot_y, SSD1306_WHITE);
            }
        }
        
        // Display information
        display.setCursor(0, PLOT_HEIGHT + 2);
        display.print(growth_detected ? F("GROWTH:") : F("NO GROWTH"));
        display.print(F(" R:"));
        display.print(growth_rate, 2);
        
        display.setCursor(0, PLOT_HEIGHT + 12);
        display.print(F("Deg:"));
        display.print(coeffs.size() - 1);
        display.print(F(" Pts:"));
        display.print(x_data.size());
        
        display.display();
    }
};

class ExponentialGrowthDetector {
    // [Previous members and methods remain the same]
    
    float computeGrowthRate(const std::vector<float>& coeffs, float x) const {
        float x_normalized = x;
        
        // Calculate current value
        float current = 0.0f;
        float x_power = 1.0f;
        for (size_t i = 0; i < coeffs.size(); ++i) {
            current += coeffs[i] * x_power;
            x_power *= x_normalized;
        }
        
        // Calculate next value
        float next = 0.0f;
        x_power = 1.0f;
        x_normalized += 1.0f;
        for (size_t i = 0; i < coeffs.size(); ++i) {
            next += coeffs[i] * x_power;
            x_power *= x_normalized;
        }
        
        return (next - current) / (std::abs(current) + 1e-6f);
    }

public:
    bool detectExponentialGrowth() {
        if (timestamps.size() < MIN_DATA_POINTS) return false;
        
        constexpr uint8_t POLYNOMIAL_DEGREE = 4;
        auto coeffs = fitter.fitPolynomial(timestamps, values, POLYNOMIAL_DEGREE);
        
        float growth_rate = computeGrowthRate(coeffs, 0.0f);
        bool growth_detected = (growth_rate > GROWTH_THRESHOLD_MIN && 
                              growth_rate < GROWTH_THRESHOLD_MAX);
        
        oled.visualizeGrowthAnalysis(timestamps, values, coeffs, 
                                    growth_detected, growth_rate);
        
        return growth_detected;
    }
};

// [Setup and loop functions remain the same]
