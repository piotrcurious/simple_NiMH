// [Previous includes and constants remain the same]

class AdvancedPolynomialFitter {
private:
    static constexpr int MAX_ITERATIONS = 100;  // Reduced to prevent overflow
    static constexpr double LEARNING_RATE = 1e-4;  // Adjusted for better convergence
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6;
    static constexpr float L2_LAMBDA = 0.001f;

    struct NormalizedData {
        std::vector<float> data;
        float min;
        float max;
    };

    NormalizedData normalizeData(const std::vector<float>& data) const {
        NormalizedData result;
        result.min = *std::min_element(data.begin(), data.end());
        result.max = *std::max_element(data.begin(), data.end());
        float range = result.max - result.min;
        
        if (range < 1e-6) range = 1.0f;  // Prevent division by zero
        
        result.data.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            result.data[i] = (data[i] - result.min) / range;
        }
        return result;
    }

    std::vector<float> gradientDescentFit(const std::vector<float>& x, 
                                         const std::vector<float>& y, 
                                         size_t degree,
                                         RegularizationType regType = RegularizationType::L2_RIDGE) {
        // Normalize both x and y
        NormalizedData x_norm = normalizeData(x);
        NormalizedData y_norm = normalizeData(y);
        
        std::vector<float> coeffs(degree + 1, 0.0f);
        coeffs[0] = y_norm.data[0];  // Initialize with first y value
        
        double curr_learning_rate = LEARNING_RATE;
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            std::vector<float> gradients(degree + 1, 0.0f);
            
            // Compute gradients
            for (size_t i = 0; i < x_norm.data.size(); ++i) {
                // Compute prediction
                float x_val = x_norm.data[i];
                float x_power = 1.0f;
                float prediction = coeffs[0];
                
                for (size_t j = 1; j <= degree; ++j) {
                    x_power *= x_val;
                    prediction += coeffs[j] * x_power;
                }
                
                // Compute error
                float error = prediction - y_norm.data[i];
                
                // Update gradients
                x_power = 1.0f;
                for (size_t j = 0; j <= degree; ++j) {
                    gradients[j] += error * x_power;
                    x_power *= x_val;
                }
            }
            
            // Update coefficients
            float max_gradient = 0.0f;
            for (size_t j = 0; j <= degree; ++j) {
                gradients[j] = (gradients[j] / x_norm.data.size()) + L2_LAMBDA * coeffs[j];
                max_gradient = std::max(max_gradient, std::abs(gradients[j]));
                coeffs[j] -= curr_learning_rate * gradients[j];
            }
            
            // Adjust learning rate
            if (max_gradient > 1.0f) {
                curr_learning_rate *= 0.5f;
            }
            
            // Early stopping if gradients are very small
            if (max_gradient < CONVERGENCE_THRESHOLD) {
                break;
            }
        }
        
        // Denormalize coefficients
        float y_range = y_norm.max - y_norm.min;
        float x_range = x_norm.max - x_norm.min;
        std::vector<float> final_coeffs(degree + 1);
        
        // Adjust coefficients for denormalized data
        for (size_t i = 0; i <= degree; ++i) {
            float coeff = coeffs[i] * y_range;
            float x_scale = std::pow(x_range, -static_cast<float>(i));
            final_coeffs[i] = coeff * x_scale;
            
            // Adjust for x offset
            for (size_t j = i; j <= degree; ++j) {
                float binomial = 1.0f;
                for (size_t k = 0; k < i; ++k) {
                    binomial *= (j - k) / (k + 1.0f);
                }
                final_coeffs[j] -= coeff * x_scale * binomial * std::pow(-x_norm.min * x_range, j - i);
            }
        }
        
        return final_coeffs;
    }

public:
    std::vector<float> fitPolynomial(const std::vector<float>& x, 
                                    const std::vector<float>& y, 
                                    size_t degree,
                                    OptimizationMethod method = OptimizationMethod::GRADIENT_DESCENT) {
        if (x.size() < degree + 1) {
            // Return linear fit if not enough points
            degree = 1;
        }
        return gradientDescentFit(x, y, degree);
    }
};

class AdvancedOLEDVisualizer {
    // ... [Previous code remains the same until visualizeGrowthAnalysis] ...

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
        
        // Add padding to ranges
        float x_padding = (x_max - x_min) * 0.1f;
        float y_padding = (y_max - y_min) * 0.1f;
        x_min -= x_padding;
        x_max += x_padding;
        y_min -= y_padding;
        y_max += y_padding;
        
        constexpr int PLOT_HEIGHT = 40;
        
        // Plot data points
        for (size_t i = 0; i < x_data.size(); ++i) {
            int x = mapFloat(x_data[i], x_min, x_max, 0, SCREEN_WIDTH - 1);
            int y = mapFloat(y_data[i], y_min, y_max, PLOT_HEIGHT - 1, 0);
            display.drawPixel(x, y, SSD1306_WHITE);
        }
        
        // Plot fitted curve with bounds checking
        float prev_y = 0;
        bool prev_valid = false;
        
        for (int i = 0; i < SCREEN_WIDTH; ++i) {
            float x = mapFloat(i, 0, SCREEN_WIDTH - 1, x_min, x_max);
            float y = 0;
            
            // Evaluate polynomial with overflow protection
            float x_power = 1.0f;
            for (size_t j = 0; j < coeffs.size() && j <= 4; ++j) {  // Limit to degree 4
                y += coeffs[j] * x_power;
                if (std::abs(y) > 1e6) {  // Prevent overflow
                    y = prev_y;
                    break;
                }
                x_power *= x;
                if (std::abs(x_power) > 1e6) break;  // Prevent overflow
            }
            
            int plot_y = mapFloat(y, y_min, y_max, PLOT_HEIGHT - 1, 0);
            
            // Draw line segment if points are valid
            if (plot_y >= 0 && plot_y < PLOT_HEIGHT) {
                if (prev_valid) {
                    display.drawLine(i-1, prev_y, i, plot_y, SSD1306_WHITE);
                }
                prev_y = plot_y;
                prev_valid = true;
            } else {
                prev_valid = false;
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

// [Rest of the code remains the same]
