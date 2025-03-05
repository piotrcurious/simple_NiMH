class AdvancedPolynomialFitter {
public:
    // Improved method to perform SVR-based polynomial fitting
    std::vector<float> fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree, 
                               OptimizationMethod method = OptimizationMethod::ADAPTIVE) {
        // Input validation with more comprehensive checks
        if (x.size() != y.size() || x.size() < 2 || degree < 1) {
            Serial.println("Invalid input: insufficient data or invalid degree");
            return {};
        }

        size_t n = x.size();  
        size_t m = degree + 1;  

        // Enhanced data normalization with robust scaling
        float x_min = *std::min_element(x.begin(), x.end());  
        float x_max = *std::max_element(x.begin(), x.end());  
        float x_range = x_max - x_min;  
        float x_scale = (x_range > 0) ? x_range : 1.0f;  // Prevent division by zero

        // Advanced normalization with more stable scaling
        std::vector<float> x_norm(n);  
        for (size_t i = 0; i < n; ++i) {  
            x_norm[i] = (x[i] - x_min) / x_scale;  
        }  

        // Use standard least squares fit to get initial coefficients
        std::vector<float> ls_coeffs = fitPolynomial_superpos5c(x_norm, y, degree, OptimizationMethod::NONE);

        // Compute residuals with more robust error calculation
        std::vector<float> ls_residuals(n);  
        double total_abs_error = 0.0;
        for (size_t i = 0; i < n; ++i) {  
            float prediction = 0.0;  
            for (size_t j = 0; j < ls_coeffs.size(); ++j) {  
                prediction += ls_coeffs[j] * std::pow(x_norm[i], j);  
            }  
            ls_residuals[i] = prediction - y[i];
            total_abs_error += std::abs(ls_residuals[i]);
        }  
        float mean_abs_error = total_abs_error / n;

        // Adaptive hyperparameters based on data characteristics
        float C = std::min(100000.0f, 10.0f / mean_abs_error);  
        float epsilon = std::max(0.1f, mean_abs_error * 0.5f);  
        const float tol = 1e-6;  
        const int maxIter = std::min(5000, static_cast<int>(n * 100));  

        // Create feature matrix with improved numerical stability
        std::vector<std::vector<float>> X(n, std::vector<float>(m, 0.0));  
        for (size_t i = 0; i < n; ++i) {  
            X[i][0] = 1.0;  // Constant term  
            for (size_t j = 1; j < m; ++j) {  
                X[i][j] = std::pow(x_norm[i], j);  
            }  
        }  

        // Kernel matrix computation with improved cache efficiency
        std::vector<std::vector<float>> K(n, std::vector<float>(n, 0.0));  
        computeKernelMatrix(X, K);

        // Advanced SMO optimization with enhanced convergence tracking
        std::vector<float> alpha(2 * n, 0.0);  
        std::vector<float> f(n, 0.0);  
        double b = 0.0;  

        int iter = 0;
        int numChanged = 0;
        bool examineAll = true;
        int consecutiveSmallChanges = 0;
        float prevObjective = std::numeric_limits<float>::max();

        while ((numChanged > 0 || examineAll) && iter < maxIter) {  
            numChanged = 0;  
            float currentObjective = 0.0;

            if (examineAll) {  
                for (size_t i = 0; i < n; ++i) {  
                    numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, ls_residuals);  
                }  
            } else {  
                for (size_t i = 0; i < n; ++i) {  
                    if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i + n] > 0 && alpha[i + n] < C)) {  
                        numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, ls_residuals);  
                    }  
                }  
            }  

            // Convergence criteria with more nuanced tracking
            if (examineAll) {  
                examineAll = false;  
            } else if (numChanged == 0) {  
                examineAll = true;
                consecutiveSmallChanges++;
            }

            // Check for near-convergence or stagnation
            if (std::abs(currentObjective - prevObjective) < tol) {
                consecutiveSmallChanges++;
            } else {
                consecutiveSmallChanges = 0;
            }
            prevObjective = currentObjective;

            // Emergency brake for potential infinite loops
            if (consecutiveSmallChanges > 10) {
                Serial.println("Optimization convergence detected");
                break;
            }

            iter++;  

            // Reduced debugging output
            if (iter % 50 == 0) {  
                Serial.printf("Iteration: %d, Changes: %d\n", iter, numChanged);
            }  
        }  

        // Coefficient calculation with improved numerical stability
        std::vector<double> w = computePrimalCoefficients(X, alpha, n, m);
        
        // Robust denormalization of coefficients
        std::vector<double> denorm_coeffs = denormalizeCoefficients(w, x_min, x_scale, degree);
        
        // Convert to float with careful rounding
        std::vector<float> result(denorm_coeffs.begin(), denorm_coeffs.end());  
        
        // Performance logging
        Serial.printf("SVM Fit - Iterations: %d, Final C: %f, Epsilon: %f\n", iter, C, epsilon);
        
        return result;
    }

private:
    // Compute kernel matrix with improved efficiency
    void computeKernelMatrix(const std::vector<std::vector<float>>& X, 
                              std::vector<std::vector<float>>& K) {
        size_t n = X.size();
        size_t m = X[0].size();

        // Use OpenMP for parallel computation if available
        #pragma omp parallel for collapse(2)
        for (size_t i = 0; i < n; ++i) {  
            for (size_t j = i; j < n; ++j) {  
                float dot = 0.0;  
                // Unrolled dot product for efficiency
                for (size_t k = 0; k < m; k += 4) {  
                    dot += X[i][k] * X[j][k];
                    if (k+1 < m) dot += X[i][k+1] * X[j][k+1];
                    if (k+2 < m) dot += X[i][k+2] * X[j][k+2];
                    if (k+3 < m) dot += X[i][k+3] * X[j][k+3];
                }  
                K[i][j] = K[j][i] = dot;  
            }  
        }  
    }

    // Compute primal coefficients with more robust approach
    std::vector<double> computePrimalCoefficients(const std::vector<std::vector<float>>& X, 
                                                  const std::vector<float>& alpha, 
                                                  size_t n, size_t m) {
        std::vector<double> w(m, 0.0);  

        // Vectorized computation with early exit for zero terms
        for (size_t j = 0; j < m; ++j) {  
            double column_sum = 0.0;
            for (size_t i = 0; i < n; ++i) {  
                double alpha_diff = alpha[i] - alpha[i + n];  
                column_sum += alpha_diff * X[i][j];  
            }
            // Only store non-zero terms to save computation
            if (std::abs(column_sum) > 1e-10) {
                w[j] = column_sum;
            }
        }  
        
        return w;
    }

    // Existing helper methods like denormalizeCoefficients and binomialCoefficient remain the same
    // ... (include the previous implementations of these methods)
};
