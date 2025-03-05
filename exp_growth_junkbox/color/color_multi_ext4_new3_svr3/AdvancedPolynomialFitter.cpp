#include "AdvancedPolynomialFitter.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// calculate using Welford's method
double AdvancedPolynomialFitter::calculateMSE(const std::vector<float>& coeffs,
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
    // Serial.println(meanSquaredError);
    return meanSquaredError;
}

// Fit a polynomial to the data using the normal equation
std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
                                                           OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }

    // Normalize x and y
    // std::vector<float> x_norm = x;
    double x_min = *std::min_element(x.begin(), x.end());
    Serial.print("xmin:");
    Serial.print(x_min);
    double x_max = *std::max_element(x.begin(), x.end());
    Serial.print(" xmax:");
    Serial.println(x_max);

    //#ifdef REVERSED_NORMALIZATION
    // std::transform(x.begin(), x.end(), x_norm.begin(), [x_max](double val) { return val - x_max; });
    //#else
    // std::transform(x.begin(), x.end(), x_norm.begin(), [x_min](double val) { return val - x_min; });
    //#endif //#ifdef REVERSED_NORMALIZATION

    // double y_max = *std::max_element(y.begin(), y.end());
    // std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });

    Serial.println(x[0]);
    Serial.println(x[x.size() - 1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Construct the Vandermonde matrix
    std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        double xi = 1.0;
        for (size_t j = 0; j < m; ++j) {
            A[i][j] = xi;
            xi *= x[i];
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

    switch (method) {
    case GRADIENT_DESCENT:
        // Implement gradient descent here if needed
        break;
    case LEVENBERG_MARQUARDT:
        result = levenbergMarquardt(result, x, y, degree);
        break;
    case NELDER_MEAD:
        // Implement Nelder-Mead here if needed
        break;
    default:  // no optimization
        // result = levenbergMarquardt(x_norm, y, degree);
        break;
    }

    return result;
}

//---------------------------------------svm

  // Improved method to perform SVR-based polynomial fitting
    std::vector<float> AdvancedPolynomialFitter::fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree, 
                               OptimizationMethod method ) {
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
        //float C = std::min(100000.0f, 10.0f / mean_abs_error);  
        float C = 100000.0f ;
        float epsilon = std::max(0.01f, mean_abs_error * 0.1f);  
        const float tol = 1e-6;  
//        const int maxIter = std::min(5000, static_cast<int>(n * 10000));  
       const int maxIter = 5000;  

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
            if (consecutiveSmallChanges > maxIter) {
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


   // Compute kernel matrix with improved efficiency
    void AdvancedPolynomialFitter::computeKernelMatrix(const std::vector<std::vector<float>>& X, 
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
    std::vector<double> AdvancedPolynomialFitter::computePrimalCoefficients(const std::vector<std::vector<float>>& X, 
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

    

int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b,
    const std::vector<float>& ls_residuals) { // LS residuals included

    // Get error  
    float y2 = y[i2];  
    float F2 = f[i2];  // f already includes Wx part  
    float r2 = F2 + b - y2;  // Residual error  

    // Compute KKT violation magnitude  
    float kkt_magnitude = std::max(0.0f, std::max(r2 - epsilon, -r2 - epsilon));

    // Normalize using LS residuals to assess influence  
    float influence_factor = 1.0f + std::abs(ls_residuals[i2]);  
    kkt_magnitude *= influence_factor;  

    // Check if KKT conditions are violated  
    bool kkt_violated = ((r2 > epsilon && alpha[i2] > 0) || (r2 < -epsilon && alpha[i2 + n] > 0) ||  
                         (r2 < -epsilon && alpha[i2] < C) || (r2 > epsilon && alpha[i2 + n] < C));

    if (kkt_violated) {  
        // Find index with maximum objective function change  
        float max_weighted_delta = 0.0;  
        size_t i1 = i2;  

        // Consider neighboring points within an adaptive window  
//        int neighborhood_size = std::max(1, (int)n / 10);  
        int neighborhood_size = 3;  

        int start_idx = std::max(0, (int)i2 - neighborhood_size);  
        int end_idx = std::min((int)n, (int)i2 + neighborhood_size);  

        // First heuristic - maximize error difference weighted by KKT violation  
        for (size_t j = start_idx; j < end_idx; ++j) {  
            if ((alpha[j] > 0 && alpha[j] < C) || (alpha[j + n] > 0 && alpha[j + n] < C)) {  
                float F1 = f[j];  
                float r1 = F1 + b - y[j];  
                float delta = fabs(r1 - r2);  

                // Adjust weighting based on residuals and KKT magnitude  
                float weight = (1.0f + std::abs(ls_residuals[j])) * kkt_magnitude;  
                delta *= weight;  

                if (delta > max_weighted_delta) {  
                    max_weighted_delta = delta;  
                    i1 = j;  
                }  
            }  
        }  

        // Optimize using the best-found pair  
        if (i1 != i2) {  
            if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {  
                return 1;  
            }  
        }  

        // Second heuristic - random selection from valid points  
        size_t rand_start = rand() % n;  
        for (size_t j = 0; j < n; ++j) {  
            i1 = (rand_start + j) % n;  
            if ((alpha[i1] > 0 && alpha[i1] < C) || (alpha[i1 + n] > 0 && alpha[i1 + n] < C)) {  
                if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {  
                    return 1;  
                }  
            }  
        }  

        // Third heuristic - exhaustive search  
        for (size_t j = 0; j < n; ++j) {  
            if (optimizePair(j, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {  
                return 1;  
            }  
        }  
    }  
    return 0;  
}

int AdvancedPolynomialFitter::optimizePair(
    size_t i1, size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b) {

    if (i1 == i2) return 0;

    // Get errors and kernel values
    float y1 = y[i1];
    float y2 = y[i2];
    float F1 = f[i1];
    float F2 = f[i2];
    float r1 = F1 + b - y1;
    float r2 = F2 + b - y2;


/*
    // Sophisticated kernel caching with thread-local storage
    static thread_local std::unordered_map<size_t, float> kernel_cache;
    
    auto getKernelValue = [&](size_t i, size_t j) -> float {
        if (i > j) std::swap(i, j);
        size_t key = (i << 16) | j;  // Fast hash for pair of indices
        auto it = kernel_cache.find(key);
        if (it != kernel_cache.end()) return it->second;
        float val = K[i][j];
        kernel_cache[key] = val;
        return val;
    };

    
    float k11 = getKernelValue(i1, i1);
    float k12 = getKernelValue(i1, i2);
    float k22 = getKernelValue(i2, i2);
*/

    // standard kernel fetch
    
    float k11 = K[i1][i1];
    float k12 = K[i1][i2];
    float k22 = K[i2][i2];

    // Compute kernel matrix determinant (eta)
    double eta = k11 + k22 - 2 * k12;
    
    // Improved numerical stability check
    if (eta < 1e-12) {
        // Matrix is near-singular, try a different approach or skip
        if (k11 + k22 == 0) return 0;  // Completely degenerate case
        
        // Add a small ridge term to regularize the problem
        eta = std::max(1e-12, eta + 1e-8 * (k11 + k22));
    }

    // Store old alpha values
    double a1p_old = alpha[i1];
    double a1m_old = alpha[i1+n];
    double a2p_old = alpha[i2];
    double a2m_old = alpha[i2+n];

    // Compute net alpha values for the pair
    double s1 = a1p_old - a1m_old;
    double s2 = a2p_old - a2m_old;
    float s_total = s1 + s2;

    // Improved update step calculation using second-order information
    // This incorporates curvature information for faster convergence
    float delta = (r1 - r2) / eta;
    
    // Compute second-order gain estimate for line search
    double expected_gain = (r1 - r2) * delta - 0.5 * eta * delta * delta;
    
    // Only proceed if gain is positive (guarantees objective improvement)
    if (expected_gain < tol) {
        return 0;
    }
    
    float s2_new = s2 + delta;

    // Compute bounds with improved numerical stability
    float L = std::max(-C, s_total - C);
    float H = std::min(C, s_total + C);
    
    // Add small buffer to avoid numerical boundary issues
    const float boundary_buffer = 1e-8 * C;
    L += boundary_buffer;
    H -= boundary_buffer;
    
    if (s2_new < L) s2_new = L;
    if (s2_new > H) s2_new = H;

    // Compute absolute change to track progress
    double absolute_change = fabs(s2_new - s2);
    if (absolute_change < tol) {
        return 0;  // Change too small, skip update
    }

    double s1_new = s_total - s2_new;

    // Convert net values back into separate α⁺ and α⁻
    // Use parameterized decomposition to balance positive/negative components
    float decomposition_param = -1.5;  // Can be tuned or made adaptive
    
    double a1p_new, a1m_new, a2p_new, a2m_new;

    if (s1_new >= 0) {
        a1p_new = s1_new;
        a1m_new = 0;
    } else {
        a1p_new = 0;
        a1m_new = -s1_new;
    }

    if (s2_new >= 0) {
        a2p_new = s2_new;
        a2m_new = 0;
    } else {
        a2p_new = 0;
        a2m_new = -s2_new;
    }

    // Ensure each component is within [0, C] with buffer
    const double max_alpha = C * (1.0 - 1e-10);  // Avoid exact C for numerical stability
    a1p_new = std::min(a1p_new, max_alpha);
    a1m_new = std::min(a1m_new, max_alpha);
    a2p_new = std::min(a2p_new, max_alpha);
    a2m_new = std::min(a2m_new, max_alpha);

    // Early termination if changes are too small
    if ((fabs(a1p_new - a1p_old) < tol && fabs(a1m_new - a1m_old) < tol) &&
        (fabs(a2p_new - a2p_old) < tol && fabs(a2m_new - a2m_old) < tol)) {
        return 0;
    }

    // Update alpha values 
    alpha[i1] = a1p_new;
    alpha[i1+n] = a1m_new;
    alpha[i2] = a2p_new;
    alpha[i2+n] = a2m_new;

    // Calculate changes in net alpha values
    double delta1 = (a1p_new - a1p_old) - (a1m_new - a1m_old);
    double delta2 = (a2p_new - a2p_old) - (a2m_new - a2m_old);

    // Use a more robust bias update approach
    // Track separate bias terms for each constraint type
    std::vector<float> bias_candidates;
    std::vector<float> bias_weights;
    
    auto addBiasCandidate = [&](float b_value, float weight) {
        bias_candidates.push_back(b_value);
        bias_weights.push_back(weight);
    };
    
    // Bias candidates from point 1
    if (a1p_new > 0 && a1p_new < C) {
        addBiasCandidate(y1 - F1 - epsilon - delta1 * k11 - delta2 * k12, 1.0);
    } else if (a1m_new > 0 && a1m_new < C) {
        addBiasCandidate(y1 - F1 + epsilon - delta1 * k11 - delta2 * k12, 1.0);
    }
    
    // Bias candidates from point 2
    if (a2p_new > 0 && a2p_new < C) {
        addBiasCandidate(y2 - F2 - epsilon - delta1 * k12 - delta2 * k22, 1.0);
    } else if (a2m_new > 0 && a2m_new < C) {
        addBiasCandidate(y2 - F2 + epsilon - delta1 * k12 - delta2 * k22, 1.0);
    }
    
    // If we have any valid bias candidates, compute weighted average
    if (!bias_candidates.empty()) {
        double sum_weights = 0.0;
        double weighted_sum = 0.0;
        
        for (size_t i = 0; i < bias_candidates.size(); ++i) {
            weighted_sum += bias_candidates[i] * bias_weights[i];
            sum_weights += bias_weights[i];
        }
        
        b = weighted_sum / sum_weights;
    }
    
    // Use cached kernel values for more efficient function value updates
    const int update_chunk_size = 16;  // Update in chunks for cache efficiency
    for (size_t chunk_start = 0; chunk_start < n; chunk_start += update_chunk_size) {
        size_t chunk_end = std::min(n, chunk_start + update_chunk_size);
        
        for (size_t i = chunk_start; i < chunk_end; ++i) {
            // Prefetch next kernel values if available (compiler optimization hint)
            #ifdef __GNUC__
            if (i + 16 < chunk_end) {
                __builtin_prefetch(&K[i1][i + 16], 0, 1);
                __builtin_prefetch(&K[i2][i + 16], 0, 1);
            }
            #endif
            
            f[i] += delta1 * K[i1][i] + delta2 * K[i2][i];
        }
    }

    return 1;
}

// Helper function to denormalize coefficients
std::vector<double> AdvancedPolynomialFitter::denormalizeCoefficients(
    const std::vector<double>& w, double x_min, double x_range, int degree) {
    
    std::vector<double> coeffs(degree + 1, 0.0);
    
    // Use binomial expansion to compute the denormalized coefficients
    // For a polynomial of form: w0 + w1*x + w2*x^2 + ... + wn*x^n
    // where x is normalized as x = (t - x_min) / x_range
    // and t is the original variable
    
    for (int i = 0; i <= degree; ++i) {
        for (int j = i; j <= degree; ++j) {
            // Coefficient for x^i in the expansion of (t - x_min)^j / x_range^j
            double binCoeff = binomialCoefficient(j, i);
            double term = binCoeff * pow(-x_min, j-i) / pow(x_range, j) * w[j];
            coeffs[i] += term;
        }
    }
    
    return coeffs;
}

// Helper function to calculate binomial coefficient (n choose k)
double AdvancedPolynomialFitter::binomialCoefficient(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    
    double res = 1;
    k = std::min(k, n - k);  // Optimize calculation
    
    for (int i = 0; i < k; ++i) {
        res *= (n - i);
        res /= (i + 1);
    }
    
    return res;
}



// superposition method with householder QR decomposition

 std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method ) {

        const size_t n = x.size();
        const size_t m = degree + 1;

        // Use Eigen or Armadillo for more robust linear algebra if possible
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        // Precompute powers to avoid repeated multiplication
        std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 1; j < m; ++j) {
                xPowers[i][j] = xPowers[i][j-1] * x[i];
            }
        }

        // Compute A^T * A and A^T * y
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += xPowers[i][j] * y[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA[j][k] += xPowers[i][j] * xPowers[i][k];
                }
            }
        }

        // Fill symmetric matrix
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA[j][k] = ATA[k][j];
            }
        }

          std::vector<double> coeffs = solveQR(ATA, ATy);
          // Convert coefficients to float
          std::vector<float> result(coeffs.begin(), coeffs.end());
          return result; 
    }


// QR decomposition solver (previous implementation remains largely the same)
   std::vector<double> AdvancedPolynomialFitter::solveQR(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        const size_t n = A.size();
        const double eps = std::numeric_limits<double>::epsilon();

        // Householder QR decomposition
        for (size_t k = 0; k < n; ++k) {
            // Compute column norm with numerical stability
            double norm_x = 0.0;
            for (size_t i = k; i < n; ++i) {
                norm_x += A[i][k] * A[i][k];
            }
            norm_x = std::sqrt(std::max(norm_x, eps));

            // Avoid potential overflow/underflow
            double alpha = (A[k][k] > 0) ? -norm_x : norm_x;
            double r = std::sqrt(std::max(0.5 * (alpha * alpha - A[k][k] * alpha), eps));

            std::vector<double> v(n, 0.0);
            v[k] = (A[k][k] - alpha) / (2 * r);
            for (size_t i = k + 1; i < n; ++i) {
                v[i] = A[i][k] / (2 * r);
            }

            // Apply Householder transformation to A and b
            for (size_t j = k; j < n; ++j) {
                double dot = 0.0;
                for (size_t i = k; i < n; ++i) {
                    dot += v[i] * A[i][j];
                }
                for (size_t i = k; i < n; ++i) {
                    A[i][j] -= 2 * v[i] * dot;
                }
            }

            double dot_b = 0.0;
            for (size_t i = k; i < n; ++i) {
                dot_b += v[i] * b[i];
            }
            for (size_t i = k; i < n; ++i) {
                b[i] -= 2 * v[i] * dot_b;
            }

            A[k][k] = alpha;
            for (size_t i = k + 1; i < n; ++i) {
                A[i][k] = 0.0;
            }

            
        }

        // Back substitution with added numerical checks
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            // Add a small check to prevent division by near-zero
            x[i] /= (std::abs(A[i][i]) > eps) ? A[i][i] : eps;
        }

        return x;
    }





// Fit segmented polynomials
std::vector<float> AdvancedPolynomialFitter::fitSegmentedPolynomials(const std::vector<float>& x, const std::vector<float>& y, int degree, int segments) {
    std::vector<float> result;
    size_t segmentSize = x.size() / segments;
    
    for (int i = 0; i < segments; ++i) {
        size_t startIdx = i * segmentSize;
        size_t endIdx = (i == segments - 1) ? x.size() : (i + 1) * segmentSize;
        
        std::vector<float> x_segment(x.begin() + startIdx, x.begin() + endIdx);
        std::vector<float> y_segment(y.begin() + startIdx, y.begin() + endIdx);
        
        std::vector<float> segmentCoeffs = fitPolynomial(x_segment, y_segment, degree);
        result.insert(result.end(), segmentCoeffs.begin(), segmentCoeffs.end());
    }
    
    return result;
}

// Implement Levenberg-Marquardt algorithm
std::vector<float> AdvancedPolynomialFitter::levenbergMarquardt(std::vector<float>& coeffs,const std::vector<float>& x, const std::vector<float>& y, int degree) {
    const int maxIterations = 200; // Maximum number of iterations
    const double lambdaInit = 0.1; // Initial lambda value
    const double lambdaFactor = 2; // Factor to increase/decrease lambda
    const double tolerance = 1e-6; // Convergence tolerance

    //std::vector<float> coeffs(degree + 1, 0.0); // Initial coefficients
    double lambda = lambdaInit;
    double prevMSE = calculateMSE(coeffs, x, y);

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Compute the Jacobian matrix and residuals
        std::vector<std::vector<double>> J(x.size(), std::vector<double>(degree + 1, 0.0));
        std::vector<double> residuals(x.size(), 0.0);
        
        for (size_t i = 0; i < x.size(); ++i) {
            double xi = 1.0;
            for (int j = 0; j <= degree; ++j) {
                J[i][j] = xi;
                xi *= x[i];
            }
            double prediction = 0.0;
            for (int j = 0; j <= degree; ++j) {
                prediction += coeffs[j] * pow(x[i], j);
            }
            residuals[i] = y[i] - prediction;
        }

        // Compute the normal equations
        std::vector<std::vector<double>> JTJ(degree + 1, std::vector<double>(degree + 1, 0.0));
        std::vector<double> JTr(degree + 1, 0.0);
        
        for (size_t i = 0; i < x.size(); ++i) {
            for (int j = 0; j <= degree; ++j) {
                for (int k = 0; k <= degree; ++k) {
                    JTJ[j][k] += J[i][j] * J[i][k];
                }
                JTr[j] += J[i][j] * residuals[i];
            }
        }

        // Add the damping factor to the diagonal elements
        for (int j = 0; j <= degree; ++j) {
            JTJ[j][j] += lambda;
        }

        // Solve for the parameter update
        std::vector<double> delta = solveLinearSystem(JTJ, JTr);

        // Update the coefficients
        std::vector<float> newCoeffs = coeffs;
        for (int j = 0; j <= degree; ++j) {
            newCoeffs[j] += delta[j];
        }

        double newMSE = calculateMSE(newCoeffs, x, y);
        
        // Check for convergence
        if (std::abs(prevMSE - newMSE) < tolerance) {
            break;
        }

        // Update lambda and coefficients based on the new MSE
        if (newMSE < prevMSE) {
            lambda /= lambdaFactor;
            Serial.println(lambda);
            coeffs = newCoeffs;
            prevMSE = newMSE;
        } else {
            lambda *= lambdaFactor;
        }
    }

    return coeffs;
}

    // Solve a linear system using Gaussian elimination
    std::vector<double> AdvancedPolynomialFitter::solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b) {
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
