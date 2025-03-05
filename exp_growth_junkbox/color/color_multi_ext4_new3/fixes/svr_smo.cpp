std::vector<float> AdvancedPolynomialFitter::fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree,
                                                    OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]);
    Serial.println(x[x.size() - 1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Normalize input data to improve numerical stability
    float x_min = *std::min_element(x.begin(), x.end());
    float x_max = *std::max_element(x.begin(), x.end());
    float x_range = x_max - x_min;

    std::vector<float> x_norm(n);
    for (size_t i = 0; i < n; ++i) {
        x_norm[i] = (x[i] - x_min) / x_range;
    }

    // Create feature matrix (Vandermonde matrix)
    std::vector<std::vector<float>> X(n, std::vector<float>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        X[i][0] = 1.0;  // Constant term
        for (size_t j = 1; j < m; ++j) {
            X[i][j] = pow(x_norm[i], j);  // Use normalized x values
        }
    }

    // Calculate LS residuals to guide SMO heuristics - NEW
    std::vector<float> ls_coeffs = fitPolynomial_superpos5c(x_norm, y, degree, OptimizationMethod::NONE);
//    std::vector<float> ls_coeffs = fitPolynomial(x, y, degree, OptimizationMethod::NONE);

    std::vector<float> ls_residuals(n);
    for (size_t i = 0; i < n; ++i) {
        float prediction = 0.0;
        for (size_t j = 0; j < ls_coeffs.size(); ++j) {
            prediction += ls_coeffs[j] * pow(x[i], j);
        }
        ls_residuals[i] = prediction - y[i];
    }

    // Hyperparameters - adjusted for better fit
    const float C = 100000.0;
    const float epsilon = 0.1;
    const float tol = 1e-6;
    const int maxIter = 5000;

    // Initialize alphas and function values
    std::vector<float> alpha(2 * n, 0.0);
    std::vector<float> f(n, 0.0);
    double b = 0.0;

    // Compute kernel matrix for linear case (dot products of feature vectors)
    std::vector<std::vector<float>> K(n, std::vector<float>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i; j < n; ++j) {  // Only compute upper triangle due to symmetry
            float dot = 0.0;
            for (size_t k = 0; k < m; ++k) {
                dot += X[i][k] * X[j][k];
            }
            K[i][j] = dot;
            if (i != j) {
                K[j][i] = dot;  // Fill lower triangle
            }
        }
    }

    // SMO Algorithm for SVR
    int iter = 0;
    int numChanged = 0;
    bool examineAll = true;

    while ((numChanged > 0 || examineAll) && iter < maxIter) {
        numChanged = 0;

        if (examineAll) {
            // Loop over all training examples
            for (size_t i = 0; i < n; ++i) {
                numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, ls_residuals);  // Pass LS residuals
            }
        } else {
            // Loop over examples where alpha is not at bounds
            for (size_t i = 0; i < n; ++i) {
                if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i + n] > 0 && alpha[i + n] < C)) {
                    numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, ls_residuals);  // Pass LS residuals
                }
            }
        }

        if (examineAll) {
            examineAll = false;
        } else if (numChanged == 0) {
            examineAll = true;
        }

        iter++;

        // Debug progress
        if (iter % 10 == 0) {
            Serial.print("Iteration: ");
            Serial.print(iter);
            Serial.print(", Changed: ");
            Serial.println(numChanged);
        }
    }

    // Calculate polynomial coefficients directly from the primal form
    std::vector<double> w(m, 0.0);

    // For each dimension in feature space
    for (size_t j = 0; j < m; ++j) {
        for (size_t i = 0; i < n; ++i) {
            // alpha[i] is alpha+, alpha[i+n] is alpha-
            double alpha_diff = alpha[i] - alpha[i + n];
            w[j] += alpha_diff * X[i][j];
        }
    }

    
    // Apply denormalization to coefficients
    std::vector<double> denorm_coeffs = denormalizeCoefficients(w, x_min, x_range, degree);
    
    // Convert to float
    std::vector<float> result(denorm_coeffs.begin(), denorm_coeffs.end());
    
    // Debug output
    Serial.print("SVM Coefficients: ");
    for (size_t i = 0; i < result.size(); ++i) {
        Serial.print(result[i]); Serial.print(" ");
    }
    Serial.println();
    
 /*
    // Calculate and print RMSE
    double rmse = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double pred = 0.0;
        for (size_t j = 0; j < m; ++j) {
            pred += result[j] * pow(x[i], j);
        }
        double err = pred - y[i];
        rmse += err * err;
    }
    rmse = sqrt(rmse / n);
    Serial.print("RMSE: ");
    Serial.println(rmse);
 */
    
    return result;
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
        int neighborhood_size = std::max(1, (int)n / 10);  
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
