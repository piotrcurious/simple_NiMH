int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b,
    const std::vector<float>& ls_residuals) {

    // Get error  
    float y2 = y[i2];  
    float F2 = f[i2];  
    float r2 = F2 + b - y2;  // Residual error  

    // First improvement: Better KKT violation detection with adaptive tolerance
    // Calculate adaptive tolerance based on sample position and least squares residual
    float adaptive_tol = tol * (1.0f + 0.5f * std::abs(ls_residuals[i2]));
    
    // More robust KKT condition checking with numerical safeguards
    bool upper_violated = (r2 > epsilon + adaptive_tol) && (alpha[i2] < C - adaptive_tol);
    bool lower_violated = (r2 < -epsilon - adaptive_tol) && (alpha[i2 + n] < C - adaptive_tol);
    bool upper_active_violated = (r2 < -epsilon + adaptive_tol) && (alpha[i2] > adaptive_tol);
    bool lower_active_violated = (r2 > epsilon - adaptive_tol) && (alpha[i2 + n] > adaptive_tol);
    
    bool kkt_violated = upper_violated || lower_violated || upper_active_violated || lower_active_violated;

    // Second improvement: Calculate KKT violation score more robustly
    float violation_score = 0.0f;
    if (upper_violated) {
        violation_score = (r2 - epsilon) * (C - alpha[i2]);
    } else if (lower_violated) {
        violation_score = (-r2 - epsilon) * (C - alpha[i2 + n]);
    } else if (upper_active_violated) {
        violation_score = (-r2 - epsilon) * alpha[i2];
    } else if (lower_active_violated) {
        violation_score = (r2 - epsilon) * alpha[i2 + n];
    }
    
    // Scale violation score by residual importance
    // Points with higher LS residuals often need more attention
    float residual_importance = std::pow(1.0f + std::abs(ls_residuals[i2]), 1.5f);
    violation_score *= residual_importance;

    if (kkt_violated && violation_score > adaptive_tol) {  
        // Third improvement: More sophisticated working set selection
        size_t i1 = n;  // Invalid initial value to detect if a valid index was found
        float best_score = -1.0f;
        
        // Multi-stage working set selection strategy
        
        // Stage 1: Look for maximum violation pair with maximum expected gain
        int neighborhood_size = std::max(2, std::min(20, (int)(n / 10)));
        int window_start = std::max(0, (int)i2 - neighborhood_size);
        int window_end = std::min((int)n - 1, (int)i2 + neighborhood_size);
        
        for (int j = window_start; j <= window_end; ++j) {
            if (j == i2) continue;  // Skip self
            
            // Check if this is a valid working variable
            bool is_free = (alpha[j] > adaptive_tol && alpha[j] < C - adaptive_tol) || 
                          (alpha[j + n] > adaptive_tol && alpha[j + n] < C - adaptive_tol);
            
            if (is_free) {
                // Calculate error for point j
                float F1 = f[j];
                float r1 = F1 + b - y[j];
                
                // Calculate expected gain using second-order information
                float k11 = K[j][j];
                float k22 = K[i2][i2];
                float k12 = K[j][i2];
                float eta = k11 + k22 - 2 * k12;
                
                // Add small ridge to prevent numerical issues
                eta = std::max(1e-6f, eta);
                
                // Calculate projected gain based on KKT violation and kernel properties
                float delta_r = r1 - r2;
                float expected_delta = delta_r / eta;
                
                // Clip expected delta to feasible range based on current alphas
                float s1 = alpha[j] - alpha[j + n];
                float s2 = alpha[i2] - alpha[i2 + n];
                float s_sum = s1 + s2;
                
                float L = std::max(-C, s_sum - C);
                float H = std::min(C, s_sum + C);
                float clipped_delta = std::max(L - s2, std::min(H - s2, expected_delta));
                
                // Calculate expected gain after clipping
                float gain = std::abs(delta_r * clipped_delta - 0.5f * eta * clipped_delta * clipped_delta);
                
                // Include residual importance for point j
                float j_importance = std::pow(1.0f + std::abs(ls_residuals[j]), 1.2f);
                gain *= j_importance;
                
                if (gain > best_score) {
                    best_score = gain;
                    i1 = j;
                }
            }
        }
        
        // Stage 2: If no good point found in neighborhood, try points with large KKT violations
        if (i1 == n || best_score < adaptive_tol) {
            // Calculate KKT violations for all points and sort
            std::vector<std::pair<float, size_t>> violations;
            for (size_t j = 0; j < n; ++j) {
                if (j == i2) continue;
                
                float F1 = f[j];
                float r1 = F1 + b - y[j];
                
                // Calculate violation score similar to above
                float j_violation = 0.0f;
                if ((r1 > epsilon) && (alpha[j] < C)) {
                    j_violation = (r1 - epsilon) * (C - alpha[j]);
                } else if ((r1 < -epsilon) && (alpha[j + n] < C)) {
                    j_violation = (-r1 - epsilon) * (C - alpha[j + n]);
                } else if ((r1 < -epsilon) && (alpha[j] > 0)) {
                    j_violation = (-r1 - epsilon) * alpha[j];
                } else if ((r1 > epsilon) && (alpha[j + n] > 0)) {
                    j_violation = (r1 - epsilon) * alpha[j + n];
                }
                
                j_violation *= std::pow(1.0f + std::abs(ls_residuals[j]), 1.2f);
                
                if (j_violation > adaptive_tol) {
                    violations.push_back({j_violation, j});
                }
            }
            
            // Sort by violation score, descending
            std::sort(violations.begin(), violations.end(), 
                     [](const std::pair<float, size_t>& a, const std::pair<float, size_t>& b) {
                         return a.first > b.first;
                     });
            
            // Try top violation candidates (limit to prevent excessive attempts)
            const int max_candidates = std::min(10, (int)violations.size());
            for (int idx = 0; idx < max_candidates; ++idx) {
                size_t j = violations[idx].second;
                if (optimizePair(j, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                    return 1;
                }
            }
        }
        
        // Stage 3: If we found a good candidate in stage 1, try to optimize with it
        if (i1 < n) {  // Valid index found
            if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                return 1;
            }
        }
        
        // Stage 4: Fallback to scanning SVs and then random selection
        // First try support vectors (points with non-zero alpha)
        for (size_t j = 0; j < n; ++j) {
            if (j == i2) continue;
            if ((alpha[j] > 0 || alpha[j + n] > 0) && 
                optimizePair(j, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                return 1;
            }
        }
        
        // Finally, try random selection with adaptive repetition
        int max_random_attempts = std::min(20, (int)std::sqrt(n));
        for (int attempt = 0; attempt < max_random_attempts; ++attempt) {
            size_t j = rand() % n;
            if (j != i2 && optimizePair(j, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                return 1;
            }
        }
    }
    
    return 0;  
}
