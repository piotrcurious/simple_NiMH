// Helper function for examining training examples in SMO
int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b,
    const std::vector<double>& ls_residuals, 
    const std::vector<bool>& active_set,  // Added active set tracking
    int& unchanged_count) {               // Added convergence tracking

    // Skip inactive examples
    if (!active_set[i2]) return 0;

    // Get error
    double y2 = y[i2];
    double F2 = f[i2];
    double r2 = F2 + b - y2;

    // Improved KKT condition checking with numerical tolerance
    const double kkt_tol = 1e-5;  // Tolerance for KKT conditions
    bool upper_violated = (r2 > epsilon + kkt_tol && alpha[i2] > 0) || 
                          (r2 < -epsilon - kkt_tol && alpha[i2+n] > 0);
    bool lower_violated = (r2 < -epsilon - kkt_tol && alpha[i2] < C) || 
                          (r2 > epsilon + kkt_tol && alpha[i2+n] < C);
    bool kkt_violated = upper_violated || lower_violated;

    if (kkt_violated) {
        // Find index with maximum objective function change
        double max_delta = 0.0;
        size_t i1 = n;  // Invalid index to detect if we found a good candidate
        
        // Cache the normalized residual for the current point
        static std::vector<double> normalized_residuals;
        static bool residuals_normalized = false;
        
        // Compute normalized residuals once and cache them
        if (!residuals_normalized) {
            double max_res = 1e-10;  // Small epsilon to avoid division by zero
            for (const auto& res : ls_residuals) {
                max_res = std::max(max_res, std::abs(res));
            }
            
            normalized_residuals.resize(ls_residuals.size());
            for (size_t i = 0; i < ls_residuals.size(); ++i) {
                normalized_residuals[i] = std::abs(ls_residuals[i]) / max_res;
            }
            residuals_normalized = true;
        }
        
        // Pre-compute common terms for current example
        double norm_r_i2 = normalized_residuals[i2];
        
        // First heuristic - Most Violating Pair (MVP)
        for (size_t j = 0; j < n; ++j) {
            if (!active_set[j]) continue;
            
            // Check if j is a non-bound support vector (more likely to change)
            bool is_free_sv = (alpha[j] > 0 && alpha[j] < C) || (alpha[j+n] > 0 && alpha[j+n] < C);
            
            if (is_free_sv) {
                double F1 = f[j];
                double r1 = F1 + b - y[j];
                
                // Basic error difference
                double delta = fabs(r1 - r2);
                
                // Advanced weighting strategy
                double norm_r_j = normalized_residuals[j];
                
                // Use sigmoid function to enhance contrast between high and low residuals
                double sigmoid_i2 = 2.0 / (1.0 + exp(-5.0 * norm_r_i2)) - 1.0;
                double sigmoid_j = 2.0 / (1.0 + exp(-5.0 * norm_r_j)) - 1.0;
                
                // Compute weighted delta - emphasize pairs with complementary errors
                double residual_weight = 1.0 + sigmoid_i2 + sigmoid_j;
                double error_correlation = r1 * r2 < 0 ? 1.5 : 1.0;  // Prioritize opposite signs
                
                delta *= residual_weight * error_correlation;
                
                if (delta > max_delta) {
                    max_delta = delta;
                    i1 = j;
                }
            }
        }

        // If we found a viable partner using the first heuristic
        if (i1 != n) {
            if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                unchanged_count = 0;  // Reset counter since we made progress
                return 1;
            }
        }

        // Second heuristic - Try recently successful examples first
        const int recent_cache_size = std::min(20, static_cast<int>(n/10 + 1));
        static std::deque<size_t> recent_successful;
        
        for (const auto& i1 : recent_successful) {
            if (i1 != i2 && active_set[i1]) {
                if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                    // Move this index to the front (most recent)
                    auto it = std::find(recent_successful.begin(), recent_successful.end(), i1);
                    if (it != recent_successful.end()) {
                        recent_successful.erase(it);
                    }
                    recent_successful.push_front(i1);
                    
                    unchanged_count = 0;
                    return 1;
                }
            }
        }

        // Third heuristic - Stratified random search
        // First try SVs close to bounds (likely to move)
        size_t rand_start = rand() % n;
        for (size_t j = 0; j < n; ++j) {
            i1 = (rand_start + j) % n;
            if (!active_set[i1]) continue;
            
            bool near_bound = (alpha[i1] > 0 && alpha[i1] < 0.1*C) || 
                             (alpha[i1] > 0.9*C && alpha[i1] < C) ||
                             (alpha[i1+n] > 0 && alpha[i1+n] < 0.1*C) || 
                             (alpha[i1+n] > 0.9*C && alpha[i1+n] < C);
                             
            if (near_bound) {
                if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                    // Add to recent successful list
                    recent_successful.push_front(i1);
                    if (recent_successful.size() > recent_cache_size) {
                        recent_successful.pop_back();
                    }
                    
                    unchanged_count = 0;
                    return 1;
                }
            }
        }

        // Fourth heuristic - Complete scan over all active examples
        rand_start = rand() % n;
        for (size_t j = 0; j < n; ++j) {
            i1 = (rand_start + j) % n;
            if (!active_set[i1]) continue;
            
            if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                // Add to recent successful list
                recent_successful.push_front(i1);
                if (recent_successful.size() > recent_cache_size) {
                    recent_successful.pop_back();
                }
                
                unchanged_count = 0;
                return 1;
            }
        }
    }

    // No progress made
    unchanged_count++;
    return 0;
}
