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
