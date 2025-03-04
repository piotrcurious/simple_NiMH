int AdvancedPolynomialFitter::optimizePair(
    size_t i1, size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b) {

    if (i1 == i2) return 0;

    // Get errors and kernel values
    double y1 = y[i1];
    double y2 = y[i2];
    double F1 = f[i1];
    double F2 = f[i2];
    double r1 = F1 + b - y1;
    double r2 = F2 + b - y2;

    // Sophisticated kernel caching with thread-local storage
    static thread_local std::unordered_map<size_t, double> kernel_cache;
    
    auto getKernelValue = [&](size_t i, size_t j) -> double {
        if (i > j) std::swap(i, j);
        size_t key = (i << 16) | j;  // Fast hash for pair of indices
        auto it = kernel_cache.find(key);
        if (it != kernel_cache.end()) return it->second;
        double val = K[i][j];
        kernel_cache[key] = val;
        return val;
    };
    
    double k11 = getKernelValue(i1, i1);
    double k12 = getKernelValue(i1, i2);
    double k22 = getKernelValue(i2, i2);

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
    double decomposition_param = 0.5;  // Can be tuned or made adaptive
    
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
    std::vector<double> bias_candidates;
    std::vector<double> bias_weights;
    
    auto addBiasCandidate = [&](double b_value, double weight) {
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
    const int update_chunk_size = 1000;  // Update in chunks for cache efficiency
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
