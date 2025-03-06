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

    // NEW: Analyze data distribution to identify potential problem regions
    std::vector<float> point_density(n-1);
    for (size_t i = 0; i < n-1; ++i) {
        point_density[i] = x_norm[i+1] - x_norm[i];
    }
    float median_density = computeMedian(point_density);
    float density_std = computeStdDev(point_density, median_density);
    
    // Create feature matrix (Vandermonde matrix) with better conditioning
    std::vector<std::vector<float>> X(n, std::vector<float>(m, 0.0));
    
    // NEW: Use orthogonal polynomial basis instead of monomial basis
    // This significantly improves conditioning of the Vandermonde matrix
    for (size_t i = 0; i < n; ++i) {
        X[i][0] = 1.0;  // Constant term
        if (m > 1) X[i][1] = x_norm[i];  // Linear term
        
        // Generate higher degree terms using recurrence relation for Chebyshev polynomials
        for (size_t j = 2; j < m; ++j) {
            X[i][j] = 2 * x_norm[i] * X[i][j-1] - X[i][j-2];
        }
    }

    // Calculate LS residuals using orthogonal basis for better stability
    std::vector<float> ls_coeffs = fitPolynomial_superpos5c(x_norm, y, degree, OptimizationMethod::NONE);

    // NEW: Compute weighted residuals based on singular value analysis
    std::vector<float> singular_values = computeSVD(X);
    float condition_number = singular_values[0] / singular_values[singular_values.size() - 1];
    
    std::vector<float> ls_residuals(n);
    std::vector<float> weighted_residuals(n);
    
    for (size_t i = 0; i < n; ++i) {
        float prediction = 0.0;
        for (size_t j = 0; j < ls_coeffs.size(); ++j) {
            prediction += ls_coeffs[j] * X[i][j];  // Use orthogonal basis
        }
        ls_residuals[i] = prediction - y[i];
        
        // NEW: Weight residuals by local sensitivity
        weighted_residuals[i] = ls_residuals[i] * computeLocalSensitivity(x_norm, i, median_density);
    }

    // Hyperparameters - adaptively tuned based on data characteristics
    const float C = 100000.0 * (1.0f / (1.0f + log(condition_number)));  // Adjust C based on conditioning
    const float epsilon = 0.1 * computeRobustScale(y);  // Adapt epsilon to data scale
    const float tol = 1e-6 * (1.0f + 0.1f * log(condition_number));  // Adaptive tolerance
    const int maxIter = std::min(5000, static_cast<int>(50 * degree * log(n)));  // Adaptive iterations

    // Initialize alphas and function values
    std::vector<float> alpha(2 * n, 0.0);
    std::vector<float> f(n, 0.0);
    double b = 0.0;

    // NEW: Use adaptive kernel based on data distribution
    std::vector<std::vector<float>> K(n, std::vector<float>(n, 0.0));
    computeAdaptiveKernel(K, X, n, m, point_density, median_density);

    // IMPROVED: Track global KKT violation patterns for geometric inference
    std::vector<float> kkt_history(n, 0.0);
    std::vector<float> kkt_gradients(n, 0.0);
    
    // NEW: Adaptive SMO with geometric inference
    int iter = 0;
    int numChanged = 0;
    bool examineAll = true;
    int plateau_counter = 0;
    float previous_violation_sum = std::numeric_limits<float>::max();

    while ((numChanged > 0 || examineAll) && iter < maxIter) {
        numChanged = 0;
        float current_violation_sum = 0.0;

        // NEW: Analyze KKT violation patterns
        updateKKTViolationPatterns(n, alpha, y, f, epsilon, C, b, kkt_history, kkt_gradients);
        
        // NEW: Detect polynomial singularities based on KKT patterns
        std::vector<size_t> critical_points = detectCriticalPoints(kkt_gradients, n);
        
        if (examineAll) {
            // NEW: Prioritize training examples with highest geometric influence
            std::vector<std::pair<float, size_t>> influence_pairs;
            for (size_t i = 0; i < n; ++i) {
                float influence = computeGeometricInfluence(i, n, kkt_history, kkt_gradients, critical_points);
                influence_pairs.push_back({influence, i});
                current_violation_sum += fabs(kkt_history[i]);
            }
            
            // Sort by influence in descending order
            std::sort(influence_pairs.begin(), influence_pairs.end(), 
                     [](const auto& a, const auto& b) { return a.first > b.first; });
            
            // Process in order of influence
            for (const auto& pair : influence_pairs) {
                size_t i = pair.second;
                numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, weighted_residuals, 
                                           kkt_history, kkt_gradients, critical_points);
            }
        } else {
            // Focus on examples with non-bound alphas and high geometric influence
            std::vector<std::pair<float, size_t>> non_bound_pairs;
            for (size_t i = 0; i < n; ++i) {
                if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i + n] > 0 && alpha[i + n] < C)) {
                    float influence = computeGeometricInfluence(i, n, kkt_history, kkt_gradients, critical_points);
                    non_bound_pairs.push_back({influence, i});
                    current_violation_sum += fabs(kkt_history[i]);
                }
            }
            
            // Sort by influence in descending order
            std::sort(non_bound_pairs.begin(), non_bound_pairs.end(), 
                     [](const auto& a, const auto& b) { return a.first > b.first; });
            
            // Process in order of influence
            for (const auto& pair : non_bound_pairs) {
                size_t i = pair.second;
                numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, weighted_residuals,
                                           kkt_history, kkt_gradients, critical_points);
            }
        }

        // NEW: Adaptive search strategy based on convergence behavior
        if (examineAll) {
            examineAll = false;
        } else if (numChanged == 0) {
            examineAll = true;
        }
        
        // NEW: Check for convergence plateaus
        float violation_change = previous_violation_sum - current_violation_sum;
        if (fabs(violation_change) < tol * previous_violation_sum) {
            plateau_counter++;
            
            // If stuck in plateau, apply algebraic correction
            if (plateau_counter > 5) {
                applyAlgebraicCorrection(n, m, alpha, X, y, f, b);
                plateau_counter = 0;
            }
        } else {
            plateau_counter = 0;
        }
        
        previous_violation_sum = current_violation_sum;
        iter++;

        // Debug progress
        if (iter % 10 == 0) {
            Serial.print("Iteration: ");
            Serial.print(iter);
            Serial.print(", Changed: ");
            Serial.print(numChanged);
            Serial.print(", Violation sum: ");
            Serial.println(current_violation_sum);
        }
    }

    // Calculate polynomial coefficients from the dual form with orthogonal basis
    std::vector<double> w(m, 0.0);

    for (size_t j = 0; j < m; ++j) {
        for (size_t i = 0; i < n; ++i) {
            double alpha_diff = alpha[i] - alpha[i + n];
            w[j] += alpha_diff * X[i][j];
        }
    }

    // NEW: Convert from orthogonal basis back to standard basis
    std::vector<double> monomial_coeffs = convertOrthogonalToMonomial(w, degree);
    
    // Apply denormalization to coefficients
    std::vector<double> denorm_coeffs = denormalizeCoefficients(monomial_coeffs, x_min, x_range, degree);
    
    // Convert to float
    std::vector<float> result(denorm_coeffs.begin(), denorm_coeffs.end());
    
    // Debug output
    Serial.print("SVM Coefficients: ");
    for (size_t i = 0; i < result.size(); ++i) {
        Serial.print(result[i]); Serial.print(" ");
    }
    Serial.println();
    
    return result;
}


int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b,
    const std::vector<float>& weighted_residuals,
    const std::vector<float>& kkt_history,
    const std::vector<float>& kkt_gradients,
    const std::vector<size_t>& critical_points) {

    // Get error  
    float y2 = y[i2];  
    float F2 = f[i2];  
    float r2 = F2 + b - y2;  // Residual error  

    // NEW: Compute KKT violation with geometric context
    float kkt_magnitude = std::max(0.0f, std::max(r2 - epsilon, -r2 - epsilon));
    
    // NEW: Adjust influence by geometric factors
    float influence_factor = 1.0f + std::abs(weighted_residuals[i2]);
    
    // NEW: Add critical point importance
    bool is_critical = std::find(critical_points.begin(), critical_points.end(), i2) != critical_points.end();
    if (is_critical) {
        influence_factor *= 2.0f;  // Prioritize critical points
    }
    
    // NEW: Consider KKT violation trends
    if (i2 > 0 && i2 < n-1) {
        float local_curvature = fabs(kkt_history[i2-1] - 2*kkt_history[i2] + kkt_history[i2+1]);
        influence_factor *= (1.0f + local_curvature);
    }
    
    kkt_magnitude *= influence_factor;

    // Check if KKT conditions are violated  
    bool kkt_violated = ((r2 > epsilon && alpha[i2] > 0) || (r2 < -epsilon && alpha[i2 + n] > 0) ||  
                         (r2 < -epsilon && alpha[i2] < C) || (r2 > epsilon && alpha[i2 + n] < C));

    if (kkt_violated) {  
        // Find index with maximum objective function change  
        float max_weighted_delta = 0.0;  
        size_t i1 = i2;  

        // NEW: Adaptively choose search neighborhood based on KKT patterns
        int neighborhood_size = determineAdaptiveNeighborhood(i2, n, kkt_gradients, critical_points);
        int start_idx = std::max(0, (int)i2 - neighborhood_size);  
        int end_idx = std::min((int)n, (int)i2 + neighborhood_size);  

        // IMPROVED: First heuristic - use algebraic geometry to find optimal pairs
        for (size_t j = start_idx; j < end_idx; ++j) {  
            if ((alpha[j] > 0 && alpha[j] < C) || (alpha[j + n] > 0 && alpha[j + n] < C)) {  
                float F1 = f[j];  
                float r1 = F1 + b - y[j];  
                
                // NEW: Compute algebraic complementarity score
                float algebraic_score = computeAlgebraicComplementarity(j, i2, kkt_history, kkt_gradients);
                float delta = fabs(r1 - r2) * algebraic_score;
                
                // NEW: Adjust by polynomial discriminant if at critical points
                if (is_critical || std::find(critical_points.begin(), critical_points.end(), j) != critical_points.end()) {
                    delta *= 1.5f;
                }

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

        // NEW: Geometric-aware second heuristic
        // Target points that lie on opposite sides of critical regions
        if (!critical_points.empty()) {
            for (size_t cp : critical_points) {
                // Find points on opposite sides of this critical point
                if (i2 < cp) {
                    // Look for points beyond the critical point
                    size_t search_start = cp + 1;
                    size_t search_end = std::min(n, cp + neighborhood_size);
                    for (size_t j = search_start; j < search_end; ++j) {
                        if (optimizePair(j, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                            return 1;
                        }
                    }
                } else if (i2 > cp) {
                    // Look for points before the critical point
                    size_t search_start = (cp > neighborhood_size) ? cp - neighborhood_size : 0;
                    for (size_t j = search_start; j < cp; ++j) {
                        if (optimizePair(j, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                            return 1;
                        }
                    }
                }
            }
        }

        // Third heuristic - strategic sampling with algebraic awareness
        const int max_samples = 20;  // Limit number of attempts
        std::vector<size_t> strategic_indices = selectStrategicIndices(i2, n, max_samples, kkt_history, kkt_gradients);
        
        for (size_t j : strategic_indices) {
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
    
    float k11 = K[i1][i1];
    float k12 = K[i1][i2];
    float k22 = K[i2][i2];

    // Compute kernel matrix determinant (eta)
    double eta = k11 + k22 - 2 * k12;
    
    // IMPROVED: Better numerical stability check with adaptive regularization
    if (eta < 1e-12) {
        // Analyze local geometry to determine appropriate regularization
        double local_curvature = estimateLocalCurvature(i1, i2, K, n);
        double regularization_term = std::max(1e-12, 1e-8 * (k11 + k22) * (1.0 + local_curvature));
        eta = std::max(1e-12, eta + regularization_term);
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

    // IMPROVED: Newton step with line search for better convergence
    float delta_init = (r1 - r2) / eta;
    
    // NEW: Implement cubic line search for optimal step size
    float delta = findOptimalStepSize(delta_init, r1, r2, eta, s2, s_total, C);
    
    // Compute second-order gain estimate
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

    // NEW: Optimized alpha decomposition strategy
    // Use a geometrically-aware approach to balance positive/negative components
    double a1p_new, a1m_new, a2p_new, a2m_new;
    
    // Analyze KKT conditions to guide decomposition
    optimizeAlphaDecomposition(s1_new, s2_new, r1, r2, epsilon, 
                              a1p_new, a1m_new, a2p_new, a2m_new);

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

    // IMPROVED: Bias update with geometric awareness
    // Use a robust geometric median approach for bias estimation
    std::vector<float> bias_candidates;
    std::vector<float> bias_weights;
    
    auto addBiasCandidate = [&](float b_value, float weight) {
        bias_candidates.push_back(b_value);
        bias_weights.push_back(weight);
    };
    
    // Bias candidates from KKT conditions
    if (a1p_new > 0 && a1p_new < C) {
        float b_cand = y1 - F1 - epsilon - delta1 * k11 - delta2 * k12;
        float weight = 1.0f + fabs(a1p_new - 0.5f * C) / (0.5f * C);  // Higher weight near center
        addBiasCandidate(b_cand, weight);
    } else if (a1m_new > 0 && a1m_new < C) {
        float b_cand = y1 - F1 + epsilon - delta1 * k11 - delta2 * k12;
        float weight = 1.0f + fabs(a1m_new - 0.5f * C) / (0.5f * C);
        addBiasCandidate(b_cand, weight);
    }
    
    if (a2p_new > 0 && a2p_new < C) {
        float b_cand = y2 - F2 - epsilon - delta1 * k12 - delta2 * k22;
        float weight = 1.0f + fabs(a2p_new - 0.5f * C) / (0.5f * C);
        addBiasCandidate(b_cand, weight);
    } else if (a2m_new > 0 && a2m_new < C) {
        float b_cand = y2 - F2 + epsilon - delta1 * k12 - delta2 * k22;
        float weight = 1.0f + fabs(a2m_new - 0.5f * C) / (0.5f * C);
        addBiasCandidate(b_cand, weight);
    }
    
    // If we have any valid bias candidates, compute weighted geometric median
    if (!bias_candidates.empty()) {
        b = computeGeometricMedian(bias_candidates, bias_weights);
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

// Compute median of a vector
float AdvancedPolynomialFitter::computeMedian(const std::vector<float>& values) {
    if (values.empty()) return 0.0f;
    
    std::vector<float> sorted_values = values;
    std::sort(sorted_values.begin(), sorted_values.end());
    
    size_t n = sorted_values.size();
    if (n % 2 == 0) {
        return (sorted_values[n/2 - 1] + sorted_values[n/2]) / 2.0f;
    } else {
        return sorted_values[n/2];
    }
}

// Compute standard deviation
float AdvancedPolynomialFitter::computeStdDev(const std::vector<float>& values, float mean) {
    if (values.empty()) return 0.0f;
    
    float variance = 0.0f;
    for (float v : values) {
        float diff = v - mean;
        variance += diff * diff;
    }
    
    return std::sqrt(variance / values.size());
}

// Compute robust scale estimate (median absolute deviation)
float AdvancedPolynomialFitter::computeRobustScale(const std::vector<float>& values) {
    if (values.empty()) return 0.0f;
    
    float median = computeMedian(values);
    
    std::vector<float> abs_deviations;
    abs_deviations.reserve(values.size());
    
    for (float v : values) {
        abs_deviations.push_back(std::fabs(v - median));
    }
    
    return 1.4826f * computeMedian(abs_deviations); // Consistency factor for normal distribution
}

// Compute local sensitivity based on point density
float AdvancedPolynomialFitter::computeLocalSensitivity(const std::vector<float>& x_norm, 
                                                       size_t index, float median_density) {
    if (x_norm.empty() || index >= x_norm.size()) return 1.0f;
    
    // Compute local density ratio
    float local_density;
    if (index == 0) {
        local_density = x_norm[1] - x_norm[0];
    } else if (index == x_norm.size() - 1) {
        local_density = x_norm[index] - x_norm[index - 1];
    } else {
        local_density = (x_norm[index + 1] - x_norm[index - 1]) / 2.0f;
    }
    
    // Higher weight for sparse regions
    float density_ratio = median_density / (local_density + 1e-6f);
    
    // Limit the maximum weight to prevent extreme values
    return std::min(5.0f, std::max(0.2f, density_ratio));
}

// Compute SVD of feature matrix (simplified version)
std::vector<float> AdvancedPolynomialFitter::computeSVD(const std::vector<std::vector<float>>& X) {
    size_t n = X.size();
    size_t m = X[0].size();
    
    // Simple power iteration method to estimate singular values
    std::vector<float> singular_values(m, 0.0f);
    std::vector<std::vector<float>> U = X; // Work with a copy
    
    for (size_t i = 0; i < m; ++i) {
        // Initialize random vector
        std::vector<float> v(m, 0.0f);
        v[i] = 1.0f; // Simple initialization
        
        // Power iteration (simplified)
        for (int iter = 0; iter < 10; ++iter) {
            // Compute U*v
            std::vector<float> Uv(n, 0.0f);
            for (size_t j = 0; j < n; ++j) {
                for (size_t k = 0; k < m; ++k) {
                    Uv[j] += U[j][k] * v[k];
                }
            }
            
            // Compute U^T * (U*v)
            std::vector<float> UTUv(m, 0.0f);
            for (size_t j = 0; j < m; ++j) {
                for (size_t k = 0; k < n; ++k) {
                    UTUv[j] += U[k][j] * Uv[k];
                }
            }
            
            // Normalize
            float norm = 0.0f;
            for (float val : UTUv) {
                norm += val * val;
            }
            norm = std::sqrt(norm);
            
            if (norm < 1e-10f) break;
            
            for (size_t j = 0; j < m; ++j) {
                v[j] = UTUv[j] / norm;
            }
        }
        
        // Compute singular value
        std::vector<float> Uv(n, 0.0f);
        for (size_t j = 0; j < n; ++j) {
            for (size_t k = 0; k < m; ++k) {
                Uv[j] += U[j][k] * v[k];
            }
        }
        
        float sv_squared = 0.0f;
        for (float val : Uv) {
            sv_squared += val * val;
        }
        
        singular_values[i] = std::sqrt(sv_squared);
        
        // Deflation (remove component from U)
        for (size_t j = 0; j < n; ++j) {
            for (size_t k = 0; k < m; ++k) {
                U[j][k] -= singular_values[i] * Uv[j] * v[k] / sv_squared;
            }
        }
    }
    
    // Sort singular values in descending order
    std::sort(singular_values.begin(), singular_values.end(), std::greater<float>());
    
    return singular_values;
}

// Compute adaptive kernel based on data distribution
void AdvancedPolynomialFitter::computeAdaptiveKernel(std::vector<std::vector<float>>& K,
                                                   const std::vector<std::vector<float>>& X,
                                                   size_t n, size_t m,
                                                   const std::vector<float>& point_density,
                                                   float median_density) {
    // Compute distances and adaptive bandwidth
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            float dist_squared = 0.0f;
            for (size_t k = 0; k < m; ++k) {
                float diff = X[i][k] - X[j][k];
                dist_squared += diff * diff;
            }
            
            // Adaptive bandwidth based on local density
            float local_density_i = (i > 0 && i < n-1) ? 
                (point_density[i] + point_density[i-1]) / 2.0f : 
                (i == 0 ? point_density[0] : point_density[n-2]);
                
            float local_density_j = (j > 0 && j < n-1) ? 
                (point_density[j] + point_density[j-1]) / 2.0f : 
                (j == 0 ? point_density[0] : point_density[n-2]);
                
            float density_factor = std::sqrt((local_density_i + local_density_j) / (2.0f * median_density + 1e-10f));
            float bandwidth = 1.0f * std::min(2.0f, std::max(0.5f, density_factor));
            
            // Compute kernel value with adaptive bandwidth
            K[i][j] = std::exp(-dist_squared / (2.0f * bandwidth * bandwidth));
            K[j][i] = K[i][j]; // Symmetric matrix
        }
    }
}

// Update KKT violation patterns
void AdvancedPolynomialFitter::updateKKTViolationPatterns(size_t n, const std::vector<float>& alpha,
                                                        const std::vector<float>& y,
                                                        const std::vector<float>& f,
                                                        float epsilon, float C, double b,
                                                        std::vector<float>& kkt_history,
                                                        std::vector<float>& kkt_gradients) {
    for (size_t i = 0; i < n; ++i) {
        float r = f[i] + b - y[i];
        float kkt_violation = 0.0f;
        
        if (r > epsilon) {
            if (alpha[i] > 0) kkt_violation = r - epsilon;
            else if (alpha[i + n] < C) kkt_violation = r - epsilon;
        } else if (r < -epsilon) {
            if (alpha[i + n] > 0) kkt_violation = -r - epsilon;
            else if (alpha[i] < C) kkt_violation = -r - epsilon;
        }
        
        kkt_history[i] = kkt_violation;
        
        // Compute gradient of KKT violations
        if (i > 0 && i < n - 1) {
            kkt_gradients[i] = (kkt_history[i+1] - kkt_history[i-1]) / 2.0f;
        } else if (i == 0) {
            kkt_gradients[i] = kkt_history[1] - kkt_history[0];
        } else { // i == n-1
            kkt_gradients[i] = kkt_history[n-1] - kkt_history[n-2];
        }
    }
}

// Detect critical points in KKT pattern
std::vector<size_t> AdvancedPolynomialFitter::detectCriticalPoints(
    const std::vector<float>& kkt_gradients, size_t n) {
    
    std::vector<size_t> critical_points;
    
    // Detect sign changes in gradient (zero crossings)
    for (size_t i = 1; i < n; ++i) {
        if (kkt_gradients[i-1] * kkt_gradients[i] < 0) {
            // Sign change detected
            critical_points.push_back(i);
        }
    }
    
    // If too many critical points, keep only the most significant ones
    if (critical_points.size() > 10) {
        std::vector<std::pair<float, size_t>> significance;
        for (size_t idx : critical_points) {
            float sig = std::fabs(kkt_gradients[idx-1] - kkt_gradients[idx]);
            significance.push_back({sig, idx});
        }
        
        std::sort(significance.begin(), significance.end(), 
                 [](const auto& a, const auto& b) { return a.first > b.first; });
        
        critical_points.clear();
        for (size_t i = 0; i < std::min(size_t(10), significance.size()); ++i) {
            critical_points.push_back(significance[i].second);
        }
        
        // Sort by index for consistency
        std::sort(critical_points.begin(), critical_points.end());
    }
    
    return critical_points;
}

// Compute geometric influence of a point
float AdvancedPolynomialFitter::computeGeometricInfluence(size_t idx, size_t n,
                                                        const std::vector<float>& kkt_history,
                                                        const std::vector<float>& kkt_gradients,
                                                        const std::vector<size_t>& critical_points) {
    float influence = 1.0f;
    
    // Base influence from KKT violation
    influence *= (1.0f + std::fabs(kkt_history[idx]));
    
    // Higher influence for points with large gradient
    influence *= (1.0f + 0.5f * std::fabs(kkt_gradients[idx]));
    
    // Higher influence near critical points
    for (size_t cp : critical_points) {
        float distance = std::fabs(static_cast<float>(idx) - static_cast<float>(cp));
        if (distance < 0.1f * n) {
            float proximity_factor = 1.0f + (0.1f * n - distance) / (0.1f * n);
            influence *= proximity_factor;
        }
    }
    
    return influence;
}

// Find optimal step size for SMO update
float AdvancedPolynomialFitter::findOptimalStepSize(float delta_init, float r1, float r2, 
                                                  float eta, float s2, float s_total, float C) {
    // Basic step from SMO
    float delta = delta_init;
    
    // Bounds check
    float L = std::max(-C, s_total - C);
    float H = std::min(C, s_total + C);
    
    if (s2 + delta < L) delta = L - s2;
    if (s2 + delta > H) delta = H - s2;
    
    // Line search to refine step size (simplified cubic interpolation)
    const int max_line_search = 5;
    float current_delta = delta;
    float best_delta = delta;
    float best_gain = (r1 - r2) * delta - 0.5f * eta * delta * delta;
    
    for (int i = 0; i < max_line_search; ++i) {
        // Try a smaller step
        float smaller_delta = 0.5f * current_delta;
        if (s2 + smaller_delta >= L && s2 + smaller_delta <= H) {
            float gain = (r1 - r2) * smaller_delta - 0.5f * eta * smaller_delta * smaller_delta;
            if (gain > best_gain) {
                best_gain = gain;
                best_delta = smaller_delta;
            }
        }
        
        // Try a larger step
        float larger_delta = std::min(1.5f * current_delta, H - s2);
        if (s2 + larger_delta >= L && s2 + larger_delta <= H) {
            float gain = (r1 - r2) * larger_delta - 0.5f * eta * larger_delta * larger_delta;
            if (gain > best_gain) {
                best_gain = gain;
                best_delta = larger_delta;
            }
        }
        
        if (best_delta == current_delta) break;
        current_delta = best_delta;
    }
    
    return best_delta;
}

// Determine adaptive neighborhood size
int AdvancedPolynomialFitter::determineAdaptiveNeighborhood(size_t idx, size_t n,
                                                          const std::vector<float>& kkt_gradients,
                                                          const std::vector<size_t>& critical_points) {
    // Base neighborhood size proportional to dataset size
    int base_size = std::max(5, static_cast<int>(0.1f * n));
    
    // Adjust based on local gradient
    float gradient_factor = 1.0f;
    if (idx < n) {
        gradient_factor = 1.0f + 0.5f * std::fabs(kkt_gradients[idx]);
    }
    
    // Narrow neighborhood near critical points for more focused search
    float critical_factor = 1.0f;
    for (size_t cp : critical_points) {
        float distance = std::fabs(static_cast<float>(idx) - static_cast<float>(cp));
        if (distance < 0.05f * n) {
            critical_factor = std::min(critical_factor, 0.5f + distance / (0.1f * n));
        }
    }
    
    return static_cast<int>(base_size * gradient_factor * critical_factor);
}

// Compute algebraic complementarity score
float AdvancedPolynomialFitter::computeAlgebraicComplementarity(size_t idx1, size_t idx2,
                                                              const std::vector<float>& kkt_history,
                                                              const std::vector<float>& kkt_gradients) {
    // Base complementarity from KKT violations
    float kkt_comp = 1.0f;
    
    if (idx1 < kkt_history.size() && idx2 < kkt_history.size()) {
        // Prefer pairs with opposite KKT violations
        float product = kkt_history[idx1] * kkt_history[idx2];
        if (product < 0) {
            kkt_comp = 2.0f; // Boost complementary points
        }
    }
    
    // Gradient complementarity (prefer opposite gradients)
    float grad_comp = 1.0f;
    if (idx1 < kkt_gradients.size() && idx2 < kkt_gradients.size()) {
        float grad_product = kkt_gradients[idx1] * kkt_gradients[idx2];
        if (grad_product < 0) {
            grad_comp = 1.5f; // Boost points with opposite gradients
        }
    }
    
    return kkt_comp * grad_comp;
}

// Select strategic indices for optimization
std::vector<size_t> AdvancedPolynomialFitter::selectStrategicIndices(size_t idx, size_t n, int max_samples,
                                                                   const std::vector<float>& kkt_history,
                                                                   const std::vector<float>& kkt_gradients) {
    std::vector<std::pair<float, size_t>> candidates;
    
    // Add candidates with highest KKT violations
    for (size_t i = 0; i < n; ++i) {
        if (i != idx) {
            float strategic_value = std::fabs(kkt_history[i]);
            
            // Boost points with opposite violations
            if (kkt_history[i] * kkt_history[idx] < 0) {
                strategic_value *= 2.0f;
            }
            
            // Boost points with strong gradients
            strategic_value *= (1.0f + 0.5f * std::fabs(kkt_gradients[i]));
            
            candidates.push_back({strategic_value, i});
        }
    }
    
    // Sort by strategic value in descending order
    std::sort(candidates.begin(), candidates.end(),
             [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // Select top candidates
    std::vector<size_t> selected;
    for (size_t i = 0; i < std::min(static_cast<size_t>(max_samples), candidates.size()); ++i) {
        selected.push_back(candidates[i].second);
    }
    
    return selected;
}

// Optimize alpha decomposition
void AdvancedPolynomialFitter::optimizeAlphaDecomposition(double s1_new, double s2_new, 
                                                        float r1, float r2, float epsilon,
                                                        double& a1p_new, double& a1m_new,
                                                        double& a2p_new, double& a2m_new) {
    const double precision = 1e-12;
    
    // Initial decomposition - set all components to zero
    a1p_new = a1m_new = a2p_new = a2m_new = 0.0;
    
    // Set decomposition based on sign of s values
    if (std::fabs(s1_new) < precision) {
        // Zero - both components are zero
        a1p_new = a1m_new = 0.0;
    } else if (s1_new > 0) {
        // Positive - first component is positive
        a1p_new = s1_new;
        a1m_new = 0.0;
    } else {
        // Negative - second component is positive
        a1p_new = 0.0;
        a1m_new = -s1_new;
    }
    
    if (std::fabs(s2_new) < precision) {
        // Zero - both components are zero
        a2p_new = a2m_new = 0.0;
    } else if (s2_new > 0) {
        // Positive - first component is positive
        a2p_new = s2_new;
        a2m_new = 0.0;
    } else {
        // Negative - second component is positive
        a2p_new = 0.0;
        a2m_new = -s2_new;
    }
    
    // Fine-tune decomposition based on KKT conditions
    // This may be necessary for numerical stability near epsilon-tube boundaries
    if (std::fabs(r1 - epsilon) < 0.01 * epsilon && a1p_new > 0) {
        // We're near the upper epsilon boundary, prefer a1p
        // No change needed, already using a1p
    } else if (std::fabs(r1 + epsilon) < 0.01 * epsilon && a1m_new > 0) {
        // We're near the lower epsilon boundary, prefer a1m
        // No change needed, already using a1m
    }
    
    if (std::fabs(r2 - epsilon) < 0.01 * epsilon && a2p_new > 0) {
        // We're near the upper epsilon boundary, prefer a2p
        // No change needed, already using a2p
    } else if (std::fabs(r2 + epsilon) < 0.01 * epsilon && a2m_new > 0) {
        // We're near the lower epsilon boundary, prefer a2m
        // No change needed, already using a2m
    }
}

// Calculate local curvature estimate
double AdvancedPolynomialFitter::estimateLocalCurvature(size_t i1, size_t i2,
                                                      const std::vector<std::vector<float>>& K,
                                                      size_t n) {
    // Estimate local curvature using neighboring kernel values
    double curvature = 0.0;
    int count = 0;
    
    for (size_t i = std::max(size_t(1), i1) - 1; i <= std::min(n - 1, i1 + 1); ++i) {
        for (size_t j = std::max(size_t(1), i2) - 1; j <= std::min(n - 1, i2 + 1); ++j) {
            if (i != i1 || j != i2) {
                double k_center = K[i1][i2];
                double k_neighbor = K[i][j];
                double k_diff = k_center - k_neighbor;
                curvature += k_diff * k_diff;
                count++;
            }
        }
    }
    
    return count > 0 ? curvature / count : 0.0;
}


// Compute weighted geometric median
double AdvancedPolynomialFitter::computeGeometricMedian(const std::vector<float>& candidates,
                                                      const std::vector<float>& weights) {
    if (candidates.empty()) return 0.0;
    if (candidates.size() == 1) return candidates[0];
    
    // Simple weighted median for small number of candidates
    if (candidates.size() <= 5) {
        // Create pairs of (candidate, weight)
        std::vector<std::pair<float, float>> pairs;
        for (size_t i = 0; i < candidates.size(); ++i) {
            pairs.push_back({candidates[i], weights[i]});
        }
        
        // Sort by candidate value
        std::sort(pairs.begin(), pairs.end());
        
        // Calculate cumulative weights
        float total_weight = 0.0f;
        for (const auto& p : pairs) {
            total_weight += p.second;
        }
        
        // Find median
        float cumulative = 0.0f;
        for (const auto& p : pairs) {
            cumulative += p.second;
            if (cumulative >= total_weight / 2.0f) {
                return p.first;
            }
        }
        
        return pairs.back().first;  // Should not reach here
    }
    
    // For larger sets, use Weiszfeld's algorithm for geometric median
    double median = 0.0;
    double weight_sum = 0.0;
    
    // Start with weighted mean as initial guess
    for (size_t i = 0; i < candidates.size(); ++i) {
        median += candidates[i] * weights[i];
        weight_sum += weights[i];
    }
    median /= weight_sum;
    
    // Weiszfeld iterations (simplified)
    const int max_iter = 5;
    for (int iter = 0; iter < max_iter; ++iter) {
        double numerator = 0.0;
        double denominator = 0.0;
        
        for (size_t i = 0; i < candidates.size(); ++i) {
            double distance = std::fabs(candidates[i] - median);
            if (distance < 1e-10) distance = 1e-10;  // Avoid division by zero
            
            double w = weights[i] / distance;
            numerator += candidates[i] * w;
            denominator += w;
        }
        
        if (denominator < 1e-10) break;
        
        double new_median = numerator / denominator;
        if (std::fabs(new_median - median) < 1e-6) break;
        
        median = new_median;
    }
    
    return median;
}

// Apply algebraic correction during convergence plateaus
void AdvancedPolynomialFitter::applyAlgebraicCorrection(size_t n, size_t m,
                                                      std::vector<float>& alpha,
                                                      const std::vector<std::vector<float>>& X,
                                                      const std::vector<float>& y,
                                                      std::vector<float>& f, double& b) {
    // Identify alpha components to adjust
    std::vector<size_t> adjust_indices;
    
    for (size_t i = 0; i < n; ++i) {
        // Find non-zero alphas close to boundaries
        if ((alpha[i] > 0 && alpha[i] < 0.01) || 
            (alpha[i+n] > 0 && alpha[i+n] < 0.01)) {
            adjust_indices.push_back(i);
        }
    }
    
    // Apply small perturbations to selected alphas
    if (!adjust_indices.empty()) {
        // Random number generation for perturbation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(-0.01f, 0.01f);
        
        for (size_t idx : adjust_indices) {
            // Generate small perturbation
            float perturbation = dist(gen);
            
            // Apply perturbation to positive component
            if (alpha[idx] > 0) {
                float old_alpha = alpha[idx];
                alpha[idx] = std::max(0.0f, old_alpha + perturbation);
                
                // Update function values
                float delta = alpha[idx] - old_alpha;
                if (std::fabs(delta) > 1e-10f) {
                    for (size_t i = 0; i < n; ++i) {
                        float kernel_val = 0.0f;
                        for (size_t j = 0; j < m; ++j) {
                            kernel_val += X[idx][j] * X[i][j];
                        }
                        f[i] += delta * kernel_val;
                    }
                }
            }
            
            // Apply perturbation to negative component
            if (alpha[idx + n] > 0) {
                float old_alpha = alpha[idx + n];
                alpha[idx + n] = std::max(0.0f, old_alpha - perturbation);
                
                // Update function values
                float delta = old_alpha - alpha[idx + n];
                if (std::fabs(delta) > 1e-10f) {
                    for (size_t i = 0; i < n; ++i) {
                        float kernel_val = 0.0f;
                        for (size_t j = 0; j < m; ++j) {
                            kernel_val += X[idx][j] * X[i][j];
                        }
                        f[i] += delta * kernel_val;
                    }
                }
            }
        }
        
        // Update bias
        updateBias(n, alpha, y, f, b);
    }
}

// Update bias after algebraic correction
void AdvancedPolynomialFitter::updateBias(size_t n, const std::vector<float>& alpha,
                                        const std::vector<float>& y,
                                        const std::vector<float>& f, double& b) {
    double sum_bias = 0.0;
    int count = 0;
    
    for (size_t i = 0; i < n; ++i) {
        // Use only support vectors
        if ((alpha[i] > 0 && alpha[i] < 1.0) || (alpha[i + n] > 0 && alpha[i + n] < 1.0)) {
            sum_bias += y[i] - f[i];
            count++;
        }
    }
    
    if (count > 0) {
        b = sum_bias / count;
    }
}

// Convert from orthogonal polynomial basis to monomial basis
std::vector<double> AdvancedPolynomialFitter::convertOrthogonalToMonomial(
    const std::vector<double>& orthogonal_coeffs, int degree) {
    
    size_t n = orthogonal_coeffs.size();
    std::vector<double> monomial_coeffs(n, 0.0);
    
    // For Chebyshev polynomials
    if (n > 0) monomial_coeffs[0] = orthogonal_coeffs[0];
    if (n > 1) monomial_coeffs[1] = orthogonal_coeffs[1];
    
    // Convert higher degree terms
    for (size_t i = 2; i < n; ++i) {
        // Recurrence relation for Chebyshev polynomials
        // T_0(x) = 1
        // T_1(x) = x
        // T_n(x) = 2x*T_{n-1}(x) - T_{n-2}(x)
        
        // Here we implement the inverse transformation
        if (i == 2) {
            monomial_coeffs[0] -= orthogonal_coeffs[2];
            monomial_coeffs[2] = 2.0 * orthogonal_coeffs[2];
        } else {
            // Apply Chebyshev recurrence relation to convert each term
            // For T_n(x) = 2x*T_{n-1}(x) - T_{n-2}(x)
            
            // First, add the contribution from 2x*T_{n-1}(x)
            for (size_t j = 0; j < i; ++j) {
                if (j + 1 < n) {
                    monomial_coeffs[j + 1] += 2.0 * orthogonal_coeffs[i] * monomial_coeffs[j] / orthogonal_coeffs[i-1];
                }
            }
            
            // Then, subtract the contribution from T_{n-2}(x)
            for (size_t j = 0; j < n; ++j) {
                if (j < n) {
                    monomial_coeffs[j] -= orthogonal_coeffs[i] * monomial_coeffs[j] / orthogonal_coeffs[i-2];
                }
            }
            
            // Apply the coefficient directly to the highest term
            monomial_coeffs[i] = orthogonal_coeffs[i];
        }
    }
    
    // Normalize according to the degree parameter if needed
    if (degree >= 0 && static_cast<size_t>(degree) < n) {
        monomial_coeffs.resize(degree + 1);
    }
    
    return monomial_coeffs;
}


