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
