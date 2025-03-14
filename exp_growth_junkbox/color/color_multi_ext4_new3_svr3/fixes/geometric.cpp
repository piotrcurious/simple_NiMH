#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip> // Required for setprecision


#ifndef ARDUINO_H
#define ARDUINO_H // Dummy Serial for non-Arduino environments
#include <stdio.h>
#define Serial SerialDummy
class SerialDummyClass {
public:
    void begin(long baud) {}
    void print(const char *str) { printf(str); }
    void print(int val) { printf("%d", val); }
    void print(float val) { printf("%f", val); }
    void print(double val) { printf("%lf", val); }
    void println(const char *str) { printf("%s\n", str); }
    void println(int val) { printf("%d\n", val); }
    void println(float val) { printf("%f\n", val); }
    void println(double val) { printf("%lf\n", val); }
};
extern SerialDummyClass SerialDummy;
SerialDummyClass SerialDummy;
#endif


class AdvancedPolynomialFitter {
public:
    enum OptimizationMethod { NONE, SVM };

    std::vector<float> fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method);

private:
    int examineExample(
        size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
        std::vector<float>& f, float epsilon, float C,
        const std::vector<std::vector<float>>& K, float tol, double& b,
        const std::vector<float>& weighted_residuals,
        const std::vector<float>& kkt_history,
        const std::vector<float>& kkt_gradients,
        const std::vector<size_t>& critical_points);

    int optimizePair(
        size_t i1, size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
        std::vector<float>& f, float epsilon, float C,
        const std::vector<std::vector<float>>& K, float tol, double& b);

    // Helper functions (to be implemented below)
    float computeMedian(const std::vector<float>& values);
    float computeStdDev(const std::vector<float>& values, float median);
    std::vector<float> computeSVD(const std::vector<std::vector<float>>& matrix); // Placeholder - Implement SVD
    float computeLocalSensitivity(const std::vector<float>& x_norm, size_t index, float median_density);
    void computeAdaptiveKernel(std::vector<std::vector<float>>& K, const std::vector<std::vector<float>>& X, size_t n, size_t m, const std::vector<float>& point_density, float median_density);
    float computeGeometricInfluence(size_t i, size_t n, const std::vector<float>& kkt_history, const std::vector<float>& kkt_gradients, const std::vector<size_t>& critical_points);
    void updateKKTViolationPatterns(size_t n, std::vector<float>& alpha, const std::vector<float>& y, std::vector<float>& f, float epsilon, float C, double b, std::vector<float>& kkt_history, std::vector<float>& kkt_gradients);
    std::vector<size_t> detectCriticalPoints(const std::vector<float>& kkt_gradients, size_t n);
    float computeAlgebraicComplementarity(size_t i1, size_t i2, const std::vector<float>& kkt_history, const std::vector<float>& kkt_gradients);
    int determineAdaptiveNeighborhood(size_t i2, size_t n, const std::vector<float>& kkt_gradients, const std::vector<size_t>& critical_points);
    std::vector<size_t> selectStrategicIndices(size_t i2, size_t n, int max_samples, const std::vector<float>& kkt_history, const std::vector<float>& kkt_gradients);
    double estimateLocalCurvature(size_t i1, size_t i2, const std::vector<std::vector<float>>& K, size_t n);
    float findOptimalStepSize(float delta_init, float r1, float r2, double eta, float s2, float s_total, float C);
    void optimizeAlphaDecomposition(double s1_new, double s2_new, float r1, float r2, float epsilon, double& a1p_new, double& a1m_new, double& a2p_new, double& a2m_new);
    double computeGeometricMedian(const std::vector<float>& bias_candidates, const std::vector<float>& bias_weights);
    std::vector<double> convertOrthogonalToMonomial(const std::vector<double>& orthogonal_coeffs, int degree);
    std::vector<double> denormalizeCoefficients(const std::vector<double>& monomial_coeffs, float x_min, float x_range, int degree);
    std::vector<float> fitPolynomial_superpos5c(const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method); // Placeholder - Implement LS fit


    float computeRobustScale(const std::vector<float>& y) { // Simple MAD estimator for robustness
        std::vector<float> abs_devs;
        float y_median = computeMedian(y);
        for(float val : y) {
            abs_devs.push_back(std::abs(val - y_median));
        }
        return 1.4826f * computeMedian(abs_devs); // Consistent estimator for normal distribution
    }
};


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

    // NEW: Compute weighted residuals based on singular value analysis (Placeholder function, SVD needed)
    std::vector<float> singular_values = computeSVD(X);
    float condition_number = (singular_values.empty() || singular_values.back() == 0.0f) ? std::numeric_limits<float>::infinity() : singular_values[0] / singular_values.back();


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
    const float C = 100000.0f * (1.0f / (1.0f + log10f(1.0f + condition_number)));  // Adjust C based on conditioning (log10f for better scaling)
    const float epsilon = 0.1f * computeRobustScale(y);  // Adapt epsilon to data scale
    const float tol = 1e-6f * (1.0f + 0.1f * log10f(1.0f + condition_number));  // Adaptive tolerance (log10f for better scaling)
    const int maxIter = std::min(5000, static_cast<int>(50 * degree * log10f(n+1)));  // Adaptive iterations (log10f for better scaling and n+1 to avoid log(0))


    // Initialize alphas and function values
    std::vector<float> alpha(2 * n, 0.0f);
    std::vector<float> f(n, 0.0f);
    double b = 0.0;

    // NEW: Use adaptive kernel based on data distribution
    std::vector<std::vector<float>> K(n, std::vector<float>(n, 0.0f));
    computeAdaptiveKernel(K, X, n, m, point_density, median_density);

    // IMPROVED: Track global KKT violation patterns for geometric inference
    std::vector<float> kkt_history(n, 0.0f);
    std::vector<float> kkt_gradients(n, 0.0f);

    // NEW: Adaptive SMO with geometric inference
    int iter = 0;
    int numChanged = 0;
    bool examineAll = true;
    int plateau_counter = 0;
    float previous_violation_sum = std::numeric_limits<float>::max();

    while ((numChanged > 0 || examineAll) && iter < maxIter) {
        numChanged = 0;
        float current_violation_sum = 0.0f;

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

            // If stuck in plateau, apply algebraic correction (Placeholder function needed)
            if (plateau_counter > 5) {
                //applyAlgebraicCorrection(n, m, alpha, X, y, f, b); // Placeholder function
                plateau_counter = 0; // Reset after correction attempt even if placeholder
                Serial.println("Algebraic Correction Placeholder Called"); // Indicate correction attempt
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
        float max_weighted_delta = 0.0f; // Initialize as float
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
    if (eta <= 0) { // Changed to <= 0 for robustness
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

    // NEW: Implement cubic line search for optimal step size (Placeholder function needed)
    float delta = findOptimalStepSize(delta_init, r1, r2, eta, s2, s_total, C); // Placeholder function

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
    const float boundary_buffer = 1e-8f * C; // Use float literal
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

    // Analyze KKT conditions to guide decomposition (Placeholder function needed)
    optimizeAlphaDecomposition(s1_new, s2_new, r1, r2, epsilon,
                              a1p_new, a1m_new, a2p_new, a2m_new); // Placeholder Function

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

// NEW: Helper functions for the improved implementation

// Compute median of a vector
float AdvancedPolynomialFitter::computeMedian(const std::vector<float>& values) {
    if (values.empty()) return 0.0f;

    std::vector<float> sorted_values = values;
    std::sort(sorted_values.begin(), sorted_values.end());
    size_t n = sorted_values.size();
    if (n % 2 == 0) {
        return (sorted_values[n / 2 - 1] + sorted_values[n / 2]) / 2.0f;
    } else {
        return sorted_values[n / 2];
    }
}

// Compute standard deviation
float AdvancedPolynomialFitter::computeStdDev(const std::vector<float>& values, float median) {
    if (values.empty()) return 0.0f;
    float sum_sq_diff = 0.0f;
    for (float val : values) {
        sum_sq_diff += (val - median) * (val - median);
    }
    return std::sqrt(sum_sq_diff / (values.size() - 1));
}

// Placeholder for SVD computation - Replace with actual SVD implementation
std::vector<float> AdvancedPolynomialFitter::computeSVD(const std::vector<std::vector<float>>& matrix) {
    // Placeholder: Implement SVD decomposition here if needed for condition number.
    // For now, return a dummy vector for compilation.
    Serial.println("Warning: SVD computation is a placeholder. Returning dummy singular values.");
    if (matrix.empty() || matrix[0].empty()) return {1.0f, 1.0f}; // Avoid division by zero in condition number
    return {100.0f, 1.0f}; // Dummy singular values - replace with actual SVD
}


float AdvancedPolynomialFitter::computeLocalSensitivity(const std::vector<float>& x_norm, size_t index, float median_density) {
    if (x_norm.empty() || index >= x_norm.size() || x_norm.size() < 2) return 1.0f; // Default sensitivity

    float density;
    if (index == 0) {
        density = x_norm[1] - x_norm[0];
    } else if (index == x_norm.size() - 1) {
        density = x_norm.back() - x_norm[x_norm.size() - 2];
    } else {
        density = (x_norm[index] - x_norm[index - 1] + x_norm[index + 1] - x_norm[index]) / 2.0f;
    }

    return std::max(0.1f, median_density / std::max(1e-6f, density)); // Avoid division by zero and ensure min sensitivity
}


void AdvancedPolynomialFitter::computeAdaptiveKernel(std::vector<std::vector<float>>& K, const std::vector<std::vector<float>>& X, size_t n, size_t m, const std::vector<float>& point_density, float median_density) {
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            float density_factor_i = (i < n - 1) ? median_density / std::max(1e-6f, point_density[i]) : 1.0f; // Avoid div by zero
            float density_factor_j = (j < n - 1) ? median_density / std::max(1e-6f, point_density[j]) : 1.0f; // Avoid div by zero
            float adaptive_factor = std::sqrt(density_factor_i * density_factor_j);
            K[i][j] = adaptive_factor * 0.1f * std::inner_product(X[i].begin(), X[i].end(), X[j].begin(), 0.0f); // Adjusted kernel & factor
        }
    }
}


float AdvancedPolynomialFitter::computeGeometricInfluence(size_t i, size_t n, const std::vector<float>& kkt_history, const std::vector<float>& kkt_gradients, const std::vector<size_t>& critical_points) {
    float influence = fabs(kkt_history[i]) + 0.1f * fabs(kkt_gradients[i]); // Base influence from KKT violation & gradient

    if (std::find(critical_points.begin(), critical_points.end(), i) != critical_points.end()) {
        influence *= 2.0f; // Critical points are more influential
    }

    if (i > 0 && i < n - 1) {
        float curvature = fabs(kkt_gradients[i - 1] - 2 * kkt_gradients[i] + kkt_gradients[i + 1]);
        influence += 0.05f * curvature; // Add influence from curvature
    }
    return influence;
}


void AdvancedPolynomialFitter::updateKKTViolationPatterns(size_t n, std::vector<float>& alpha, const std::vector<float>& y, std::vector<float>& f, float epsilon, float C, double b, std::vector<float>& kkt_history, std::vector<float>& kkt_gradients) {
    for (size_t i = 0; i < n; ++i) {
        float r_i = f[i] + b - y[i];
        if (alpha[i] == 0) {
            kkt_history[i] = std::max(0.0f, -r_i - epsilon);
            kkt_gradients[i] = (r_i < -epsilon) ? -1.0f : 0.0f; // Gradient direction
        } else if (alpha[i] == C) {
            kkt_history[i] = std::max(0.0f, r_i - epsilon);
            kkt_gradients[i] = (r_i > epsilon) ? 1.0f : 0.0f; // Gradient direction
        } else {
            kkt_history[i] = std::max(0.0f, fabs(r_i) - epsilon); // For 0 < alpha < C, violation is around epsilon
            kkt_gradients[i] = (r_i > 0) ? 1.0f : (r_i < 0 ? -1.0f : 0.0f); // Gradient direction
        }
    }
}


std::vector<size_t> AdvancedPolynomialFitter::detectCriticalPoints(const std::vector<float>& kkt_gradients, size_t n) {
    std::vector<size_t> critical_points;
    for (size_t i = 1; i < n - 1; ++i) {
        if ((kkt_gradients[i] > 0 && kkt_gradients[i - 1] < 0) || (kkt_gradients[i] < 0 && kkt_gradients[i - 1] > 0)) {
            critical_points.push_back(i); // Sign change in gradient indicates critical point
        } else if (kkt_gradients[i] == 0 && kkt_gradients[i-1] != 0) { // Zero gradient after non-zero
             critical_points.push_back(i);
        }
    }
    return critical_points;
}


float AdvancedPolynomialFitter::computeAlgebraicComplementarity(size_t i1, size_t i2, const std::vector<float>& kkt_history, const std::vector<float>& kkt_gradients) {
    float history_complement = 1.0f / (1.0f + fabs(kkt_history[i1] + kkt_history[i2])); // Lower if violations are complementary
    float gradient_alignment = std::max(0.1f, kkt_gradients[i1] * kkt_gradients[i2] + 0.5f); // Higher if gradients align (more complementary update)
    return history_complement * gradient_alignment;
}


int AdvancedPolynomialFitter::determineAdaptiveNeighborhood(size_t i2, size_t n, const std::vector<float>& kkt_gradients, const std::vector<size_t>& critical_points) {
    int neighborhood_size = 5; // Base neighborhood

    if (std::find(critical_points.begin(), critical_points.end(), i2) != critical_points.end()) {
        neighborhood_size = std::min(n, neighborhood_size * 2); // Larger near critical points
    }

    float local_gradient_change = 0;
    if (i2 > 0 && i2 < n - 1) {
        local_gradient_change = fabs(kkt_gradients[i2 + 1] - kkt_gradients[i2 - 1]);
    }
    neighborhood_size = std::max(neighborhood_size, (int)(local_gradient_change * 3)); // Increase with gradient change

    return std::min(n, neighborhood_size);
}

std::vector<size_t> AdvancedPolynomialFitter::selectStrategicIndices(size_t i2, size_t n, int max_samples, const std::vector<float>& kkt_history, const std::vector<float>& kkt_gradients) {
    std::vector<std::pair<float, size_t>> strategic_candidates;

    for (size_t i = 0; i < n; ++i) {
        if (i == i2) continue; // Skip i2 itself

        float strategic_value = fabs(kkt_history[i]) + 0.01f * fabs(kkt_gradients[i]); // Prioritize high violation & gradient

        if (i > 0 && i < n - 1) {
             strategic_value += 0.005f * fabs(kkt_gradients[i-1] - 2*kkt_gradients[i] + kkt_gradients[i+1]); // Add curvature influence
        }
        strategic_candidates.push_back({strategic_value, i});
    }

    std::sort(strategic_candidates.begin(), strategic_candidates.end(), [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<size_t> strategic_indices;
    int samples_taken = 0;
    for (const auto& candidate : strategic_candidates) {
        strategic_indices.push_back(candidate.second);
        samples_taken++;
        if (samples_taken >= max_samples) break;
    }
    return strategic_indices;
}


double AdvancedPolynomialFitter::estimateLocalCurvature(size_t i1, size_t i2, const std::vector<std::vector<float>>& K, size_t n) {
    float k11 = K[i1][i1];
    float k22 = K[i2][i2];
    float k12 = K[i1][i2];
    double eta = k11 + k22 - 2 * k12;
    if (eta <= 0) return 1.0; // Flat or negative curvature

    double curvature = eta / std::max(1e-6, (double)(k11 + k22 + k12)); // Normalized curvature measure
    return curvature;
}

float AdvancedPolynomialFitter::findOptimalStepSize(float delta_init, float r1, float r2, double eta, float s2, float s_total, float C) {
     // Placeholder: Implement a more sophisticated line search (e.g., cubic interpolation)
     // For now, using a simple step size reduction if bounds are violated.
     float delta = delta_init;
     float s2_trial = s2 + delta;
     float L = std::max(-C, s_total - C);
     float H = std::min(C, s_total + C);

     if (s2_trial < L || s2_trial > H) {
         delta *= 0.5f; // Simple step size reduction - could be replaced with cubic line search
     }
     return delta;
}


void AdvancedPolynomialFitter::optimizeAlphaDecomposition(double s1_new, double s2_new, float r1, float r2, float epsilon, double& a1p_new, double& a1m_new, double& a2p_new, double& a2m_new) {
    // Placeholder: Implement a more refined decomposition strategy, possibly based on KKT violations or geometric context.
    // For now, using a simple heuristic: prioritize positive components if residuals are positive, and negative otherwise.

    if ((r1 + r2) > 0) { // If sum of residuals is positive, prioritize positive alphas
        a2p_new = std::max(0.0, s2_new);
        a2m_new = std::max(0.0, a2p_new - s2_new); // a2m_new will be 0 if a2p_new = s2_new
        a1p_new = std::max(0.0, s1_new);
        a1m_new = std::max(0.0, a1p_new - s1_new); // a1m_new will be 0 if a1p_new = s1_new
    } else {          // Otherwise, prioritize negative alphas
        a2m_new = std::max(0.0, -s2_new);
        a2p_new = std::max(0.0, a2m_new + s2_new); // a2p_new will be 0 if a2m_new = -s2_new
        a1m_new = std::max(0.0, -s1_new);
        a1p_new = std::max(0.0, a1m_new + s1_new); // a1p_new will be 0 if a1m_new = -s1_new
    }

    // Ensure non-negativity (redundant with std::max(0.0, ...), but for clarity)
    a1p_new = std::max(0.0, a1p_new);
    a1m_new = std::max(0.0, a1m_new);
    a2p_new = std::max(0.0, a2p_new);
    a2m_new = std::max(0.0, a2m_new);
}


double AdvancedPolynomialFitter::computeGeometricMedian(const std::vector<float>& bias_candidates, const std::vector<float>& bias_weights) {
    if (bias_candidates.empty()) return 0.0;

    double weighted_sum = 0.0;
    double total_weight = 0.0;
    for (size_t i = 0; i < bias_candidates.size(); ++i) {
        weighted_sum += bias_candidates[i] * bias_weights[i];
        total_weight += bias_weights[i];
    }
    if (total_weight == 0) return 0.0; // Avoid division by zero if all weights are zero (unlikely)
    return weighted_sum / total_weight; // Weighted average as a simplified geometric median
}


std::vector<double> AdvancedPolynomialFitter::convertOrthogonalToMonomial(const std::vector<double>& orthogonal_coeffs, int degree) {
    std::vector<double> monomial_coeffs(degree + 1, 0.0);
    if (orthogonal_coeffs.empty()) return monomial_coeffs;

    monomial_coeffs[0] = orthogonal_coeffs[0]; // Constant term is the same

    if (degree >= 1) {
        monomial_coeffs[1] = orthogonal_coeffs[1] * 1.0; // Linear term (Chebyshev T1(x) = x)
        monomial_coeffs[0] -= orthogonal_coeffs[1] * 0.0; // Constant adjustment (none for T1)
    }
    if (degree >= 2) {
        monomial_coeffs[2] = orthogonal_coeffs[2] * 2.0; // Quadratic term (Chebyshev T2(x) = 2x^2 - 1)
        monomial_coeffs[0] -= orthogonal_coeffs[2] * 1.0; // Constant adjustment
    }
    if (degree >= 3) {
        monomial_coeffs[3] = orthogonal_coeffs[3] * 4.0; // Cubic term (Chebyshev T3(x) = 4x^3 - 3x)
        monomial_coeffs[1] -= orthogonal_coeffs[3] * 3.0; // Linear adjustment
    }
    if (degree >= 4) {
        monomial_coeffs[4] = orthogonal_coeffs[4] * 8.0; // Quartic term (Chebyshev T4(x) = 8x^4 - 8x^2 + 1)
        monomial_coeffs[2] -= orthogonal_coeffs[4] * 8.0; // Quadratic adjustment
        monomial_coeffs[0] += orthogonal_coeffs[4] * 1.0; // Constant adjustment
    }
    // Add higher degree conversions if needed up to the maximum degree.
    // For higher degrees, you'd continue expanding Chebyshev polynomials in terms of monomials.

    return monomial_coeffs;
}


std::vector<double> AdvancedPolynomialFitter::denormalizeCoefficients(const std::vector<double>& monomial_coeffs, float x_min, float x_range, int degree) {
    std::vector<double> denorm_coeffs(degree + 1, 0.0);
    if (x_range == 0.0) { // Handle zero range to prevent NaN/Inf
         return monomial_coeffs; // No denormalization needed if no normalization occurred. Or handle differently as per use case.
    }

    for (int i = 0; i <= degree; ++i) {
        denorm_coeffs[i] = 0.0;
        for (int j = i; j <= degree; ++j) {
            denorm_coeffs[i] += monomial_coeffs[j] * std::pow( -x_min / x_range, j - i) * ( (j == i) ? 1.0 : std::pow(1.0 / x_range, (j-i)) );
        }
    }

    return denorm_coeffs;
}
