int AdvancedPolynomialFitter::optimizePair(
    size_t i1, size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
    std::vector<float>& f, float epsilon, float C,
    const std::vector<std::vector<float>>& K, float tol, double& b) {

    if (i1 == i2) return 0;

    float y1 = y[i1], y2 = y[i2];
    float F1 = f[i1], F2 = f[i2];
    float r1 = F1 + b - y1, r2 = F2 + b - y2;

    float k11 = K[i1][i1], k12 = K[i1][i2], k22 = K[i2][i2];

    // Compute eta (curvature) and adjust for near-singularity
    double eta = k11 + k22 - 2 * k12;
    if (eta < 1e-12) {
        if (k11 + k22 == 0) return 0;
        eta = std::max(1e-12, eta + 1e-8 * (k11 + k22));
    }

    // Save old dual variables
    double a1p_old = alpha[i1], a1m_old = alpha[i1+n];
    double a2p_old = alpha[i2], a2m_old = alpha[i2+n];

    double s1 = a1p_old - a1m_old;
    double s2 = a2p_old - a2m_old;
    float s_total = s1 + s2;

    // Improved update step using second-order information
    float delta = (r1 - r2) / eta;
    double expected_gain = (r1 - r2) * delta - 0.5 * eta * delta * delta;
    if (expected_gain < tol) return 0;

    float s2_new = s2 + delta;
    float L = std::max(-C, s_total - C);
    float H = std::min(C, s_total + C);
    const float boundary_buffer = 1e-8f * C;
    L += boundary_buffer;
    H -= boundary_buffer;
    s2_new = std::min(std::max(s2_new, L), H);

    if (fabs(s2_new - s2) < tol) return 0;
    double s1_new = s_total - s2_new;

    // Convert net variables back to positive and negative components
    double a1p_new = (s1_new >= 0) ? s1_new : 0;
    double a1m_new = (s1_new < 0) ? -s1_new : 0;
    double a2p_new = (s2_new >= 0) ? s2_new : 0;
    double a2m_new = (s2_new < 0) ? -s2_new : 0;

    const double max_alpha = C * (1.0 - 1e-10);
    a1p_new = std::min(a1p_new, max_alpha);
    a1m_new = std::min(a1m_new, max_alpha);
    a2p_new = std::min(a2p_new, max_alpha);
    a2m_new = std::min(a2m_new, max_alpha);

    if ((fabs(a1p_new - a1p_old) < tol && fabs(a1m_new - a1m_old) < tol) &&
        (fabs(a2p_new - a2p_old) < tol && fabs(a2m_new - a2m_old) < tol)) {
        return 0;
    }

    // Update dual variables
    alpha[i1]   = a1p_new;
    alpha[i1+n] = a1m_new;
    alpha[i2]   = a2p_new;
    alpha[i2+n] = a2m_new;

    double delta1 = (a1p_new - a1p_old) - (a1m_new - a1m_old);
    double delta2 = (a2p_new - a2p_old) - (a2m_new - a2m_old);

    // Update bias using candidate biases from support vectors
    std::vector<float> bias_candidates, bias_weights;
    auto addBiasCandidate = [&](float b_value, float weight) {
        bias_candidates.push_back(b_value);
        bias_weights.push_back(weight);
    };

    if (a1p_new > 0 && a1p_new < C)
        addBiasCandidate(y1 - F1 - epsilon - delta1 * k11 - delta2 * k12, 1.0f);
    else if (a1m_new > 0 && a1m_new < C)
        addBiasCandidate(y1 - F1 + epsilon - delta1 * k11 - delta2 * k12, 1.0f);

    if (a2p_new > 0 && a2p_new < C)
        addBiasCandidate(y2 - F2 - epsilon - delta1 * k12 - delta2 * k22, 1.0f);
    else if (a2m_new > 0 && a2m_new < C)
        addBiasCandidate(y2 - F2 + epsilon - delta1 * k12 - delta2 * k22, 1.0f);

    if (!bias_candidates.empty()) {
        double sum_weights = 0.0, weighted_sum = 0.0;
        for (size_t i = 0; i < bias_candidates.size(); ++i) {
            weighted_sum += bias_candidates[i] * bias_weights[i];
            sum_weights += bias_weights[i];
        }
        b = weighted_sum / sum_weights;
    }

    // Update f (the decision function) using cached kernel values in chunks for efficiency
    const int update_chunk_size = 16;
    for (size_t chunk_start = 0; chunk_start < n; chunk_start += update_chunk_size) {
        size_t chunk_end = std::min(n, chunk_start + update_chunk_size);
        for (size_t i = chunk_start; i < chunk_end; ++i) {
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
