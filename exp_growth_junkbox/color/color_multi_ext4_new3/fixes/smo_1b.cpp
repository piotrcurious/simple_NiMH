int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
    std::vector<double>& f, double epsilon, double C,
    const std::vector<std::vector<double>>& K, double tol, double& b) {

    double y2 = y[i2];
    double F2 = f[i2];  
    double r2 = F2 + b - y2;

    bool kkt_violated = ((r2 > epsilon && alpha[i2] > 0) || (r2 < -epsilon && alpha[i2 + n] > 0) ||
                         (r2 < -epsilon && alpha[i2] < C) || (r2 > epsilon && alpha[i2 + n] < C));

    if (!kkt_violated) return 0;

    size_t i1 = -1;
    double max_delta = -1.0;

    // **FIRST HEURISTIC: Find max violation among support vectors**
    for (size_t j = 0; j < n; ++j) {
        if ((alpha[j] > 0 && alpha[j] < C) || (alpha[j + n] > 0 && alpha[j + n] < C)) {
            double r1 = f[j] + b - y[j];
            double delta = fabs(r1 - r2);

            if (delta > max_delta) {
                max_delta = delta;
                i1 = j;
            }
        }
    }

    // **SECOND HEURISTIC: Pick max violation from all samples if no support vectors work**
    if (i1 == -1) {
        for (size_t j = 0; j < n; ++j) {
            double r1 = f[j] + b - y[j];
            double delta = fabs(r1 - r2);
            if (delta > max_delta) {
                max_delta = delta;
                i1 = j;
            }
        }
    }

    // **THIRD HEURISTIC: Random selection as last resort**
    if (i1 == -1) i1 = rand() % n;

    return optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b);
}

int AdvancedPolynomialFitter::optimizePair(
    size_t i1, size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
    std::vector<double>& f, double epsilon, double C,
    const std::vector<std::vector<double>>& K, double tol, double& b) {

    if (i1 == i2) return 0;

    double y1 = y[i1];
    double y2 = y[i2];
    double F1 = f[i1];
    double F2 = f[i2];

    double k11 = K[i1][i1];
    double k12 = K[i1][i2];
    double k22 = K[i2][i2];

    double eta = k11 + k22 - 2 * k12;
    if (eta < 1e-12) return 0;

    double a1_old = alpha[i1];
    double a2_old = alpha[i2];

    double r1 = F1 + b - y1;
    double r2 = F2 + b - y2;

    double a2_new = a2_old + (y2 * (r1 - r2)) / eta;

    double L = std::max(0.0, a2_old - a1_old);
    double H = std::min(C, C + a2_old - a1_old);

    a2_new = std::max(L, std::min(H, a2_new));
    double a1_new = a1_old + (a2_old - a2_new);

    if (fabs(a2_new - a2_old) < tol) return 0;

    alpha[i1] = a1_new;
    alpha[i2] = a2_new;

    // **Improved bias updates**
    double b_new;
    if (a1_new > 0 && a1_new < C) {
        b_new = y1 - (F1 + (a1_new - a1_old) * k11 + (a2_new - a2_old) * k12);
    } else if (a2_new > 0 && a2_new < C) {
        b_new = y2 - (F2 + (a1_new - a1_old) * k12 + (a2_new - a2_old) * k22);
    } else {
        b_new = (b + b_new) / 2.0;
    }

    b = b_new;

    // Update function values
    for (size_t i = 0; i < n; ++i) {
        f[i] += (a1_new - a1_old) * K[i1][i] + (a2_new - a2_old) * K[i2][i];
    }

    return 1;
}
