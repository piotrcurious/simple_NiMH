int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
    std::vector<double>& f, double epsilon, double C,
    const std::vector<std::vector<double>>& K, double tol, double& b) {
    
    // Compute error for the second example
    double y2 = y[i2];
    double E2 = f[i2] + b - y2;
    double r2 = E2;
    
    // Check KKT violation for i2 (using both positive and negative alpha components)
    bool violated = ( (r2 > epsilon && alpha[i2] > 0) ||
                      (r2 < -epsilon && alpha[i2+n] > 0) ||
                      (r2 < -epsilon && alpha[i2] < C) ||
                      (r2 > epsilon && alpha[i2+n] < C) );
    if (!violated) {
        return 0;
    }
    
    // Build a list of indices for non-bound examples
    std::vector<size_t> nonBoundIndices;
    for (size_t i = 0; i < n; ++i) {
        if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i+n] > 0 && alpha[i+n] < C)) {
            nonBoundIndices.push_back(i);
        }
    }
    
    // Sort the non-bound indices in descending order by |E[i] - E2|
    std::sort(nonBoundIndices.begin(), nonBoundIndices.end(), [&](size_t i1, size_t i2_) {
        double E1 = f[i1] + b - y[i1];
        double E1_alt = f[i2_] + b - y[i2_];
        return fabs(E1 - E2) > fabs(E1_alt - E2);
    });
    
    // First try: loop over non-bound examples in sorted order
    for (size_t i1 : nonBoundIndices) {
        if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b))
            return 1;
    }
    
    // Second try: loop over all examples in random order
    std::vector<size_t> allIndices(n);
    std::iota(allIndices.begin(), allIndices.end(), 0);
    std::random_shuffle(allIndices.begin(), allIndices.end());
    for (size_t i1 : allIndices) {
        if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b))
            return 1;
    }
    
    return 0;
}

// SMO Algorithm for SVR - Main loop
int iter = 0;
int numChanged = 0;
bool examineAll = true;

while ((numChanged > 0 || examineAll) && iter < maxIter) {
    numChanged = 0;
    std::vector<size_t> indices;
    
    if (examineAll) {
        // All training examples
        indices.resize(n);
        std::iota(indices.begin(), indices.end(), 0);
        std::random_shuffle(indices.begin(), indices.end());
    } else {
        // Only examples where alpha is not at bounds
        for (size_t i = 0; i < n; ++i) {
            if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i+n] > 0 && alpha[i+n] < C))
                indices.push_back(i);
        }
        std::random_shuffle(indices.begin(), indices.end());
    }
    
    for (size_t i : indices) {
        numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b);
    }
    
    if (examineAll)
        examineAll = false;
    else if (numChanged == 0)
        examineAll = true;
    
    iter++;
    
    // Debug output (optional)
    if (iter % 100 == 0) {
        Serial.print("Iteration: ");
        Serial.print(iter);
        Serial.print(", Changed: ");
        Serial.println(numChanged);
    }
}
