std::vector<float> AdvancedPolynomialFitter::fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree,
                                                    OptimizationMethod method) {
    // Check for invalid input
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    
    // Debug: print first and last x values
    Serial.println(x.front());
    Serial.println(x.back());

    size_t n = x.size();
    size_t m = degree + 1;

    // Normalize input data for numerical stability
    float x_min = *std::min_element(x.begin(), x.end());
    float x_max = *std::max_element(x.begin(), x.end());
    float x_range = x_max - x_min;

    std::vector<float> x_norm(n);
    for (size_t i = 0; i < n; ++i) {
        x_norm[i] = (x[i] - x_min) / x_range;
    }

    // Build the Vandermonde matrix using normalized x values.
    // (Using a running multiplier rather than pow() improves efficiency.)
    std::vector<std::vector<float>> X(n, std::vector<float>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        X[i][0] = 1.0;
        float x_power = 1.0;  // x_norm[i]^0
        for (size_t j = 1; j < m; ++j) {
            x_power *= x_norm[i];
            X[i][j] = x_power;
        }
    }

    // Compute standard LS coefficients using the normalized x data.
    std::vector<float> ls_coeffs = fitPolynomial_superpos5c(x_norm, y, degree, OptimizationMethod::NONE);

    // Compute LS residuals using normalized x (ensuring consistency with LS fit).
    std::vector<float> ls_residuals(n);
    for (size_t i = 0; i < n; ++i) {
        float prediction = 0.0;
        float x_power = 1.0;
        for (size_t j = 0; j < ls_coeffs.size(); ++j) {
            prediction += ls_coeffs[j] * x_power;
            x_power *= x_norm[i];
        }
        ls_residuals[i] = prediction - y[i];
    }
    
    // Hyperparameters for SVR (epsilon-insensitive loss and regularization)
    const float C = 100000.0f;
    const float epsilon = 0.1f;
    const float tol = 1e-6f;
    const int maxIter = 5000;

    // Initialize dual variables alpha (size 2*n for positive and negative parts)
    std::vector<float> alpha(2 * n, 0.0f);

    // Warm-start: Initialize function values f using the LS predictions.
    std::vector<float> f(n, 0.0f);
    for (size_t i = 0; i < n; ++i) {
        float prediction = 0.0f;
        float x_power = 1.0f;
        for (size_t j = 0; j < ls_coeffs.size(); ++j) {
            prediction += ls_coeffs[j] * x_power;
            x_power *= x_norm[i];
        }
        f[i] = prediction;
    }
    
    // Initialize bias b as the average residual (y - prediction) from the LS fit.
    double b = 0.0;
    for (size_t i = 0; i < n; ++i) {
        b += (y[i] - f[i]);
    }
    b /= n;

    // Compute the kernel matrix (linear kernel: dot products of feature vectors from X)
    std::vector<std::vector<float>> K(n, std::vector<float>(n, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i; j < n; ++j) {  // Compute only upper triangle (matrix is symmetric)
            float dot = 0.0f;
            for (size_t k = 0; k < m; ++k) {
                dot += X[i][k] * X[j][k];
            }
            K[i][j] = dot;
            if (i != j) {
                K[j][i] = dot;
            }
        }
    }

    // --- SMO algorithm for SVR ---
    int iter = 0;
    int numChanged = 0;
    bool examineAll = true;

    while ((numChanged > 0 || examineAll) && iter < maxIter) {
        numChanged = 0;

        if (examineAll) {
            // Loop over all examples
            for (size_t i = 0; i < n; ++i) {
                numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, ls_residuals);
            }
        } else {
            // Loop only over examples with non-bound alpha values
            for (size_t i = 0; i < n; ++i) {
                if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i + n] > 0 && alpha[i + n] < C)) {
                    numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b, ls_residuals);
                }
            }
        }

        if (examineAll) {
            examineAll = false;
        } else if (numChanged == 0) {
            examineAll = true;
        }

        iter++;
        if (iter % 10 == 0) {
            Serial.print("Iteration: ");
            Serial.print(iter);
            Serial.print(", Changed: ");
            Serial.println(numChanged);
        }
    }

    // Compute the primal coefficients from the dual variables.
    std::vector<double> w(m, 0.0);
    for (size_t j = 0; j < m; ++j) {
        for (size_t i = 0; i < n; ++i) {
            double alpha_diff = alpha[i] - alpha[i + n];  // Difference of positive and negative parts
            w[j] += alpha_diff * X[i][j];
        }
    }

    // Denormalize coefficients to the original x scale
    std::vector<double> denorm_coeffs = denormalizeCoefficients(w, x_min, x_range, degree);
    std::vector<float> result(denorm_coeffs.begin(), denorm_coeffs.end());

    // Debug output of final coefficients
    Serial.print("SVM Coefficients: ");
    for (size_t i = 0; i < result.size(); ++i) {
        Serial.print(result[i]); 
        Serial.print(" ");
    }
    Serial.println();

    return result;
}
