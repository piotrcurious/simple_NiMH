std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method ) {

        const size_t n = x.size();
        const size_t m = degree + 1;

        // Use Eigen or Armadillo for more robust linear algebra if possible
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        // Precompute powers to avoid repeated multiplication
        std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 1; j < m; ++j) {
                xPowers[i][j] = xPowers[i][j-1] * x[i];
            }
        }

        // Compute A^T * A and A^T * y
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += xPowers[i][j] * y[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA[j][k] += xPowers[i][j] * xPowers[i][k];
                }
            }
        }

        // Fill symmetric matrix
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA[j][k] = ATA[k][j];
            }
        }

          std::vector<double> coeffs = solveQR(ATA, ATy);
          // Convert coefficients to float
          std::vector<float> result(coeffs.begin(), coeffs.end());
          return result; 
    }


// QR decomposition solver (previous implementation remains largely the same)
   std::vector<double> AdvancedPolynomialFitter::solveQR(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        const size_t n = A.size();
        const double eps = std::numeric_limits<double>::epsilon();

        // Householder QR decomposition
        for (size_t k = 0; k < n; ++k) {
            // Compute column norm with numerical stability
            double norm_x = 0.0;
            for (size_t i = k; i < n; ++i) {
                norm_x += A[i][k] * A[i][k];
            }
            norm_x = std::sqrt(std::max(norm_x, eps));

            // Avoid potential overflow/underflow
            double alpha = (A[k][k] > 0) ? -norm_x : norm_x;
            double r = std::sqrt(std::max(0.5 * (alpha * alpha - A[k][k] * alpha), eps));

            std::vector<double> v(n, 0.0);
            v[k] = (A[k][k] - alpha) / (2 * r);
            for (size_t i = k + 1; i < n; ++i) {
                v[i] = A[i][k] / (2 * r);
            }

            // Apply Householder transformation to A and b
            for (size_t j = k; j < n; ++j) {
                double dot = 0.0;
                for (size_t i = k; i < n; ++i) {
                    dot += v[i] * A[i][j];
                }
                for (size_t i = k; i < n; ++i) {
                    A[i][j] -= 2 * v[i] * dot;
                }
            }

            double dot_b = 0.0;
            for (size_t i = k; i < n; ++i) {
                dot_b += v[i] * b[i];
            }
            for (size_t i = k; i < n; ++i) {
                b[i] -= 2 * v[i] * dot_b;
            }

            A[k][k] = alpha;
            for (size_t i = k + 1; i < n; ++i) {
                A[i][k] = 0.0;
            }

            
        }

        // Back substitution with added numerical checks
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            // Add a small check to prevent division by near-zero
            x[i] /= (std::abs(A[i][i]) > eps) ? A[i][i] : eps;
        }

        return x;
    }
