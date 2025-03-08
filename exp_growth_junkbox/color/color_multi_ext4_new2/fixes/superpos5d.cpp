#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric> // for std::inner_product
#include <algorithm> // for std::abs, std::max, std::transform

class AdvancedPolynomialFitter {
public:
    /**
     * @brief Fits a polynomial to the given data points using a superposition Sequential Minimal Optimization (superposSMO) approach.
     *
     * This method first performs an initial polynomial fit using QR decomposition.
     * Then, it iteratively refines the polynomial coefficients using a weighted local
     * optimization method (SMO-like) to reduce the impact of potential outliers or
     * points with large residuals.
     *
     * @param x The vector of x-coordinates of the data points.
     * @param y The vector of y-coordinates of the data points.
     * @param degree The degree of the polynomial to fit.
     * @return A vector of floats representing the polynomial coefficients, starting from the coefficient of x^0.
     *
     * @note The algorithm is named "superposSMO" to reflect its combination of initial QR-based fitting
     *       with a subsequent Sequential Minimal Optimization-inspired refinement process. The "superposition"
     *       aspect refers to the iterative refinement on top of the initial global fit.
     */
    std::vector<float> fitPolynomial_superposSMO(
        const std::vector<float>& x, const std::vector<float>& y, int degree) {

        if (x.size() != y.size()) {
            throw std::invalid_argument("Input vectors x and y must have the same size.");
        }
        if (x.empty()) {
            return {}; // Return empty coefficients for empty input
        }
        if (degree < 0) {
            throw std::invalid_argument("Polynomial degree must be non-negative.");
        }

        const size_t n = x.size();
        const size_t m = degree + 1; // Number of coefficients
        const double tolerance = 1e-6;
        const int maxIterations = 20;
        const double residualThreshold = 1e-3;

        // 1. Precompute powers of x for efficiency
        std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 1; j < m; ++j) {
                xPowers[i][j] = xPowers[i][j - 1] * x[i];
            }
        }

        // 2. Calculate ATA and ATy for normal equations (for initial QR solution)
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += xPowers[i][j] * y[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA[j][k] += xPowers[i][j] * xPowers[i][k];
                }
            }
        }
        for (size_t j = 0; j < m; ++j) { // Ensure symmetry of ATA
            for (size_t k = j + 1; k < m; ++k) {
                ATA[j][k] = ATA[k][j];
            }
        }

        // 3. Solve initial polynomial coefficients using QR decomposition
        std::vector<double> coefficients = solveQR(ATA, ATy);

        // 4. Iterative Superposition SMO refinement
        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<double> residuals(n, 0.0);
            for (size_t i = 0; i < n; ++i) {
                double fittedValue = 0.0;
                for (size_t j = 0; j < m; ++j) {
                    fittedValue += coefficients[j] * xPowers[i][j];
                }
                residuals[i] = y[i] - fittedValue;
            }

            std::vector<size_t> problemIndices;
            for (size_t i = 0; i < n; ++i) {
                if (std::abs(residuals[i]) > residualThreshold) {
                    problemIndices.push_back(i);
                }
            }

            if (problemIndices.empty()) {
                break; // No more problem points, converge
            }

            bool anyCoefficientUpdated = false;
            // Iterate through all pairs of coefficients for local SMO update
            for (size_t p = 0; p < m; ++p) {
                for (size_t q = p + 1; q < m; ++q) {
                    std::pair<double, double> updatedPair = solveLocalSMO(
                        coefficients, p, q, xPowers, y, problemIndices);

                    if (std::abs(updatedPair.first - coefficients[p]) > tolerance ||
                        std::abs(updatedPair.second - coefficients[q]) > tolerance) {
                        coefficients[p] = updatedPair.first;
                        coefficients[q] = updatedPair.second;
                        anyCoefficientUpdated = true;
                    }
                }
            }
            if (!anyCoefficientUpdated) {
                break; // No coefficient updated significantly, converge
            }
        }

        // 5. Return the coefficients as vector<float>
        return std::vector<float>(coefficients.begin(), coefficients.end());
    }

private:
    /**
     * @brief Solves a 2x2 weighted least squares problem to locally optimize two polynomial coefficients.
     *
     * @param coeffs Current polynomial coefficients.
     * @param p Index of the first coefficient to optimize.
     * @param q Index of the second coefficient to optimize (q > p).
     * @param xPowers Precomputed powers of x values.
     * @param y Target y values.
     * @param problemIndices Indices of data points with residuals above a threshold.
     * @return A pair of doubles representing the updated coefficients for index p and q.
     */
    std::pair<double, double> solveLocalSMO(
        const std::vector<double>& coeffs, size_t p, size_t q,
        const std::vector<std::vector<double>>& xPowers,
        const std::vector<float>& y, const std::vector<size_t>& problemIndices) {

        double Apq = 0.0, App = 0.0, Aqq = 0.0, bp = 0.0, bq = 0.0;
        double weightSum = 0.0;

        for (size_t i : problemIndices) {
            // Weighting based on residual magnitude - focusing on larger residuals.
            // Using squared residual for weighting might be more standard or explore Huber loss.
            double residual = y[i] - std::inner_product(coeffs.begin(), coeffs.end(), xPowers[i].begin(), 0.0);
            double weight = std::exp(-std::abs(residual)); // Or try: weight = 1.0 / (1.0 + residual * residual);

            weightSum += weight;
            App += weight * xPowers[i][p] * xPowers[i][p]; // Corrected: App instead of Aq
            Aqq += weight * xPowers[i][q] * xPowers[i][q];
            Apq += weight * xPowers[i][p] * xPowers[i][q];
            bp += weight * xPowers[i][p] * y[i];
            bq += weight * xPowers[i][q] * y[i];
        }

        // Robustness checks for singular matrix cases
        if (std::abs(App) < std::numeric_limits<double>::epsilon() ||
            std::abs(Aqq) < std::numeric_limits<double>::epsilon()) {
            return {coeffs[p], coeffs[q]}; // Avoid division by zero
        }

        double det = App * Aqq - Apq * Apq;
        if (std::abs(det) < std::numeric_limits<double>::epsilon()) {
            return {coeffs[p], coeffs[q]}; // Matrix is singular or close to singular
        }

        // Solve 2x2 system using Cramer's rule or direct inversion
        double newP = (Aqq * bp - Apq * bq) / det;
        double newQ = (App * bq - Apq * bp) / det;

        return {newP, newQ};
    }


    /**
     * @brief Solves a linear system Ax = b using QR decomposition via Householder reflections.
     *
     * This method modifies the input matrix A and vector b in place to store the
     * intermediate steps of the QR decomposition.
     *
     * @param A The matrix A (modified in place to store R).
     * @param b The vector b (modified in place to store transformed b).
     * @return The solution vector x.
     * @note Implements Householder QR decomposition for solving Ax=b. Matrix A is assumed to be square.
     */
    std::vector<double> solveQR(std::vector<std::vector<double>>& A, std::vector<double>& b) {

        const size_t n = A.size();
        const double eps = std::numeric_limits<double>::epsilon();

        for (size_t k = 0; k < n; ++k) {
            // 1. Compute the norm of the k-th column from row k downwards
            double norm_x = 0.0;
            for (size_t i = k; i < n; ++i) {
                norm_x += A[i][k] * A[i][k];
            }
            norm_x = std::sqrt(std::max(norm_x, eps));

            // 2. Choose alpha and compute intermediate values
            double alpha = (A[k][k] > 0) ? -norm_x : norm_x;
            double r = std::sqrt(std::max(0.5 * (alpha * alpha - A[k][k] * alpha), eps));

            // 3. Compute Householder vector v
            std::vector<double> v(n, 0.0);
            v[k] = (A[k][k] - alpha) / (2 * r);
            for (size_t i = k + 1; i < n; ++i) {
                v[i] = A[i][k] / (2 * r);
            }

            // 4. Apply Householder reflection to matrix A (for columns k to n-1)
            for (size_t j = k; j < n; ++j) {
                double dotProduct = 0.0;
                for (size_t i = k; i < n; ++i) {
                    dotProduct += v[i] * A[i][j];
                }
                for (size_t i = k; i < n; ++i) {
                    A[i][j] -= 2 * v[i] * dotProduct;
                }
            }

            // 5. Apply Householder reflection to vector b
            double dotProduct_b = 0.0;
            for (size_t i = k; i < n; ++i) {
                dotProduct_b += v[i] * b[i];
            }
            for (size_t i = k; i < n; ++i) {
                b[i] -= 2 * v[i] * dotProduct_b;
            }

            // 6. Update A[k][k] and set sub-diagonal elements to zero (for upper triangular R)
            A[k][k] = alpha;
            for (size_t i = k + 1; i < n; ++i) {
                A[i][k] = 0.0;
            }
        }

        // 7. Back substitution to solve Rx = b
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= (std::abs(A[i][i]) > eps) ? A[i][i] : eps; // Avoid division by tiny values
        }
        return x;
    }
};
