#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric>
#include <algorithm>

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
    std::vector<double> fitPolynomial_superposSMO(
        const std::vector<double>& x, const std::vector<double>& y, int degree) {

        // Input validation
        if (x.size() != y.size()) {
            throw std::invalid_argument("Input vectors x and y must have the same size.");
        }
        if (x.empty()) {
            return {}; // Return empty coefficients for empty input
        }
        if (degree < 0) {
            throw std::invalid_argument("Polynomial degree must be non-negative.");
        }
        if (static_cast<size_t>(degree) >= x.size()) {
            // Prevent overfitting with too high degree
            degree = static_cast<int>(x.size()) - 1;
            std::cerr << "Warning: Degree reduced to " << degree << " to prevent overfitting.\n";
        }

        const size_t n = x.size();
        const size_t m = degree + 1; // Number of coefficients
        
        // Algorithm parameters
        const double tolerance = 1e-10;
        const int maxIterations = 50;
        const double residualThreshold = 1e-3;

        // 1. Precompute powers of x for efficiency
        std::vector<std::vector<double>> xPowers(n, std::vector<double>(m, 1.0));
        computePowers(x, xPowers, m);

        // 2. Calculate ATA and ATy for normal equations (for initial QR solution)
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);
        
        computeNormalEquations(xPowers, y, ATA, ATy, n, m);

        // 3. Solve initial polynomial coefficients using QR decomposition
        std::vector<double> coefficients = solveQR(ATA, ATy);

        // 4. Iterative Superposition SMO refinement
        if (!refineCoefficients(xPowers, y, coefficients, maxIterations, 
                               residualThreshold, tolerance, n, m)) {
            std::cerr << "Warning: SMO refinement may not have fully converged.\n";
        }

        return coefficients;
    }

private:
    /**
     * @brief Computes x raised to powers 0 through m-1 for all data points.
     */
    void computePowers(const std::vector<double>& x, 
                      std::vector<std::vector<double>>& xPowers, 
                      size_t m) const {
        for (size_t i = 0; i < x.size(); ++i) {
            for (size_t j = 1; j < m; ++j) {
                xPowers[i][j] = xPowers[i][j - 1] * x[i];
            }
        }
    }

    /**
     * @brief Computes ATA and ATy matrices for normal equations.
     */
    void computeNormalEquations(const std::vector<std::vector<double>>& xPowers,
                               const std::vector<double>& y,
                               std::vector<std::vector<double>>& ATA,
                               std::vector<double>& ATy,
                               size_t n, size_t m) const {
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += xPowers[i][j] * y[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA[j][k] += xPowers[i][j] * xPowers[i][k];
                }
            }
        }
        
        // Ensure symmetry of ATA
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA[j][k] = ATA[k][j];
            }
        }
    }

    /**
     * @brief Iteratively refines coefficients using Sequential Minimal Optimization.
     * 
     * @return true if converged, false if max iterations reached
     */
    bool refineCoefficients(const std::vector<std::vector<double>>& xPowers,
                           const std::vector<double>& y,
                           std::vector<double>& coefficients,
                           int maxIterations,
                           double residualThreshold,
                           double tolerance,
                           size_t n, size_t m) {
        for (int iter = 0; iter < maxIterations; ++iter) {
            // Calculate current residuals
            std::vector<double> residuals = calculateResiduals(xPowers, y, coefficients, n, m);
            
            // Find indices of data points with large residuals
            std::vector<size_t> problemIndices = identifyProblemPoints(residuals, residualThreshold);
            
            if (problemIndices.empty()) {
                return true; // No more problem points, converged
            }

            // Track if any coefficient was significantly updated
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
                return true; // No coefficient updated significantly, converged
            }
        }
        
        return false; // Max iterations reached without convergence
    }

    /**
     * @brief Calculates residuals for the current model.
     */
    std::vector<double> calculateResiduals(const std::vector<std::vector<double>>& xPowers,
                                         const std::vector<double>& y,
                                         const std::vector<double>& coefficients,
                                         size_t n, size_t m) const {
        std::vector<double> residuals(n);
        for (size_t i = 0; i < n; ++i) {
            double fittedValue = 0.0;
            for (size_t j = 0; j < m; ++j) {
                fittedValue += coefficients[j] * xPowers[i][j];
            }
            residuals[i] = y[i] - fittedValue;
        }
        return residuals;
    }

    /**
     * @brief Identifies data points with residuals above threshold.
     */
    std::vector<size_t> identifyProblemPoints(const std::vector<double>& residuals,
                                            double residualThreshold) const {
        std::vector<size_t> problemIndices;
        for (size_t i = 0; i < residuals.size(); ++i) {
            if (std::abs(residuals[i]) > residualThreshold) {
                problemIndices.push_back(i);
            }
        }
        return problemIndices;
    }

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
        const std::vector<double>& y, const std::vector<size_t>& problemIndices) const {

        double Apq = 0.0, App = 0.0, Aqq = 0.0, bp = 0.0, bq = 0.0;
        double weightSum = 0.0;

        // Calculate contribution from each problem point
        for (size_t idx : problemIndices) {
            // Calculate residual (target - prediction)
            double prediction = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                prediction += coeffs[j] * xPowers[idx][j];
            }
            double residual = y[idx] - prediction;
            
            // Apply Huber-like weighting function for robustness
            double weight = huberWeight(residual);
            
            weightSum += weight;
            App += weight * xPowers[idx][p] * xPowers[idx][p];
            Aqq += weight * xPowers[idx][q] * xPowers[idx][q];
            Apq += weight * xPowers[idx][p] * xPowers[idx][q];
            
            // Calculate weighted contributions to right-hand side
            // Account for all terms except p and q in the prediction
            double partialPrediction = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                if (j != p && j != q) {
                    partialPrediction += coeffs[j] * xPowers[idx][j];
                }
            }
            double partialResidual = y[idx] - partialPrediction;
            
            bp += weight * xPowers[idx][p] * partialResidual;
            bq += weight * xPowers[idx][q] * partialResidual;
        }

        // Numerical stability: ensure matrix is well-conditioned
        const double epsilon = std::numeric_limits<double>::epsilon() * 1e4;
        
        if (weightSum < epsilon || 
            std::abs(App) < epsilon ||
            std::abs(Aqq) < epsilon) {
            return {coeffs[p], coeffs[q]}; // Avoid division by near-zero
        }

        double det = App * Aqq - Apq * Apq;
        if (std::abs(det) < epsilon * std::max(App, Aqq)) {
            // Apply regularization for near-singular cases
            double lambda = 1e-6 * std::max(App, Aqq);
            App += lambda;
            Aqq += lambda;
            det = App * Aqq - Apq * Apq;
        }

        // Solve 2x2 system
        double newP = (Aqq * bp - Apq * bq) / det;
        double newQ = (App * bq - Apq * bp) / det;

        return {newP, newQ};
    }
    
    /**
     * @brief Computes Huber-like weight for robust regression
     */
    double huberWeight(double residual, double delta = 1.0) const {
        double absRes = std::abs(residual);
        if (absRes <= delta) {
            return 1.0; // Full weight for small residuals
        } else {
            return delta / absRes; // Reduced weight for outliers
        }
    }

    /**
     * @brief Solves a linear system Ax = b using QR decomposition via Householder reflections.
     *
     * This method implements a numerically stable QR decomposition to solve the system.
     *
     * @param A The coefficient matrix (symmetric positive definite)
     * @param b The right-hand side vector
     * @return The solution vector x
     */
    std::vector<double> solveQR(const std::vector<std::vector<double>>& A, 
                              const std::vector<double>& b) const {
        const size_t n = A.size();
        const double eps = std::numeric_limits<double>::epsilon() * 1e2;
        
        // Create working copies of A and b
        std::vector<std::vector<double>> R = A;
        std::vector<double> QTb = b;

        // Create Q explicitly for numerical stability
        std::vector<std::vector<double>> Q(n, std::vector<double>(n, 0.0));
        for (size_t i = 0; i < n; ++i) {
            Q[i][i] = 1.0; // Initialize Q as identity matrix
        }

        // QR factorization using Householder reflections
        for (size_t k = 0; k < n; ++k) {
            // Extract column k of R
            std::vector<double> x(n - k);
            for (size_t i = k; i < n; ++i) {
                x[i - k] = R[i][k];
            }
            
            // Compute Householder vector v
            double xNorm = 0.0;
            for (const double& val : x) {
                xNorm += val * val;
            }
            xNorm = std::sqrt(xNorm);
            
            if (xNorm < eps) {
                continue; // Skip if column is already close to zero
            }
            
            // Choose sign for numerical stability
            double alpha = (x[0] >= 0) ? -xNorm : xNorm;
            double u1 = x[0] - alpha;
            double vNorm = u1 * u1;
            
            for (size_t i = 1; i < x.size(); ++i) {
                vNorm += x[i] * x[i];
            }
            
            if (vNorm < eps) {
                continue; // Skip if Householder vector is near zero
            }
            
            vNorm = std::sqrt(vNorm);
            
            // Normalize Householder vector
            std::vector<double> v(n, 0.0);
            v[k] = u1 / vNorm;
            for (size_t i = 1; i < x.size(); ++i) {
                v[i + k] = x[i] / vNorm;
            }
            
            // Apply H to R: R = H * R
            for (size_t j = k; j < n; ++j) {
                double dotProduct = 0.0;
                for (size_t i = k; i < n; ++i) {
                    dotProduct += v[i] * R[i][j];
                }
                
                for (size_t i = k; i < n; ++i) {
                    R[i][j] -= 2.0 * v[i] * dotProduct;
                }
            }
            
            // Apply H to Q: Q = Q * H
            for (size_t i = 0; i < n; ++i) {
                double dotProduct = 0.0;
                for (size_t j = k; j < n; ++j) {
                    dotProduct += Q[i][j] * v[j];
                }
                
                for (size_t j = k; j < n; ++j) {
                    Q[i][j] -= 2.0 * dotProduct * v[j];
                }
            }
            
            // Apply H to b: QTb = H * b
            double dotProduct = 0.0;
            for (size_t i = k; i < n; ++i) {
                dotProduct += v[i] * QTb[i];
            }
            
            for (size_t i = k; i < n; ++i) {
                QTb[i] -= 2.0 * v[i] * dotProduct;
            }
        }
        
        // Back substitution to solve Rx = QTb
        std::vector<double> x(n, 0.0);
        for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
            double sum = QTb[i];
            for (size_t j = i + 1; j < n; ++j) {
                sum -= R[i][j] * x[j];
            }
            
            if (std::abs(R[i][i]) > eps) {
                x[i] = sum / R[i][i];
            } else {
                // Handle rank deficiency with regularization
                x[i] = sum / (R[i][i] + eps);
            }
        }
        
        return x;
    }
    
    /**
     * @brief Calculate the quality of fit (R-squared)
     *
     * @param x Input x values
     * @param y Input y values
     * @param coefficients Polynomial coefficients
     * @return R-squared value (1.0 is perfect fit)
     */
    double calculateRSquared(const std::vector<double>& x, 
                           const std::vector<double>& y,
                           const std::vector<double>& coefficients) const {
        if (x.empty() || y.empty() || coefficients.empty()) {
            return 0.0;
        }
        
        // Calculate mean of y
        double yMean = std::accumulate(y.begin(), y.end(), 0.0) / y.size();
        
        // Calculate total sum of squares
        double ssTot = 0.0;
        for (const double& yi : y) {
            double diff = yi - yMean;
            ssTot += diff * diff;
        }
        
        if (ssTot < std::numeric_limits<double>::epsilon()) {
            return 1.0; // If all y values are identical, perfect fit
        }
        
        // Calculate residual sum of squares
        double ssRes = 0.0;
        for (size_t i = 0; i < x.size(); ++i) {
            double pred = 0.0;
            for (size_t j = 0; j < coefficients.size(); ++j) {
                pred += coefficients[j] * std::pow(x[i], static_cast<double>(j));
            }
            double diff = y[i] - pred;
            ssRes += diff * diff;
        }
        
        return 1.0 - (ssRes / ssTot);
    }
    
public:
    /**
     * @brief Evaluates the polynomial at a given x value
     *
     * @param coefficients Polynomial coefficients [a₀, a₁, a₂, ...]
     * @param x The x value to evaluate P(x) = a₀ + a₁x + a₂x² + ...
     * @return The polynomial value at x
     */
    static double evaluatePolynomial(const std::vector<double>& coefficients, double x) {
        double result = 0.0;
        double xPower = 1.0;
        
        for (const double& coef : coefficients) {
            result += coef * xPower;
            xPower *= x;
        }
        
        return result;
    }
};
