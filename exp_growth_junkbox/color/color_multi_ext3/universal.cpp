#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

class EnhancedPolynomialFitter {
public:
    // Enumeration for different basis function types
    enum BasisType {
        MONOMIAL,
        CHEBYSHEV,
        LEGENDRE,
        RATIONAL
    };

    struct FittingResult {
        std::vector<float> numeratorCoeffs;
        std::vector<float> denominatorCoeffs;  // Empty for non-rational fits
        float condition_number;
        float residual_error;
    };

private:
    // Chebyshev polynomial evaluation
    static double evaluateChebyshev(double x, int degree) {
        if (degree == 0) return 1.0;
        if (degree == 1) return x;
        return 2.0 * x * evaluateChebyshev(x, degree - 1) - evaluateChebyshev(x, degree - 2);
    }

    // Transform data to [-1, 1] interval
    static std::vector<double> normalizeInterval(const std::vector<float>& x) {
        double x_min = *std::min_element(x.begin(), x.end());
        double x_max = *std::max_element(x.begin(), x.end());
        double scale = 2.0 / (x_max - x_min);
        
        std::vector<double> x_normalized(x.size());
        std::transform(x.begin(), x.end(), x_normalized.begin(),
            [x_min, scale](float val) { return scale * (val - x_min) - 1.0; });
        return x_normalized;
    }

    // Compute basis functions matrix
    static std::vector<std::vector<double>> computeBasisMatrix(
        const std::vector<double>& x_norm, 
        int degree,
        BasisType basis_type) {
        
        size_t n = x_norm.size();
        std::vector<std::vector<double>> basis_matrix(n, std::vector<double>(degree + 1));

        for (size_t i = 0; i < n; ++i) {
            switch (basis_type) {
                case CHEBYSHEV:
                    for (int j = 0; j <= degree; ++j) {
                        basis_matrix[i][j] = evaluateChebyshev(x_norm[i], j);
                    }
                    break;
                case LEGENDRE:
                    // Implement Legendre polynomials using recurrence relation
                    for (int j = 0; j <= degree; ++j) {
                        if (j == 0) basis_matrix[i][j] = 1.0;
                        else if (j == 1) basis_matrix[i][j] = x_norm[i];
                        else {
                            double p0 = basis_matrix[i][j-2];
                            double p1 = basis_matrix[i][j-1];
                            basis_matrix[i][j] = ((2.0 * j - 1.0) * x_norm[i] * p1 - 
                                                (j - 1.0) * p0) / j;
                        }
                    }
                    break;
                default:  // MONOMIAL
                    double xi = 1.0;
                    for (int j = 0; j <= degree; ++j) {
                        basis_matrix[i][j] = xi;
                        xi *= x_norm[i];
                    }
            }
        }
        return basis_matrix;
    }

    // Compute rational approximation using AAA algorithm
    static FittingResult computeRationalApproximation(
        const std::vector<double>& x_norm,
        const std::vector<float>& y,
        int num_degree,
        int den_degree) {
        
        size_t n = x_norm.size();
        std::vector<std::vector<double>> A(n, std::vector<double>(num_degree + den_degree + 2));
        std::vector<double> b(n);

        // Construct the system matrix for rational approximation
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (int j = 0; j <= num_degree; ++j) {
                A[i][j] = xi;
                xi *= x_norm[i];
            }
            
            xi = y[i];
            for (int j = 0; j <= den_degree; ++j) {
                A[i][num_degree + 1 + j] = -xi;
                xi *= x_norm[i];
            }
            b[i] = y[i];
        }

        // Solve using SVD or other robust method
        auto solution = solveLinearSystemSVD(A, b);
        
        FittingResult result;
        result.numeratorCoeffs = std::vector<float>(
            solution.begin(), 
            solution.begin() + num_degree + 1
        );
        result.denominatorCoeffs = std::vector<float>(
            solution.begin() + num_degree + 1,
            solution.end()
        );
        
        return result;
    }

public:
    static FittingResult fit(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree,
        BasisType basis_type = CHEBYSHEV,
        bool use_rational = true) {
        
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return FittingResult();
        }

        // Normalize input data to [-1, 1] interval
        std::vector<double> x_norm = normalizeInterval(x);

        if (use_rational) {
            // Use rational approximation with degree-1 denominator
            return computeRationalApproximation(x_norm, y, degree, degree/2);
        }

        // Compute basis functions matrix
        auto basis_matrix = computeBasisMatrix(x_norm, degree, basis_type);

        // Solve using QR decomposition for better numerical stability
        auto coeffs = solveLinearSystemQR(basis_matrix, y);
        
        FittingResult result;
        result.numeratorCoeffs = std::vector<float>(coeffs.begin(), coeffs.end());
        result.condition_number = computeConditionNumber(basis_matrix);
        result.residual_error = computeResidualError(basis_matrix, coeffs, y);
        
        return result;
    }

    static std::vector<float> evaluate(
        const FittingResult& result,
        const std::vector<float>& x,
        BasisType basis_type = CHEBYSHEV) {
        
        std::vector<double> x_norm = normalizeInterval(x);
        std::vector<float> y_pred(x.size());

        if (!result.denominatorCoeffs.empty()) {
            // Rational function evaluation
            for (size_t i = 0; i < x.size(); ++i) {
                double num = 0.0, den = 0.0;
                double xi = 1.0;
                
                for (float coeff : result.numeratorCoeffs) {
                    num += coeff * xi;
                    xi *= x_norm[i];
                }
                
                xi = 1.0;
                for (float coeff : result.denominatorCoeffs) {
                    den += coeff * xi;
                    xi *= x_norm[i];
                }
                
                y_pred[i] = static_cast<float>(num / den);
            }
        } else {
            // Regular polynomial evaluation using chosen basis
            auto basis_matrix = computeBasisMatrix(x_norm, 
                result.numeratorCoeffs.size() - 1, basis_type);
            
            for (size_t i = 0; i < x.size(); ++i) {
                double sum = 0.0;
                for (size_t j = 0; j < result.numeratorCoeffs.size(); ++j) {
                    sum += result.numeratorCoeffs[j] * basis_matrix[i][j];
                }
                y_pred[i] = static_cast<float>(sum);
            }
        }
        
        return y_pred;
    }
};
