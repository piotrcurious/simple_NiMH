#include "AdvancedPolynomialFitter.hpp"
#include <algorithm>
#include <cmath>

    // calculate using Welford's method
    double AdvancedPolynomialFitter::calculateMSE(const std::vector<float>& coeffs, 
                   const std::vector<float>& x, 
                   const std::vector<float>& y) {
    double meanSquaredError = 0.0;
    double mean = 0.0;
    double M2 = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        float prediction = 0.0;
        for (size_t j = 0; j < coeffs.size(); ++j) {
            prediction += coeffs[j] * pow(x[i], j);
        }
        float error = prediction - y[i];
        double squaredError = error * error;
        double delta = squaredError - mean;
        mean += delta / (i + 1);
        M2 += delta * (squaredError - mean);
    }

    meanSquaredError = mean;
    //Serial.println(meanSquaredError);
    return meanSquaredError;
}

    // Fit a polynomial to the data using the normal equation
    std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method ) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

       // Normalize x and y
//        std::vector<float> x_norm = x;
        double x_min = *std::min_element(x.begin(), x.end());
        Serial.print("xmin:");
        Serial.print(x_min);
        double x_max = *std::max_element(x.begin(), x.end());
        Serial.print(" xmax:");
        Serial.println(x_max);

//#ifdef REVERSED_NORMALIZATION       
//        std::transform(x.begin(), x.end(), x_norm.begin(), [x_max](double val) { return val -x_max; });
//#else  
//        std::transform(x.begin(), x.end(), x_norm.begin(), [x_min](double val) { return val -x_min; });
//#endif //#ifdef REVERSED_NORMALIZATION       

       // double y_max = *std::max_element(y.begin(), y.end());
//        std::transform(y.begin(), y.end(), y_norm.begin(), [y_max](double val) { return val / y_max; });

        Serial.println(x[0]); 
        Serial.println(x[x.size()-1]);

        size_t n = x.size();
        size_t m = degree + 1;

        // Construct the Vandermonde matrix
        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A[i][j] = xi;
                xi *= x[i];
            }
        }

        // Construct the normal equation: (A^T * A) * coeffs = A^T * y
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += A[i][j] * y[i];
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += A[i][j] * A[i][k];
                }
            }
        }

        // Solve the normal equation using Gaussian elimination
        std::vector<double> coeffs = solveLinearSystem(ATA, ATy);

        // Convert coefficients to float
        std::vector<float> result(coeffs.begin(), coeffs.end());

        switch (method) {
        case GRADIENT_DESCENT:
            // Implement gradient descent here if needed
            break;
        case LEVENBERG_MARQUARDT:
            result = levenbergMarquardt(result, x, y, degree);
            break;
        case NELDER_MEAD:
            // Implement Nelder-Mead here if needed
            break;
        default: // no optimization
            //result = levenbergMarquardt(x_norm, y, degree);
            break;
    }
        
        return result;
    }


#include <vector>
#include <cmath>
#include <limits>
#include <numeric>
#include <algorithm>
#include <utility>


std::vector<float> AdvancedPolynomialFitter::fitPolynomial_superpos5c(
    const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method  ) {


        if (x.size() != y.size()) {
            //throw std::invalid_argument("Input vectors x and y must have the same size.");
        }
        if (x.empty()) {
            return {};
        }
        if (degree < 0) {
            //throw std::invalid_argument("Polynomial degree must be non-negative.");
        }

        const size_t n = x.size();
        const size_t m = degree + 1;
        const double tolerance = 1e-7; // Reduced tolerance for early stopping check
        const int maxIterations = 20;
        const double residualThreshold = 0.2; // Further reduced residual threshold
        const double smo_dampening_factor = 0.5; // Dampening factor for SMO updates

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
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += xPowers[i][j] * xPowers[i][k];
                }
            }
        }

        std::vector<double> coefficients = solveQR(ATA, ATy);
        std::vector<double> prev_coefficients = coefficients; // For early stopping


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
            double max_coeff_change = 0.0; // For early stopping

            for (size_t p = 0; p < m; ++p) {
                for (size_t q = p + 1; q < m; ++q) {
                    std::pair<double, double> updatedPair = solveLocalSMO(
                        coefficients, p, q, xPowers, y, problemIndices);

                    if (std::abs(updatedPair.first - coefficients[p]) > tolerance ||
                        std::abs(updatedPair.second - coefficients[q]) > tolerance) {
                        max_coeff_change = std::max({max_coeff_change, std::abs(updatedPair.first - coefficients[p]), std::abs(updatedPair.second - coefficients[q])});

                        coefficients[p] = (1.0 - smo_dampening_factor) * coefficients[p] + smo_dampening_factor * updatedPair.first; // Dampened update
                        coefficients[q] = (1.0 - smo_dampening_factor) * coefficients[q] + smo_dampening_factor * updatedPair.second; // Dampened update

                        anyCoefficientUpdated = true;
                    }
                }
            }
            if (!anyCoefficientUpdated) {
                break; // No coefficient updated significantly, converge
            }

            bool converged = true;
            for(size_t j = 0; j < m; ++j) {
                if(std::abs(coefficients[j] - prev_coefficients[j]) > tolerance) {
                    converged = false;
                    break;
                }
            }
            if (converged) {
            //    std::cout << "Converged early at iteration: " << iter << std::endl;
                break; // Early stopping based on coefficient change
            }
            prev_coefficients = coefficients; // Update for next iteration's convergence check

             if (max_coeff_change < tolerance) {
                 //std::cout << "Converged early due to small coeff change at iteration: " << iter << std::endl;
                 break; // Early stop if max coefficient change is very small
             }
        }

        // 5. Return the coefficients as vector<float>
        std::vector<float> floatCoefficients(coefficients.begin(), coefficients.end());
        return floatCoefficients;

}

std::pair<double, double> AdvancedPolynomialFitter::solveLocalSMO(
    const std::vector<double>& coeffs, size_t p, size_t q,
    const std::vector<std::vector<double>>& xPowers, 
    const std::vector<float>& y, const std::vector<size_t>& problemIndices) {


      double App = 0.0, Aqq = 0.0, Apq = 0.0, bp = 0.0, bq = 0.0;

        for (size_t i : problemIndices) {
            double residual = y[i] - std::inner_product(coeffs.begin(), coeffs.end(), xPowers[i].begin(), 0.0);
            // More aggressive weighting (inverse fourth power)
            double weight = 1.0 / (1.0 + residual * residual * residual * residual);
            double base_fitted_i = 0.0;

            for (size_t j = 0; j < coeffs.size(); ++j) {
                if (j != p && j != q) {
                    base_fitted_i += coeffs[j] * xPowers[i][j];
                }
            }
            double target_i = y[i] - base_fitted_i;


            App += weight * xPowers[i][p] * xPowers[i][p];
            Aqq += weight * xPowers[i][q] * xPowers[i][q];
            Apq += weight * xPowers[i][p] * xPowers[i][q];
            bp  += weight * xPowers[i][p] * target_i;
            bq  += weight * xPowers[i][q] * target_i;
        }


        if (std::abs(App) < std::numeric_limits<double>::epsilon() ||
            std::abs(Aqq) < std::numeric_limits<double>::epsilon()) {
            return {coeffs[p], coeffs[q]};
        }

        double det = App * Aqq - Apq * Apq;
        if (std::abs(det) < std::numeric_limits<double>::epsilon()) {
            return {coeffs[p], coeffs[q]};
        }

        double updated_coeffs_p = (Aqq * bp - Apq * bq) / det;
        double updated_coeffs_q = (App * bq - Apq * bp) / det;

        return {updated_coeffs_p, updated_coeffs_q};
}



// QR decomposition solver (previous implementation remains largely the same)
   std::vector<double> AdvancedPolynomialFitter::solveQR(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {

     const size_t n = A.size();
        std::vector<double> x(n);

        for (size_t k = 0; k < n; ++k) {
            double norm_x = 0.0;
            for (size_t i = k; i < n; ++i) {
                norm_x += A[i][k] * A[i][k];
            }
            norm_x = std::sqrt(norm_x);

            double alpha = -std::copysign(norm_x, A[k][k]);
            double r = 0.5 * std::sqrt(2.0 * (alpha * alpha - alpha * A[k][k]));


            std::vector<double> v(n, 0.0);
            v[k] = (A[k][k] - alpha) ;
            for (size_t i = k + 1; i < n; ++i) {
                v[i] = A[i][k];
            }


            if (r != 0) { // Avoid division by zero if r is zero, which means no reflection needed
                for (size_t i = 0; i < n; ++i) {
                     v[i] /= (2.0 * r); // Normalize v by 2r only when r is not zero
                }


                for (size_t j = k; j < n; ++j) {
                    double dotProduct = 0.0;
                    for (size_t i = k; i < n; ++i) {
                        dotProduct += v[i] * A[i][j];
                    }
                    for (size_t i = k; i < n; ++i) {
                        A[i][j] -= 2.0 * dotProduct * v[i];
                    }
                }

                double dotProduct_b = 0.0;
                for (size_t i = k; i < n; ++i) {
                    dotProduct_b += v[i] * b[i];
                }
                for (size_t i = k; i < n; ++i) {
                    b[i] -= 2.0 * dotProduct_b * v[i];
                }
            }
            A[k][k] = alpha;
             for (size_t i = k + 1; i < n; ++i) {
                A[i][k] = 0.0; // Ensure sub-diagonal elements are exactly zero for upper triangular form
            }
        }


        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            for (size_t j = i + 1; j < n; ++j) {
                sum += A[i][j] * x[j];
            }
            x[i] = (b[i] - sum) / A[i][i];
        }
        return x;
    }
  




// Fit segmented polynomials
std::vector<float> AdvancedPolynomialFitter::fitSegmentedPolynomials(const std::vector<float>& x, const std::vector<float>& y, int degree, int segments) {
    std::vector<float> result;
    size_t segmentSize = x.size() / segments;
    
    for (int i = 0; i < segments; ++i) {
        size_t startIdx = i * segmentSize;
        size_t endIdx = (i == segments - 1) ? x.size() : (i + 1) * segmentSize;
        
        std::vector<float> x_segment(x.begin() + startIdx, x.begin() + endIdx);
        std::vector<float> y_segment(y.begin() + startIdx, y.begin() + endIdx);
        
        std::vector<float> segmentCoeffs = fitPolynomial(x_segment, y_segment, degree);
        result.insert(result.end(), segmentCoeffs.begin(), segmentCoeffs.end());
    }
    
    return result;
}

// Implement Levenberg-Marquardt algorithm
std::vector<float> AdvancedPolynomialFitter::levenbergMarquardt(std::vector<float>& coeffs,const std::vector<float>& x, const std::vector<float>& y, int degree) {
    const int maxIterations = 200; // Maximum number of iterations
    const double lambdaInit = 0.1; // Initial lambda value
    const double lambdaFactor = 2; // Factor to increase/decrease lambda
    const double tolerance = 1e-6; // Convergence tolerance

    //std::vector<float> coeffs(degree + 1, 0.0); // Initial coefficients
    double lambda = lambdaInit;
    double prevMSE = calculateMSE(coeffs, x, y);

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Compute the Jacobian matrix and residuals
        std::vector<std::vector<double>> J(x.size(), std::vector<double>(degree + 1, 0.0));
        std::vector<double> residuals(x.size(), 0.0);
        
        for (size_t i = 0; i < x.size(); ++i) {
            double xi = 1.0;
            for (int j = 0; j <= degree; ++j) {
                J[i][j] = xi;
                xi *= x[i];
            }
            double prediction = 0.0;
            for (int j = 0; j <= degree; ++j) {
                prediction += coeffs[j] * pow(x[i], j);
            }
            residuals[i] = y[i] - prediction;
        }

        // Compute the normal equations
        std::vector<std::vector<double>> JTJ(degree + 1, std::vector<double>(degree + 1, 0.0));
        std::vector<double> JTr(degree + 1, 0.0);
        
        for (size_t i = 0; i < x.size(); ++i) {
            for (int j = 0; j <= degree; ++j) {
                for (int k = 0; k <= degree; ++k) {
                    JTJ[j][k] += J[i][j] * J[i][k];
                }
                JTr[j] += J[i][j] * residuals[i];
            }
        }

        // Add the damping factor to the diagonal elements
        for (int j = 0; j <= degree; ++j) {
            JTJ[j][j] += lambda;
        }

        // Solve for the parameter update
        std::vector<double> delta = solveLinearSystem(JTJ, JTr);

        // Update the coefficients
        std::vector<float> newCoeffs = coeffs;
        for (int j = 0; j <= degree; ++j) {
            newCoeffs[j] += delta[j];
        }

        double newMSE = calculateMSE(newCoeffs, x, y);
        
        // Check for convergence
        if (std::abs(prevMSE - newMSE) < tolerance) {
            break;
        }

        // Update lambda and coefficients based on the new MSE
        if (newMSE < prevMSE) {
            lambda /= lambdaFactor;
            Serial.println(lambda);
            coeffs = newCoeffs;
            prevMSE = newMSE;
        } else {
            lambda *= lambdaFactor;
        }
    }

    return coeffs;
}

    // Solve a linear system using Gaussian elimination
    std::vector<double> AdvancedPolynomialFitter::solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b) {
        size_t n = A.size();

        // Forward elimination
        for (size_t k = 0; k < n; ++k) {
            // Pivot for numerical stability
            for (size_t i = k + 1; i < n; ++i) {
                if (fabs(A[i][k]) > fabs(A[k][k])) {
                    std::swap(A[k], A[i]);
                    std::swap(b[k], b[i]);
                }
            }

            for (size_t i = k + 1; i < n; ++i) {
                double factor = A[i][k] / A[k][k];
                for (size_t j = k; j < n; ++j) {
                    A[i][j] -= factor * A[k][j];
                }
                b[i] -= factor * b[k];
            }
        }

        // Back substitution
        std::vector<double> x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }
