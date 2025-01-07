#ifndef POLYNOMIALFITTER_H
#define POLYNOMIALFITTER_H

#include <vector>

class PolynomialFitter {
public:
    // Optimization methods enum
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
    };

public:
    /**
     * Calculate the Mean Squared Error (MSE) for a given polynomial model.
     * @param coeffs Polynomial coefficients.
     * @param x X-data points.
     * @param y Y-data points.
     * @return Mean Squared Error.
     */
    double calculateMSE(const std::vector<float>& coeffs, const std::vector<float>& x, const std::vector<float>& y);

    /**
     * Fit a polynomial of a given degree to the data.
     * @param x X-data points.
     * @param y Y-data points.
     * @param degree Degree of the polynomial to fit.
     * @return Polynomial coefficients.
     */
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree, OptimizationMethod method );

private:
    /**
     * Solve a linear system of equations using Gaussian elimination.
     * @param A Coefficient matrix.
     * @param b Constant vector.
     * @return Solution vector.
     */
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b);
};

#endif // POLYNOMIALFITTER_H
