#ifndef ADVANCED_POLYNOMIAL_FITTER_H
#define ADVANCED_POLYNOMIAL_FITTER_H

#include <Arduino.h>
#include <vector>

class AdvancedPolynomialFitter {
public:
    enum OptimizationMethod {
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD,
        NONE,
    };

    double calculateMSE(const std::vector<float>& coeffs, const std::vector<float>& x, const std::vector<float>& y);
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
                                     OptimizationMethod method = GRADIENT_DESCENT);
    std::vector<float> fitPolynomial_superpos5c(const std::vector<float>& x, const std::vector<float>& y, int degree,
                                     OptimizationMethod method = GRADIENT_DESCENT);                                 
    std::vector<float> fitSegmentedPolynomials(const std::vector<float>& x, const std::vector<float>& y, int degree, int segments);
    std::vector<float> levenbergMarquardt(std::vector<float>& coeffs, const std::vector<float>& x, const std::vector<float>& y, int degree);

private:
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b);
    std::vector<double>           solveQR(std::vector<std::vector<double>>& A, std::vector<double>& b);
/*    
    std::pair<double, double> solveLocalSMO(
    const std::vector<double>& coeffs,
    size_t p,
    size_t q,
    const std::vector<std::vector<double>>& xPowers,
    const std::vector<float>& y,
    const std::vector<size_t>& problemIndices);
*/

    // Local SMO update for a given pair of coefficients
    std::pair<double, double> solveLocalSMO(
        const std::vector<double>& coeffs,
        size_t p,
        size_t q,
        const std::vector<std::vector<double>>& xPowers,
        const std::vector<float>& y,
        const std::vector<size_t>& problemIndices
    );


};


#endif
