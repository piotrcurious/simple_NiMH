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
    std::vector<float> fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree,
                                     OptimizationMethod method = GRADIENT_DESCENT);              

                                                                                          
    std::vector<float> fitSegmentedPolynomials(const std::vector<float>& x, const std::vector<float>& y, int degree, int segments);
    std::vector<float> levenbergMarquardt(std::vector<float>& coeffs, const std::vector<float>& x, const std::vector<float>& y, int degree);

private:
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b);
    std::vector<double>           solveQR(std::vector<std::vector<double>>& A, std::vector<double>& b);

//svm helper functions

int examineExample(size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
                 std::vector<float>& f, float epsilon, float C,
                 const std::vector<std::vector<float>>& K, float tol, double& b);

int optimizePair(size_t i1, size_t i2, size_t n, std::vector<float>& alpha, const std::vector<float>& y,
                std::vector<float>& f, float epsilon, float C,
                const std::vector<std::vector<float>>& K, float tol, double& b);

std::vector<double> denormalizeCoefficients(const std::vector<double>& w, double x_min, double x_range, int degree);

double binomialCoefficient(int n, int k);
    
};

#endif
