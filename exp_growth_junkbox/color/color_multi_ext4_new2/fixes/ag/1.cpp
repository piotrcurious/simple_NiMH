#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric> // for std::inner_product
#include <algorithm> // for std::abs, std::max, std::transform
#include <stdexcept> // for std::runtime_error

// Class for Finite Field elements in Fp (Z/pZ)
class FpElement {
public:
    int value; // Represent element in Z/pZ
    int p;     // Prime modulus

    FpElement(int v = 0, int prime_p = 2) : value(v % prime_p), p(prime_p) {
        if (value < 0) value += p; // Ensure value is in [0, p-1]
        if (p <= 1 || !isPrime(p)) {
            throw std::invalid_argument("Modulus p must be a prime number greater than 1.");
        }
    }

    FpElement operator+(const FpElement& other) const {
        if (p != other.p) throw std::invalid_argument("FpElements must have the same modulus for addition.");
        return FpElement((value + other.value) % p, p);
    }

    FpElement operator-(const FpElement& other) const {
        if (p != other.p) throw std::invalid_argument("FpElements must have the same modulus for subtraction.");
        return FpElement((value - other.value + p) % p, p); // Add p to handle negative result
    }

    FpElement operator*(const FpElement& other) const {
        if (p != other.p) throw std::invalid_argument("FpElements must have the same modulus for multiplication.");
        return FpElement((value * other.value) % p, p);
    }

    FpElement operator/(const FpElement& other) const {
        if (p != other.p) throw std::invalid_argument("FpElements must have the same modulus for division.");
        if (other.value == 0) throw std::runtime_error("Division by zero in FpElement.");
        int inv_val = modInverse(other.value, p);
        return FpElement((value * inv_val) % p, p);
    }

    bool operator==(const FpElement& other) const {
        if (p != other.p) return false;
        return value == other.value;
    }

    bool operator!=(const FpElement& other) const {
        return !(*this == other);
    }

    FpElement& operator=(const FpElement& other) {
        if (&other == this) return *this;
        if (p != other.p) throw std::invalid_argument("Cannot assign FpElements with different moduli.");
        value = other.value;
        p = other.p;
        return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const FpElement& fp) {
        os << fp.value;
        return os;
    }

private:
    static bool isPrime(int n) {
        if (n <= 1) return false;
        for (int i = 2; i * i <= n; ++i) {
            if (n % i == 0) return false;
        }
        return true;
    }

    static int modInverse(int a, int m) {
        int m0 = m, y = 0, x = 1;
        if (m == 1) return 0;
        while (a > 1) {
            int q = a / m;
            int t = m;
            m = a % m, a = t;
            t = y;
            y = x - q * y;
            x = t;
        }
        if (x < 0) x += m0;
        return x;
    }
};


class AdvancedPolynomialFitter {
public:
    std::vector<FpElement> fitPolynomial_superposSMO(
        const std::vector<FpElement>& x, const std::vector<FpElement>& y, int degree, int prime_p) {

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
        const int maxIterations = 20;
        const FpElement residualThreshold(0, prime_p); // Residual threshold is now FpElement zero
        const FpElement tolerance(0, prime_p); // Tolerance for coefficient update is also FpElement zero

        // 1. Precompute powers of x for efficiency in Fp
        std::vector<std::vector<FpElement>> xPowers(n, std::vector<FpElement>(m, FpElement(1, prime_p)));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 1; j < m; ++j) {
                xPowers[i][j] = xPowers[i][j - 1] * x[i];
            }
        }

        // 2. Calculate ATA and ATy for normal equations (for initial Gaussian Elimination solution) in Fp
        std::vector<std::vector<FpElement>> ATA(m, std::vector<FpElement>(m, FpElement(0, prime_p)));
        std::vector<FpElement> ATy(m, FpElement(0, prime_p));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] = ATy[j] + xPowers[i][j] * y[i];
                for (size_t k = 0; k <= j; ++k) {
                    ATA[j][k] = ATA[j][k] + xPowers[i][j] * xPowers[i][k];
                }
            }
        }
        for (size_t j = 0; j < m; ++j) { // Ensure symmetry of ATA
            for (size_t k = j + 1; k < m; ++k) {
                ATA[j][k] = ATA[k][j];
            }
        }

        // 3. Solve initial polynomial coefficients using Gaussian Elimination in Fp
        std::vector<FpElement> coefficients = solveGaussianElimination_Fp(ATA, ATy);

        // 4. Iterative Superposition SMO refinement
        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<FpElement> residuals(n, FpElement(0, prime_p));
            for (size_t i = 0; i < n; ++i) {
                FpElement fittedValue(0, prime_p);
                for (size_t j = 0; j < m; ++j) {
                    fittedValue = fittedValue + coefficients[j] * xPowers[i][j];
                }
                residuals[i] = y[i] - fittedValue;
            }

            std::vector<size_t> problemIndices;
            for (size_t i = 0; i < n; ++i) {
                if (residuals[i] != residualThreshold) { // Check for non-zero residual in Fp
                    problemIndices.push_back(i);
                }
            }

            if (problemIndices.empty()) {
                break; // No more problem points, converge in Fp
            }

            bool anyCoefficientUpdated = false;
            // Iterate through all pairs of coefficients for local SMO update
            for (size_t p = 0; p < m; ++p) {
                for (size_t q = p + 1; q < m; ++q) {
                    std::pair<FpElement, FpElement> updatedPair = solveLocalSMO(
                        coefficients, p, q, xPowers, y, problemIndices, prime_p); // Pass prime_p

                    if ((updatedPair.first - coefficients[p]) != tolerance || // Exact equality check in Fp
                        (updatedPair.second - coefficients[q]) != tolerance) {
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

        // 5. Return the coefficients as vector<FpElement>
        return coefficients;
    }

private:
    std::pair<FpElement, FpElement> solveLocalSMO(
        const std::vector<FpElement>& coeffs, size_t p, size_t q,
        const std::vector<std::vector<FpElement>>& xPowers,
        const std::vector<FpElement>& y, const std::vector<size_t>& problemIndices, int prime_p) {

        FpElement Apq(0, prime_p), App(0, prime_p), Aqq(0, prime_p), bp(0, prime_p), bq(0, prime_p);

        for (size_t i : problemIndices) {
            FpElement weight(1, prime_p); // No weighting for now in Fp version, set weight = 1
            App = App + weight * xPowers[i][p] * xPowers[i][p];
            Aqq = Aqq + weight * xPowers[i][q] * xPowers[i][q];
            Apq = Apq + weight * xPowers[i][p] * xPowers[i][q];
            bp = bp + weight * xPowers[i][p] * y[i];
            bq = bq + weight * xPowers[i][q] * y[i];
        }

        FpElement zero_fp(0, prime_p);
        if (App == zero_fp || Aqq == zero_fp) { // Check for exact zero in Fp
            return {coeffs[p], coeffs[q]}; // Avoid division by zero
        }

        FpElement det = App * Aqq - Apq * Apq;
        if (det == zero_fp) { // Check for exact zero determinant in Fp
            return {coeffs[p], coeffs[q]}; // Matrix is singular in Fp
        }

        FpElement newP = (Aqq * bp - Apq * bq) / det;
        FpElement newQ = (App * bq - Apq * bp) / det;

        return {newP, newQ};
    }


    std::vector<FpElement> solveGaussianElimination_Fp(
        std::vector<std::vector<FpElement>>& A, std::vector<FpElement>& b) {
        size_t n = A.size();
        size_t m = A[0].size(); // Assume square matrix for now, n = m
        int p = A[0][0].p;      // Assume all elements are in the same Fp
        FpElement zero_fp(0, p);

        for (size_t i = 0; i < n; ++i) {
            // 1. Find pivot in column i, starting from row i
            size_t pivot_row = i;
            for (size_t j = i + 1; j < n; ++j) {
                if (A[j][i] != zero_fp) {
                    pivot_row = j;
                    break;
                }
            }
            if (A[pivot_row][i] == zero_fp) {
                // No pivot found in this column - matrix might be singular (in Fp context)
                throw std::runtime_error("Singular matrix encountered in Gaussian Elimination over Fp");
            }
            std::swap(A[i], A[pivot_row]);
            std::swap(b[i], b[pivot_row]);

            // 2. Eliminate below pivot
            for (size_t j = i + 1; j < n; ++j) {
                FpElement factor = A[j][i] / A[i][i]; // Use Fp division
                for (size_t k = i; k < n; ++k) {
                    A[j][k] = A[j][k] - factor * A[i][k]; // Use Fp arithmetic
                }
                b[j] = b[j] - factor * b[i]; // Apply to b as well
            }
        }

        // Back substitution (using Fp arithmetic)
        std::vector<FpElement> x(n);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] = x[i] - A[i][j] * x[j];
            }
            x[i] = x[i] / A[i][i]; // Fp division
        }
        return x;
    }
};


int main() {
    int prime_p = 17; // Choose a prime modulus
    AdvancedPolynomialFitter fitter;

    std::vector<FpElement> x_fp, y_fp;
    // Example data in Fp (mod 17)
    x_fp.push_back(FpElement(1, prime_p)); y_fp.push_back(FpElement(2, prime_p));
    x_fp.push_back(FpElement(2, prime_p)); y_fp.push_back(FpElement(8, prime_p));
    x_fp.push_back(FpElement(3, prime_p)); y_fp.push_back(FpElement(7, prime_p));
    x_fp.push_back(FpElement(4, prime_p)); y_fp.push_back(FpElement(9, prime_p));
    x_fp.push_back(FpElement(5, prime_p)); y_fp.push_back(FpElement(12, prime_p));


    int degree = 2;
    std::vector<FpElement> coeffs_fp = fitter.fitPolynomial_superposSMO(x_fp, y_fp, degree, prime_p);

    std::cout << "Polynomial Coefficients in Fp(" << prime_p << "):" << std::endl;
    for (size_t i = 0; i < coeffs_fp.size(); ++i) {
        std::cout << "Coeff " << i << ": " << coeffs_fp[i] << std::endl;
    }

    return 0;
}
