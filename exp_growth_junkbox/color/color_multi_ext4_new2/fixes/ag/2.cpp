#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <cassert>

// Class for Finite Field elements in Fp (Z/pZ)
class FpElement {
private:
    int value; // Represent element in Z/pZ
    int p;     // Prime modulus

    // Helper functions
    static bool isPrime(int n) {
        if (n <= 1) return false;
        if (n <= 3) return true;
        if (n % 2 == 0 || n % 3 == 0) return false;
        
        // Check using 6k+-1 optimization
        for (int i = 5; i * i <= n; i += 6) {
            if (n % i == 0 || n % (i + 2) == 0) return false;
        }
        return true;
    }

    // Extended Euclidean Algorithm for modular inverse
    static int modInverse(int a, int m) {
        int m0 = m, y = 0, x = 1;
        
        if (m == 1) return 0;
        
        // Process while a > 1
        while (a > 1) {
            // q is quotient
            int q = a / m;
            int t = m;
            
            // m is remainder now, process same as Euclid's algorithm
            m = a % m;
            a = t;
            t = y;
            
            // Update x and y
            y = x - q * y;
            x = t;
        }
        
        // Make x positive
        if (x < 0) x += m0;
        
        return x;
    }

public:
    // Constructors
    FpElement() : value(0), p(2) {}
    
    explicit FpElement(int v, int prime_p) : p(prime_p) {
        if (p <= 1 || !isPrime(p)) {
            throw std::invalid_argument("Modulus p must be a prime number greater than 1.");
        }
        
        // Normalize value to [0, p-1]
        value = v % p;
        if (value < 0) value += p;
    }
    
    // Copy constructor
    FpElement(const FpElement& other) = default;
    
    // Move constructor
    FpElement(FpElement&& other) noexcept = default;
    
    // Copy assignment operator
    FpElement& operator=(const FpElement& other) {
        if (this != &other) {
            if (p != other.p) {
                throw std::invalid_argument("Cannot assign FpElements with different moduli.");
            }
            value = other.value;
        }
        return *this;
    }
    
    // Move assignment operator
    FpElement& operator=(FpElement&& other) noexcept {
        if (this != &other) {
            if (p != other.p) {
                throw std::invalid_argument("Cannot assign FpElements with different moduli.");
            }
            value = other.value;
        }
        return *this;
    }
    
    // Getters
    int getValue() const { return value; }
    int getModulus() const { return p; }
    
    // Arithmetic operators
    FpElement operator+(const FpElement& other) const {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for addition.");
        }
        return FpElement((value + other.value) % p, p);
    }
    
    FpElement operator-(const FpElement& other) const {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for subtraction.");
        }
        return FpElement((value - other.value + p) % p, p);
    }
    
    FpElement operator*(const FpElement& other) const {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for multiplication.");
        }
        return FpElement(static_cast<int>(static_cast<long long>(value) * other.value % p), p);
    }
    
    FpElement operator/(const FpElement& other) const {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for division.");
        }
        if (other.value == 0) {
            throw std::runtime_error("Division by zero in FpElement.");
        }
        int inv_val = modInverse(other.value, p);
        return FpElement(static_cast<int>(static_cast<long long>(value) * inv_val % p), p);
    }
    
    // Compound assignment operators
    FpElement& operator+=(const FpElement& other) {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for addition.");
        }
        value = (value + other.value) % p;
        return *this;
    }
    
    FpElement& operator-=(const FpElement& other) {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for subtraction.");
        }
        value = (value - other.value + p) % p;
        return *this;
    }
    
    FpElement& operator*=(const FpElement& other) {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for multiplication.");
        }
        value = static_cast<int>(static_cast<long long>(value) * other.value % p);
        return *this;
    }
    
    FpElement& operator/=(const FpElement& other) {
        if (p != other.p) {
            throw std::invalid_argument("FpElements must have the same modulus for division.");
        }
        if (other.value == 0) {
            throw std::runtime_error("Division by zero in FpElement.");
        }
        int inv_val = modInverse(other.value, p);
        value = static_cast<int>(static_cast<long long>(value) * inv_val % p);
        return *this;
    }
    
    // Comparison operators
    bool operator==(const FpElement& other) const {
        return p == other.p && value == other.value;
    }
    
    bool operator!=(const FpElement& other) const {
        return !(*this == other);
    }
    
    // Stream insertion operator
    friend std::ostream& operator<<(std::ostream& os, const FpElement& fp) {
        os << fp.value;
        return os;
    }
    
    // Create zero element with same modulus
    static FpElement zero(int modulus) {
        return FpElement(0, modulus);
    }
    
    // Create one element with same modulus
    static FpElement one(int modulus) {
        return FpElement(1, modulus);
    }
};

class AdvancedPolynomialFitter {
public:
    // Fit a polynomial over a finite field using superposition SMO algorithm
    std::vector<FpElement> fitPolynomial_superposSMO(
        const std::vector<FpElement>& x, 
        const std::vector<FpElement>& y, 
        int degree, 
        int prime_p) {
        
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
        
        // Validate all elements have the same modulus
        for (const auto& val : x) {
            if (val.getModulus() != prime_p) {
                throw std::invalid_argument("All x values must have the same modulus as prime_p.");
            }
        }
        for (const auto& val : y) {
            if (val.getModulus() != prime_p) {
                throw std::invalid_argument("All y values must have the same modulus as prime_p.");
            }
        }

        const size_t n = x.size();
        const size_t m = degree + 1; // Number of coefficients
        const int maxIterations = 20;
        
        // Create elements in Fp
        const FpElement zeroElement = FpElement::zero(prime_p);
        const FpElement oneElement = FpElement::one(prime_p);

        // 1. Precompute powers of x for efficiency in Fp
        std::vector<std::vector<FpElement>> xPowers(n, std::vector<FpElement>(m, oneElement));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 1; j < m; ++j) {
                xPowers[i][j] = xPowers[i][j - 1] * x[i];
            }
        }

        // 2. Calculate ATA and ATy for normal equations (for initial Gaussian Elimination solution) in Fp
        std::vector<std::vector<FpElement>> ATA(m, std::vector<FpElement>(m, zeroElement));
        std::vector<FpElement> ATy(m, zeroElement);
        
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

        // 3. Solve initial polynomial coefficients using Gaussian Elimination in Fp
        std::vector<FpElement> coefficients = solveGaussianElimination_Fp(ATA, ATy);

        // 4. Iterative Superposition SMO refinement
        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<FpElement> residuals(n, zeroElement);
            
            // Calculate residuals
            for (size_t i = 0; i < n; ++i) {
                FpElement fittedValue = zeroElement;
                for (size_t j = 0; j < m; ++j) {
                    fittedValue += coefficients[j] * xPowers[i][j];
                }
                residuals[i] = y[i] - fittedValue;
            }

            // Find points with non-zero residuals
            std::vector<size_t> problemIndices;
            for (size_t i = 0; i < n; ++i) {
                if (residuals[i] != zeroElement) {
                    problemIndices.push_back(i);
                }
            }

            if (problemIndices.empty()) {
                break; // No more problem points, converged
            }

            bool anyCoefficientUpdated = false;
            
            // Iterate through all pairs of coefficients for local SMO update
            for (size_t p = 0; p < m; ++p) {
                for (size_t q = p + 1; q < m; ++q) {
                    auto [updatedP, updatedQ] = solveLocalSMO(
                        coefficients, p, q, xPowers, y, problemIndices, prime_p);

                    if (updatedP != coefficients[p] || updatedQ != coefficients[q]) {
                        coefficients[p] = updatedP;
                        coefficients[q] = updatedQ;
                        anyCoefficientUpdated = true;
                    }
                }
            }
            
            if (!anyCoefficientUpdated) {
                break; // No coefficient updated significantly, converged
            }
        }

        return coefficients;
    }

    // Test if a polynomial fits the data points
    bool testFit(
        const std::vector<FpElement>& coefficients,
        const std::vector<FpElement>& x,
        const std::vector<FpElement>& y) {
        
        if (x.size() != y.size() || x.empty() || coefficients.empty()) {
            return false;
        }
        
        const int prime_p = coefficients[0].getModulus();
        const FpElement zeroElement = FpElement::zero(prime_p);
        
        for (size_t i = 0; i < x.size(); ++i) {
            FpElement result = zeroElement;
            FpElement xPower = FpElement::one(prime_p);
            
            for (const auto& coeff : coefficients) {
                result += coeff * xPower;
                xPower *= x[i];
            }
            
            if (result != y[i]) {
                return false;
            }
        }
        
        return true;
    }

private:
    // Solve a 2x2 local problem using Sequential Minimal Optimization
    std::pair<FpElement, FpElement> solveLocalSMO(
        const std::vector<FpElement>& coeffs, 
        size_t p, 
        size_t q,
        const std::vector<std::vector<FpElement>>& xPowers,
        const std::vector<FpElement>& y, 
        const std::vector<size_t>& problemIndices, 
        int prime_p) {

        FpElement Apq = FpElement::zero(prime_p);
        FpElement App = FpElement::zero(prime_p);
        FpElement Aqq = FpElement::zero(prime_p);
        FpElement bp = FpElement::zero(prime_p);
        FpElement bq = FpElement::zero(prime_p);
        FpElement weight = FpElement::one(prime_p);  // Unit weight for now

        // Calculate matrix elements and right-hand side
        for (size_t idx : problemIndices) {
            App += weight * xPowers[idx][p] * xPowers[idx][p];
            Aqq += weight * xPowers[idx][q] * xPowers[idx][q];
            Apq += weight * xPowers[idx][p] * xPowers[idx][q];
            bp += weight * xPowers[idx][p] * y[idx];
            bq += weight * xPowers[idx][q] * y[idx];
        }

        // Check for zero diagonal elements
        if (App == FpElement::zero(prime_p) || Aqq == FpElement::zero(prime_p)) {
            return {coeffs[p], coeffs[q]}; // Avoid division by zero
        }

        // Calculate determinant
        FpElement det = App * Aqq - Apq * Apq;
        
        // Check for singular matrix
        if (det == FpElement::zero(prime_p)) {
            return {coeffs[p], coeffs[q]}; // Matrix is singular
        }

        // Calculate new coefficient values
        FpElement newP = (Aqq * bp - Apq * bq) / det;
        FpElement newQ = (App * bq - Apq * bp) / det;

        return {newP, newQ};
    }

    // Solve a system of linear equations using Gaussian Elimination in Fp
    std::vector<FpElement> solveGaussianElimination_Fp(
        std::vector<std::vector<FpElement>>& A, 
        std::vector<FpElement>& b) {
        
        size_t n = A.size();
        if (n == 0 || A[0].size() != n || b.size() != n) {
            throw std::invalid_argument("Invalid matrix or vector dimensions for Gaussian elimination");
        }
        
        // Get modulus from first element
        int p = A[0][0].getModulus();
        FpElement zeroElement = FpElement::zero(p);

        // Forward elimination with pivoting
        for (size_t i = 0; i < n; ++i) {
            // Find pivot (non-zero element)
            size_t pivot_row = i;
            for (size_t j = i; j < n; ++j) {
                if (A[j][i] != zeroElement) {
                    pivot_row = j;
                    break;
                }
            }
            
            if (A[pivot_row][i] == zeroElement) {
                throw std::runtime_error("Singular matrix encountered in Gaussian Elimination over Fp");
            }
            
            // Swap rows if needed
            if (pivot_row != i) {
                std::swap(A[i], A[pivot_row]);
                std::swap(b[i], b[pivot_row]);
            }

            // Eliminate below pivot
            for (size_t j = i + 1; j < n; ++j) {
                if (A[j][i] != zeroElement) {
                    FpElement factor = A[j][i] / A[i][i];
                    
                    for (size_t k = i; k < n; ++k) {
                        A[j][k] -= factor * A[i][k];
                    }
                    
                    b[j] -= factor * b[i];
                }
            }
        }

        // Back substitution
        std::vector<FpElement> x(n, zeroElement);
        for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
            x[i] = b[i];
            
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            
            x[i] /= A[i][i];
        }
        
        return x;
    }
};

// Function to evaluate a polynomial at point x
FpElement evaluatePolynomial(const std::vector<FpElement>& coeffs, const FpElement& x) {
    if (coeffs.empty()) {
        throw std::invalid_argument("Coefficients vector cannot be empty");
    }
    
    FpElement result = FpElement::zero(x.getModulus());
    FpElement xPower = FpElement::one(x.getModulus());
    
    for (const auto& coeff : coeffs) {
        result += coeff * xPower;
        xPower *= x;
    }
    
    return result;
}

int main() {
    try {
        int prime_p = 17; // Choose a prime modulus
        AdvancedPolynomialFitter fitter;

        // Example data in Fp (mod 17)
        std::vector<FpElement> x_fp = {
            FpElement(1, prime_p),
            FpElement(2, prime_p),
            FpElement(3, prime_p),
            FpElement(4, prime_p),
            FpElement(5, prime_p)
        };
        
        std::vector<FpElement> y_fp = {
            FpElement(2, prime_p),
            FpElement(8, prime_p),
            FpElement(7, prime_p),
            FpElement(9, prime_p),
            FpElement(12, prime_p)
        };

        int degree = 2; // Quadratic polynomial
        
        // Fit the polynomial
        std::vector<FpElement> coeffs_fp = fitter.fitPolynomial_superposSMO(x_fp, y_fp, degree, prime_p);

        // Print the results
        std::cout << "Polynomial Coefficients in Fp(" << prime_p << "):" << std::endl;
        for (size_t i = 0; i < coeffs_fp.size(); ++i) {
            std::cout << "Coeff " << i << ": " << coeffs_fp[i] << std::endl;
        }
        
        // Print the polynomial in readable form
        std::cout << "\nPolynomial: ";
        for (int i = coeffs_fp.size() - 1; i >= 0; --i) {
            if (i < static_cast<int>(coeffs_fp.size()) - 1) {
                std::cout << " + ";
            }
            if (i > 0) {
                std::cout << coeffs_fp[i] << "x^" << i;
            } else {
                std::cout << coeffs_fp[i];
            }
        }
        std::cout << std::endl;
        
        // Verify the fit
        std::cout << "\nVerifying fit at data points:" << std::endl;
        for (size_t i = 0; i < x_fp.size(); ++i) {
            FpElement calculated = evaluatePolynomial(coeffs_fp, x_fp[i]);
            std::cout << "f(" << x_fp[i] << ") = " << calculated << " (Expected: " << y_fp[i] << ")";
            std::cout << (calculated == y_fp[i] ? " ✓" : " ✗") << std::endl;
        }
        
        // Test if the polynomial fits all points
        bool perfectFit = fitter.testFit(coeffs_fp, x_fp, y_fp);
        std::cout << "\nPerfect fit: " << (perfectFit ? "Yes" : "No") << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
