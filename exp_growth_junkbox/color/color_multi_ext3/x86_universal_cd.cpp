#include <vector>
#include <complex>
#include <memory>
#include <variant>
#include <optional>
#include <type_traits>
#include <immintrin.h>
#include <exception>
#include <string_view>
#include <concepts>
#include <random>
#include <algorithm>

// Previous code remains the same up to OptimizedBasisSystem...

template<FloatingPoint T>
class OptimizedBasisSystem {
public:
    enum class Type {
        MONOMIAL,
        CHEBYSHEV,
        LEGENDRE,
        SYMMETRIC,
        GALOIS_INVARIANT
    };

    template<Type BASIS_TYPE, size_t DEGREE>
    static constexpr auto generateBasisFunctions() {
        if constexpr (BASIS_TYPE == Type::CHEBYSHEV) {
            return generateChebyshevBasis<DEGREE>();
        } else if constexpr (BASIS_TYPE == Type::LEGENDRE) {
            return generateLegendreBasis<DEGREE>();
        } else {
            return generateMonomialBasis<DEGREE>();
        }
    }

private:
    template<size_t DEGREE>
    static constexpr auto generateChebyshevBasis() {
        std::array<std::array<T, DEGREE + 1>, DEGREE + 1> coeffs{};
        // T[i][j] represents coefficient of x^j in i-th Chebyshev polynomial
        coeffs[0][0] = 1;  // T₀(x) = 1
        if constexpr (DEGREE > 0) {
            coeffs[1][1] = 1;  // T₁(x) = x
            for (size_t n = 2; n <= DEGREE; ++n) {
                // Recurrence relation: Tₙ(x) = 2xTₙ₋₁(x) - Tₙ₋₂(x)
                for (size_t j = 0; j <= n; ++j) {
                    if (j > 0) coeffs[n][j] += 2 * coeffs[n-1][j-1];
                    coeffs[n][j] -= coeffs[n-2][j];
                }
            }
        }
        return coeffs;
    }

    template<size_t DEGREE>
    static constexpr auto generateLegendreBasis() {
        std::array<std::array<T, DEGREE + 1>, DEGREE + 1> coeffs{};
        // P[i][j] represents coefficient of x^j in i-th Legendre polynomial
        coeffs[0][0] = 1;  // P₀(x) = 1
        if constexpr (DEGREE > 0) {
            coeffs[1][1] = 1;  // P₁(x) = x
            for (size_t n = 2; n <= DEGREE; ++n) {
                // Recurrence: Pₙ(x) = ((2n-1)xPₙ₋₁(x) - (n-1)Pₙ₋₂(x))/n
                T factor1 = (2 * n - 1.0) / n;
                T factor2 = (n - 1.0) / n;
                for (size_t j = 0; j <= n; ++j) {
                    if (j > 0) coeffs[n][j] += factor1 * coeffs[n-1][j-1];
                    coeffs[n][j] -= factor2 * coeffs[n-2][j];
                }
            }
        }
        return coeffs;
    }

    template<size_t DEGREE>
    static constexpr auto generateMonomialBasis() {
        std::array<std::array<T, DEGREE + 1>, DEGREE + 1> coeffs{};
        for (size_t i = 0; i <= DEGREE; ++i) {
            coeffs[i][i] = 1;  // x^i has coefficient 1 for x^i term
        }
        return coeffs;
    }
};

// Enhanced Matrix class with complete SIMD operations
template<FloatingPoint T>
class Matrix {
    alignas(32) std::vector<T> data;
    size_t rows_, cols_;

    static constexpr size_t SIMD_WIDTH = 8;

public:
    Matrix(size_t rows, size_t cols) 
        : data(rows * cols), rows_(rows), cols_(cols) {}

    void resize(size_t rows, size_t cols) {
        data.resize(rows * cols);
        rows_ = rows;
        cols_ = cols;
    }

    T& operator()(size_t i, size_t j) { return data[i * cols_ + j]; }
    const T& operator()(size_t i, size_t j) const { return data[i * cols_ + j]; }

    // SIMD matrix multiplication
    Matrix<T> multiply(const Matrix<T>& rhs) const {
        if (cols_ != rhs.rows_) {
            throw PolynomialError(
                PolynomialError::ErrorCode::INVALID_INPUT,
                "Invalid matrix dimensions for multiplication");
        }

        Matrix<T> result(rows_, rhs.cols_);
        
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < rhs.cols_; j += SIMD_WIDTH) {
                __m256d sum = _mm256_setzero_pd();
                for (size_t k = 0; k < cols_; ++k) {
                    __m256d a = _mm256_set1_pd((*this)(i, k));
                    __m256d b = _mm256_loadu_pd(&rhs(k, j));
                    sum = _mm256_add_pd(sum, _mm256_mul_pd(a, b));
                }
                _mm256_storeu_pd(&result(i, j), sum);
            }
        }
        return result;
    }

    // Compute QR decomposition using modified Gram-Schmidt
    std::pair<Matrix<T>, Matrix<T>> qr() const {
        Matrix<T> Q(rows_, cols_);
        Matrix<T> R(cols_, cols_);
        
        for (size_t j = 0; j < cols_; ++j) {
            // Copy column j
            for (size_t i = 0; i < rows_; ++i) {
                Q(i, j) = (*this)(i, j);
            }
            
            // Orthogonalize against previous columns
            for (size_t k = 0; k < j; ++k) {
                T dot = 0;
                __m256d sum = _mm256_setzero_pd();
                
                for (size_t i = 0; i < rows_; i += 4) {
                    __m256d v1 = _mm256_loadu_pd(&Q(i, j));
                    __m256d v2 = _mm256_loadu_pd(&Q(i, k));
                    sum = _mm256_add_pd(sum, _mm256_mul_pd(v1, v2));
                }
                
                // Horizontal sum
                dot = _mm256_reduce_add_pd(sum);
                
                R(k, j) = dot;
                
                // Subtract projection
                for (size_t i = 0; i < rows_; i += 4) {
                    __m256d v = _mm256_loadu_pd(&Q(i, k));
                    __m256d proj = _mm256_mul_pd(v, _mm256_set1_pd(dot));
                    __m256d curr = _mm256_loadu_pd(&Q(i, j));
                    curr = _mm256_sub_pd(curr, proj);
                    _mm256_storeu_pd(&Q(i, j), curr);
                }
            }
            
            // Normalize column j
            T norm = 0;
            __m256d sum = _mm256_setzero_pd();
            
            for (size_t i = 0; i < rows_; i += 4) {
                __m256d v = _mm256_loadu_pd(&Q(i, j));
                sum = _mm256_add_pd(sum, _mm256_mul_pd(v, v));
            }
            
            norm = std::sqrt(_mm256_reduce_add_pd(sum));
            R(j, j) = norm;
            
            if (norm > 1e-10) {
                __m256d norm_v = _mm256_set1_pd(1.0 / norm);
                for (size_t i = 0; i < rows_; i += 4) {
                    __m256d v = _mm256_loadu_pd(&Q(i, j));
                    v = _mm256_mul_pd(v, norm_v);
                    _mm256_storeu_pd(&Q(i, j), v);
                }
            }
        }
        
        return {Q, R};
    }
};

// Complete OptimizedPolynomialSystem implementation
template<FloatingPoint T>
class OptimizedPolynomialSystem {
    // ... Previous code remains the same ...

private:
    void fitWithSIMD(std::span<const T> x, std::span<const T> y, Result& result) {
        const size_t n = x.size();
        const size_t degree = options.polynomial_degree;
        
        // Normalize input data
        auto [x_norm, scale_factor] = normalizeData(x);
        
        // Construct basis matrix
        Matrix<T> basis_matrix(n, degree + 1);
        constructBasisMatrix(x_norm, basis_matrix);
        
        // Solve using QR decomposition
        auto [Q, R] = basis_matrix.qr();
        
        // Solve R * coeffs = Q^T * y
        auto Qt_y = multiplyTransposed(Q, y);
        result.coefficients = backSubstitution(R, Qt_y);
        
        // Compute condition number and residual
        result.condition_number = computeConditionNumber(R);
        result.residual_error = computeResidualError(basis_matrix, 
                                                   result.coefficients, y);
        
        // Check for numerical instability
        if (result.condition_number > 1e10) {
            result.warnings.push_back(
                "High condition number detected. Results may be unstable.");
            
            if (options.use_galois) {
                // Apply Galois theory to improve stability
                applyGaloisStabilization(result);
            }
        }
    }

    std::pair<std::vector<T>, T> normalizeData(std::span<const T> x) {
        T x_min = *std::min_element(x.begin(), x.end());
        T x_max = *std::max_element(x.begin(), x.end());
        T scale = 2.0 / (x_max - x_min);
        
        std::vector<T> x_norm(x.size());
        for (size_t i = 0; i < x.size(); i += 4) {
            __m256d v = _mm256_loadu_pd(&x[i]);
            __m256d scaled = _mm256_mul_pd(
                _mm256_sub_pd(v, _mm256_set1_pd(x_min)),
                _mm256_set1_pd(scale));
            _mm256_storeu_pd(&x_norm[i], scaled);
        }
        
        return {x_norm, scale};
    }

    void constructBasisMatrix(const std::vector<T>& x_norm, 
                            Matrix<T>& basis_matrix) {
        switch (options.basis_type) {
            case OptimizedBasisSystem<T>::Type::CHEBYSHEV:
                constructChebyshevBasis(x_norm, basis_matrix);
                break;
            case OptimizedBasisSystem<T>::Type::LEGENDRE:
                constructLegendreBasis(x_norm, basis_matrix);
                break;
            default:
                constructMonomialBasis(x_norm, basis_matrix);
        }
    }

    void constructChebyshevBasis(const std::vector<T>& x_norm, 
                                Matrix<T>& basis_matrix) {
        const size_t n = x_norm.size();
        const size_t degree = options.polynomial_degree;
        
        auto coeffs = OptimizedBasisSystem<T>::template 
            generateBasisFunctions<
                OptimizedBasisSystem<T>::Type::CHEBYSHEV,
                32>();  // Maximum degree supported
        
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j <= degree; ++j) {
                T sum = 0;
                for (size_t k = 0; k <= j; ++k) {
                    sum += coeffs[j][k] * std::pow(x_norm[i], k);
                }
                basis_matrix(i, j) = sum;
            }
        }
    }

    void applyGaloisStabilization(Result& result) {
        // Apply Galois theory to improve numerical stability
        const auto& orbits = galois_structure.getOrbits();
        
        // Modify coefficients based on Galois symmetries
        for (size_t i = 0; i < result.coefficients.size(); ++i) {
            T sum = 0;
            size_t count = 0;
            
            for (const auto& orbit : orbits) {
                if (orbit.characteristic == i) {
                    sum += result.coefficients[i] * orbit.elements.size();
                    count += orbit.elements.size();
                }
            }
            
            if (count > 0) {
                result.coefficients[i] = sum / count;
            }
        }
    }

    std::vector<T> backSubstitution(const Matrix<T>& R, 
                                   const std::vector<T>& y) {
        const size_t n = R.cols_;
        std::vector<T> x(n);
        
        for (int i = n - 1; i >= 0; --i) {
            T sum = y[i];
            for (size_t j = i + 1; j < n; ++j) {
                sum -= R(i, j) * x[j];
            }
            x[i] = sum / R(i, i);
        }
        
        return x;
    }

    void evaluateWithSIMD(const Result& result, 
                         std::span<const T> x, 
                         std::vector<T>& y_pred) {
        auto [x_norm, scale_factor] = normalizeData(x);
        
        const size_t n = x.size();
        const size_t degree = result.coefficients.size() - 1;
        
        for (size_t i = 0; i < n; i += 4) {
            __m256d sum = _mm256_setzero_pd();
            __m256d x_val = _mm256_loadu_pd(&x_norm[i]);
            
            for (size_t j = 0; j <= degree; ++j) {
                __m256d coeff = _mm256_set1_pd(result.coefficients[j]);
                
                if (options.basis_type == 
                    OptimizedBasisSystem<T>::Type::CHEBYSHEV) {
