#include <vector>
#include <complex>
#include <memory>
#include <variant>
#include <optional>

// Forward declarations
class BasisSystem;
class GaloisStructure;
class FittingStrategy;

// Complex number typedef for clarity
using Complex = std::complex<double>;

// Unified basis system interface
class BasisSystem {
public:
    enum class Type {
        MONOMIAL,
        CHEBYSHEV,
        LEGENDRE,
        SYMMETRIC,
        GALOIS_INVARIANT
    };

    virtual ~BasisSystem() = default;
    virtual std::vector<std::vector<double>> computeBasis(
        const std::vector<double>& x_norm, 
        int degree) const = 0;
    virtual std::vector<double> evaluate(
        const std::vector<double>& x_norm,
        const std::vector<double>& coeffs) const = 0;
};

// Implementation of classical orthogonal polynomials
class OrthogonalBasis : public BasisSystem {
private:
    Type type;

    std::vector<double> evaluateChebyshev(double x, int degree) const {
        std::vector<double> values(degree + 1);
        values[0] = 1.0;
        if (degree > 0) {
            values[1] = x;
            for (int i = 2; i <= degree; ++i) {
                values[i] = 2.0 * x * values[i-1] - values[i-2];
            }
        }
        return values;
    }

    std::vector<double> evaluateLegendre(double x, int degree) const {
        std::vector<double> values(degree + 1);
        values[0] = 1.0;
        if (degree > 0) {
            values[1] = x;
            for (int i = 2; i <= degree; ++i) {
                values[i] = ((2.0 * i - 1.0) * x * values[i-1] - 
                            (i - 1.0) * values[i-2]) / i;
            }
        }
        return values;
    }

public:
    explicit OrthogonalBasis(Type t) : type(t) {}

    std::vector<std::vector<double>> computeBasis(
        const std::vector<double>& x_norm, 
        int degree) const override {
        
        size_t n = x_norm.size();
        std::vector<std::vector<double>> basis(n, std::vector<double>(degree + 1));

        for (size_t i = 0; i < n; ++i) {
            std::vector<double> values;
            switch (type) {
                case Type::CHEBYSHEV:
                    values = evaluateChebyshev(x_norm[i], degree);
                    break;
                case Type::LEGENDRE:
                    values = evaluateLegendre(x_norm[i], degree);
                    break;
                default:  // MONOMIAL
                    values.resize(degree + 1);
                    double xi = 1.0;
                    for (int j = 0; j <= degree; ++j) {
                        values[j] = xi;
                        xi *= x_norm[i];
                    }
            }
            basis[i] = std::move(values);
        }
        return basis;
    }

    std::vector<double> evaluate(
        const std::vector<double>& x_norm,
        const std::vector<double>& coeffs) const override {
        
        std::vector<double> result(x_norm.size());
        for (size_t i = 0; i < x_norm.size(); ++i) {
            auto basis_values = (type == Type::CHEBYSHEV) ? 
                evaluateChebyshev(x_norm[i], coeffs.size() - 1) :
                evaluateLegendre(x_norm[i], coeffs.size() - 1);
            
            result[i] = std::inner_product(
                coeffs.begin(), coeffs.end(),
                basis_values.begin(), 0.0);
        }
        return result;
    }
};

// Enhanced Galois structure with symmetric polynomials
class GaloisStructure {
private:
    std::vector<std::vector<Complex>> orbits;
    std::vector<SymmetricPolynomial> symmetricBasis;
    std::unique_ptr<GaloisGroup> galoisGroup;

public:
    GaloisStructure(int fieldDegree) {
        auto cyclotomicField = std::make_unique<CyclotomicField>(fieldDegree);
        galoisGroup = std::make_unique<GaloisGroup>(*cyclotomicField);
        orbits = galoisGroup->getOrbits();
        initializeSymmetricBasis(fieldDegree);
    }

    void initializeSymmetricBasis(int degree) {
        auto elementary = SymmetricPolynomial::elementarySymmetric(degree);
        auto powerSum = SymmetricPolynomial::powerSum(degree);
        symmetricBasis = elementary;
        symmetricBasis.insert(symmetricBasis.end(), 
                            powerSum.begin(), powerSum.end());
    }

    const std::vector<std::vector<Complex>>& getOrbits() const { return orbits; }
    const std::vector<SymmetricPolynomial>& getSymmetricBasis() const { 
        return symmetricBasis; 
    }
};

// Unified fitting strategy interface
class FittingStrategy {
public:
    struct FittingResult {
        std::vector<float> coefficients;
        std::optional<std::vector<float>> denominatorCoeffs;
        std::optional<std::vector<std::vector<Complex>>> galoisOrbits;
        double condition_number;
        double residual_error;
        BasisSystem::Type basis_type;
    };

    virtual ~FittingStrategy() = default;
    virtual FittingResult fit(
        const std::vector<double>& x_norm,
        const std::vector<float>& y,
        int degree) const = 0;
};

// Implementation of different fitting strategies
class StandardFitting : public FittingStrategy {
private:
    std::shared_ptr<BasisSystem> basis;
    bool use_rational;

public:
    StandardFitting(std::shared_ptr<BasisSystem> b, bool rational = false)
        : basis(b), use_rational(rational) {}

    FittingResult fit(
        const std::vector<double>& x_norm,
        const std::vector<float>& y,
        int degree) const override {
        
        auto basis_matrix = basis->computeBasis(x_norm, degree);
        
        FittingResult result;
        if (use_rational) {
            auto [num_coeffs, den_coeffs] = fitRational(
                basis_matrix, y, degree, degree/2);
            result.coefficients = num_coeffs;
            result.denominatorCoeffs = den_coeffs;
        } else {
            result.coefficients = solveLinearSystemQR(basis_matrix, y);
        }
        
        result.condition_number = computeConditionNumber(basis_matrix);
        result.residual_error = computeResidualError(
            basis_matrix, result.coefficients, y);
        return result;
    }
};

class GaloisFitting : public FittingStrategy {
private:
    std::shared_ptr<GaloisStructure> galois;
    std::shared_ptr<BasisSystem> basis;

public:
    GaloisFitting(
        std::shared_ptr<GaloisStructure> g,
        std::shared_ptr<BasisSystem> b)
        : galois(g), basis(b) {}

    FittingResult fit(
        const std::vector<double>& x_norm,
        const std::vector<float>& y,
        int degree) const override {
        
        auto invariant_basis = computeInvariantBasis(x_norm, degree);
        auto coeffs = solveLinearSystemQR(invariant_basis, y);
        
        FittingResult result;
        result.coefficients = coeffs;
        result.galoisOrbits = galois->getOrbits();
        result.condition_number = computeConditionNumber(invariant_basis);
        result.residual_error = computeResidualError(
            invariant_basis, coeffs, y);
        return result;
    }
};

// Main integrated fitting system
class IntegratedPolynomialSystem {
private:
    std::shared_ptr<BasisSystem> basis;
    std::shared_ptr<GaloisStructure> galois;
    std::unique_ptr<FittingStrategy> strategy;

public:
    struct FittingOptions {
        BasisSystem::Type basis_type = BasisSystem::Type::CHEBYSHEV;
        bool use_rational = false;
        bool use_galois = false;
        int galois_field_degree = 8;
        int polynomial_degree = 5;
    };

    IntegratedPolynomialSystem(const FittingOptions& options) {
        // Initialize basis system
        switch (options.basis_type) {
            case BasisSystem::Type::CHEBYSHEV:
                basis = std::make_shared<OrthogonalBasis>(
                    BasisSystem::Type::CHEBYSHEV);
                break;
            case BasisSystem::Type::LEGENDRE:
                basis = std::make_shared<OrthogonalBasis>(
                    BasisSystem::Type::LEGENDRE);
                break;
            default:
                basis = std::make_shared<OrthogonalBasis>(
                    BasisSystem::Type::MONOMIAL);
        }

        // Initialize Galois structure if needed
        if (options.use_galois) {
            galois = std::make_shared<GaloisStructure>(
                options.galois_field_degree);
            strategy = std::make_unique<GaloisFitting>(galois, basis);
        } else {
            strategy = std::make_unique<StandardFitting>(
                basis, options.use_rational);
        }
    }

    FittingStrategy::FittingResult fit(
        const std::vector<float>& x,
        const std::vector<float>& y,
        int degree) {
        
        // Normalize input data
        auto x_norm = normalizeInterval(x);
        return strategy->fit(x_norm, y, degree);
    }

    std::vector<float> evaluate(
        const FittingStrategy::FittingResult& result,
        const std::vector<float>& x) {
        
        auto x_norm = normalizeInterval(x);
        
        if (result.galoisOrbits) {
            return evaluateGaloisFit(result, x_norm);
        } else if (result.denominatorCoeffs) {
            return evaluateRationalFit(result, x_norm);
        } else {
            auto values = basis->evaluate(x_norm, 
                std::vector<double>(result.coefficients.begin(),
                                  result.coefficients.end()));
            return std::vector<float>(values.begin(), values.end());
        }
    }
};
