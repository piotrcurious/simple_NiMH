To use this improved version:

```cpp
std::vector<float> x = {...};  // Your x data
std::vector<float> y = {...};  // Your y data

// Fit using Chebyshev basis with rational approximation
auto result = EnhancedPolynomialFitter::fit(x, y, 5, 
    EnhancedPolynomialFitter::CHEBYSHEV, true);

// Evaluate at new points
std::vector<float> x_new = {...};  // Points to evaluate at
auto y_pred = EnhancedPolynomialFitter::evaluate(result, x_new);
```
