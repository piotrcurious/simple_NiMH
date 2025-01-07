void testPolynomialFitter() {
    OptimizedPolynomialFitter fitter;

    // Example data points
    std::vector<float> x = {1, 2, 3, 4, 5};
    std::vector<float> y = {2.2, 4.1, 6.3, 8.0, 10.1};

    // Fit a polynomial of degree 2
    int degree = 2;
    std::vector<float> coeffs = fitter.fitPolynomial(x, y, degree);

    // Print coefficients
    Serial.println("Fitted coefficients:");
    for (float c : coeffs) {
        Serial.println(c);
    }
}
