// Fit the data
std::vector<float> coeffs = fitter.fitWithBernstein(x_data, y_data, degree);

// Evaluate at a single point
float x_eval = 5.0f;
float y_eval = fitter.evaluateBernstein(coeffs, x_eval, x_min, x_max);
Serial.print("Evaluated value at x = ");
Serial.print(x_eval);
Serial.print(": ");
Serial.println(y_eval);

// Evaluate over a range
std::vector<float> x_range = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
std::vector<float> y_range = fitter.evaluateBernsteinRange(coeffs, x_range, x_min, x_max);

Serial.println("Evaluated range:");
for (size_t i = 0; i < x_range.size(); ++i) {
    Serial.print("x = ");
    Serial.print(x_range[i]);
    Serial.print(", y = ");
    Serial.println(y_range[i]);
}
