#include <Arduino.h>
#include <TFT_eSPI.h>
#include <vector>
#include <numeric>
#include <cmath>

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

#define MAX_DATASET_WINDOW 100
#define GROWTH_EST_BACKWARD_TIME_WINDOW 5.0 // seconds
#define GROWTH_EST_FORWARD_TIME_WINDOW 2.0 // seconds

class AdvancedPolynomialFitter {
public:
    double calculateMSE(const std::vector<float>& coeffs,
                        const std::vector<float>& x,
                        const std::vector<float>& y) {
        double meanSquaredError = 0.0;
        double mean = 0.0;
        double M2 = 0.0;

        for (size_t i = 0; i < x.size(); ++i) {
            float prediction = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                prediction += coeffs[j] * pow(x[i], j);
            }
            float error = prediction - y[i];
            double squaredError = error * error;
            double delta = squaredError - mean;
            mean += delta / (i + 1);
            M2 += delta * (squaredError - mean);
        }

        meanSquaredError = mean;
        return meanSquaredError;
    }

    std::vector<float> fitPolynomial(const std::vector<float>& x,
                                      const std::vector<float>& y, int degree) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        std::vector<float> x_norm = x;
        double x_min = *std::min_element(x.begin(), x.end());
        double x_max = *std::max_element(x.begin(), x.end());
        std::transform(x.begin(), x.end(), x_norm.begin(), [x_min](double val) { return val - x_min; });

        size_t n = x_norm.size();
        size_t m = degree + 1;

        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A[i][j] = xi;
                xi *= x_norm[i];
            }
        }

        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += A[i][j] * y[i];
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += A[i][j] * A[i][k];
                }
            }
        }

        std::vector<double> coeffs = solveLinearSystem(ATA, ATy);
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }

private:
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A,
                                          std::vector<double>& b) {
        size_t n = A.size();

        for (size_t k = 0; k < n; ++k) {
            for (size_t i = k + 1; i < n; ++i) {
                if (fabs(A[i][k]) > fabs(A[k][k])) {
                    std::swap(A[k], A[i]);
                    std::swap(b[k], b[i]);
                }
            }

            for (size_t i = k + 1; i < n; ++i) {
                double factor = A[i][k] / A[k][k];
                for (size_t j = k; j < n; ++j) {
                    A[i][j] -= factor * A[k][j];
                }
                b[i] -= factor * b[k];
            }
        }

        std::vector<double> x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }
};

class AdvancedTFTVisualizer {
private:
    TFT_eSPI display;

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    AdvancedTFTVisualizer() : display(TFT_eSPI()) {}

    void begin() {
        display.init();
        display.setRotation(1);  // Landscape mode
        display.fillScreen(TFT_BLACK);
    }

    void visualizeGrowthAnalysis(const std::vector<float>& xData,
                                 const std::vector<float>& yData,
                                 const std::vector<float>& coeffs,
                                 bool growthDetected,
                                 float growthRate) {
        display.fillScreen(TFT_BLACK);

        float xMin = *std::min_element(xData.begin(), xData.end());
        float xMax = *std::max_element(xData.begin(), xData.end());
        float yMin = *std::min_element(yData.begin(), yData.end());
        float yMax = *std::max_element(yData.begin(), yData.end());

        for (size_t i = 0; i < xData.size(); ++i) {
            int x = mapFloat(xData[i], xMin, xMax, 0, SCREEN_WIDTH - 1);
            int y = mapFloat(yData[i], yMin, yMax, SCREEN_HEIGHT - 40, 0);
            display.drawPixel(x, y, TFT_WHITE);
        }

        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            float dataX = mapFloat(x, 0, SCREEN_WIDTH - 1, xMin, xMax);

            float yFitted = 0.0;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                yFitted += coeffs[j] * pow(dataX - xMin, j);
            }

            int y = mapFloat(yFitted, yMin, yMax, SCREEN_HEIGHT - 40, 0);
            display.drawPixel(x, y, TFT_GREEN);
        }

        display.setCursor(10, SCREEN_HEIGHT - 30);
        display.setTextColor(TFT_WHITE);
        display.printf("Growth: %s", growthDetected ? "YES" : "NO");
        display.setCursor(10, SCREEN_HEIGHT - 20);
        display.printf("Rate: %.2f", growthRate);
        display.display();
    }
};

// Main classes and logic remain similar, using AdvancedTFTVisualizer instead of the OLED variant.
