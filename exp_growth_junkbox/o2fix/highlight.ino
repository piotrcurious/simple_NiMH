// ... (previous code remains the same until AdvancedOLEDVisualizer class)

class AdvancedOLEDVisualizer {
private:
    Adafruit_SSD1306 display;
    
    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        if (in_max == in_min) return out_min;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Convert normalized coordinates to screen coordinates
    int16_t normalizedToScreenX(float x) const {
        return static_cast<int16_t>(mapFloat(x, 0.0f, 1.0f, 0, SCREEN_WIDTH - 1));
    }

    int16_t normalizedToScreenY(float y) const {
        return static_cast<int16_t>(mapFloat(y, 0.0f, 1.0f, SCREEN_HEIGHT - 24, 0));
    }

public:
    AdvancedOLEDVisualizer() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE) {}

    bool begin() {
        Wire.begin();
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            return false;
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        return true;
    }

    void clearDisplay() {
        display.clearDisplay();
    }

    void drawPoint(float x, float y) {
        int16_t screenX = normalizedToScreenX(x);
        int16_t screenY = normalizedToScreenY(y);
        display.drawPixel(screenX, screenY, SSD1306_WHITE);
    }

    // Draw a highlighted region for exponential growth features
    void highlightRegion(float startX, float endX, float confidence = 1.0f) {
        int16_t screenStartX = normalizedToScreenX(startX);
        int16_t screenEndX = normalizedToScreenX(endX);
        
        // Draw top and bottom borders
        const int16_t topY = 0;
        const int16_t bottomY = SCREEN_HEIGHT - 24;
        
        // Draw vertical borders
        display.drawLine(screenStartX, topY, screenStartX, bottomY, SSD1306_WHITE);
        display.drawLine(screenEndX, topY, screenEndX, bottomY, SSD1306_WHITE);
        
        // Draw dashed horizontal lines to indicate feature region
        for (int16_t x = screenStartX; x <= screenEndX; x += 4) {
            if ((x - screenStartX) % 8 < 4) {  // Create dashed pattern
                display.drawLine(x, topY, x, topY + 2, SSD1306_WHITE);
                display.drawLine(x, bottomY - 2, x, bottomY, SSD1306_WHITE);
            }
        }

        // Display confidence level
        display.setCursor(screenStartX, bottomY + 2);
        display.print(F("C:"));
        display.print(static_cast<int>(confidence * 100));
        display.print(F("%"));
    }

    void displayStats(const ExponentialFeature* features, size_t featureCount) {
        const int16_t statsY = SCREEN_HEIGHT - 20;
        display.setCursor(0, statsY);
        display.print(F("Features: "));
        display.print(featureCount);
        
        if (featureCount > 0) {
            // Display latest feature's growth rate
            display.setCursor(0, statsY + 10);
            display.print(F("Growth: "));
            display.print(features[featureCount - 1].growthRate, 1);
        }
    }

    void visualizeData(const CircularBuffer<float, MAX_DATA_POINTS>& timestamps,
                      const CircularBuffer<float, MAX_DATA_POINTS>& values,
                      const ExponentialFeature* features,
                      size_t featureCount) {
        clearDisplay();

        // Draw data points
        for (size_t i = 0; i < timestamps.size(); i++) {
            drawPoint(timestamps[i], values[i]);
        }

        // Draw active features
        for (size_t i = 0; i < featureCount; i++) {
            if (features[i].active) {
                highlightRegion(features[i].startTime, features[i].endTime, features[i].confidence);
            }
        }

        // Display statistics
        displayStats(features, featureCount);
        
        display.display();
    }

    void displayErrorState(const char* errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("ERROR:"));
        display.println(errorMsg);
        display.display();
    }
};

// Update the ExponentialGrowthDetector's displayFeatures method
private:
    void displayFeatures() {
        display.visualizeData(timestamps, values, features, featureCount);
    }

// ... (rest of the code remains the same)
