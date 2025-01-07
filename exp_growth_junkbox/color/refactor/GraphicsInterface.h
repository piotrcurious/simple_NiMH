#ifndef GRAPHICS_INTERFACE_H
#define GRAPHICS_INTERFACE_H

#include <vector>
#include <string>

class GraphicsInterface {
public:
    virtual ~GraphicsInterface() {}
    virtual void begin() = 0;
    virtual void clear() = 0;
    virtual void drawPixel(int x, int y, int color) = 0;
    virtual void drawText(int x, int y, const std::string& text) = 0;
    virtual void display() = 0;
    virtual void visualizeGrowthAnalysis(
        const std::vector<float>& xData,
        const std::vector<float>& yData,
        const std::vector<float>& coeffs,
        bool growthDetected,
        float growthRate
    ) = 0;
    virtual void displayErrorState(const std::string& errorMsg) = 0;
};

#endif // GRAPHICS_INTERFACE_H
