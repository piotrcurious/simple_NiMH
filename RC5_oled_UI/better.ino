#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <functional>

// Forward declarations
class LayoutManager;
class Widget;
class Screen;

// Basic configuration remains same as before, showing relevant parts here
namespace Config {
  constexpr uint8_t SCREEN_WIDTH = 128;
  constexpr uint8_t SCREEN_HEIGHT = 64;
  // ... other config constants
}

// Geometry and Layout Primitives
struct Point {
  int16_t x, y;
  Point(int16_t x_ = 0, int16_t y_ = 0) : x(x_), y(y_) {}
};

struct Size {
  int16_t width, height;
  Size(int16_t w = 0, int16_t h = 0) : width(w), height(h) {}
};

struct Rect {
  Point origin;
  Size size;
  
  Rect(int16_t x = 0, int16_t y = 0, int16_t w = 0, int16_t h = 0)
    : origin(x, y), size(w, h) {}
    
  int16_t left() const { return origin.x; }
  int16_t right() const { return origin.x + size.width; }
  int16_t top() const { return origin.y; }
  int16_t bottom() const { return origin.y + size.height; }
  
  Rect inset(int16_t dx, int16_t dy) const {
    return Rect(origin.x + dx, origin.y + dy, 
                size.width - 2*dx, size.height - 2*dy);
  }
};

// Layout Constraints
struct LayoutConstraints {
  Size minSize;
  Size maxSize;
  float flexGrow;
  float flexShrink;
  
  LayoutConstraints(Size min = Size(), Size max = Size(32767, 32767),
                   float grow = 0, float shrink = 1)
    : minSize(min), maxSize(max), flexGrow(grow), flexShrink(shrink) {}
};

// Abstract Widget Base Class
class Widget {
protected:
  Rect bounds_;
  LayoutConstraints constraints_;
  bool needsLayout_;
  bool needsDisplay_;
  
public:
  Widget() : needsLayout_(true), needsDisplay_(true) {}
  virtual ~Widget() = default;
  
  virtual void layout(const Rect& frame) {
    bounds_ = frame;
    needsLayout_ = false;
  }
  
  virtual void draw(Adafruit_SSD1306& display) = 0;
  virtual Size intrinsicSize() const { return Size(); }
  
  void setNeedsLayout() { needsLayout_ = true; }
  void setNeedsDisplay() { needsDisplay_ = true; }
  
  const Rect& bounds() const { return bounds_; }
  void setConstraints(const LayoutConstraints& c) { constraints_ = c; }
  const LayoutConstraints& constraints() const { return constraints_; }
};

// Layout Managers
class StackLayout : public Widget {
public:
  enum class Direction { Vertical, Horizontal };
  
private:
  std::vector<Widget*> children_;
  Direction direction_;
  uint8_t spacing_;
  
public:
  StackLayout(Direction dir = Direction::Vertical, uint8_t spacing = 2)
    : direction_(dir), spacing_(spacing) {}
    
  void addChild(Widget* child) {
    children_.push_back(child);
    setNeedsLayout();
  }
  
  void layout(const Rect& frame) override {
    bounds_ = frame;
    if(children_.empty()) return;
    
    float totalFlex = 0;
    float fixedSpace = (children_.size() - 1) * spacing_;
    
    // Calculate total flex and fixed space
    for(auto* child : children_) {
      const auto& constraints = child->constraints();
      totalFlex += constraints.flexGrow;
      if(constraints.flexGrow == 0) {
        Size intrinsic = child->intrinsicSize();
        fixedSpace += (direction_ == Direction::Vertical) 
                     ? intrinsic.height : intrinsic.width;
      }
    }
    
    // Distribute remaining space according to flex
    float flexSpace = (direction_ == Direction::Vertical)
                     ? frame.size.height - fixedSpace
                     : frame.size.width - fixedSpace;
                     
    float flexUnit = totalFlex > 0 ? flexSpace / totalFlex : 0;
    float pos = (direction_ == Direction::Vertical) ? frame.top() : frame.left();
    
    for(auto* child : children_) {
      const auto& constraints = child->constraints();
      Size childSize = child->intrinsicSize();
      
      if(direction_ == Direction::Vertical) {
        float height = constraints.flexGrow > 0
                      ? flexUnit * constraints.flexGrow
                      : childSize.height;
        child->layout(Rect(frame.left(), pos, frame.size.width, height));
        pos += height + spacing_;
      } else {
        float width = constraints.flexGrow > 0
                     ? flexUnit * constraints.flexGrow
                     : childSize.width;
        child->layout(Rect(pos, frame.top(), width, frame.size.height));
        pos += width + spacing_;
      }
    }
    
    needsLayout_ = false;
  }
  
  void draw(Adafruit_SSD1306& display) override {
    for(auto* child : children_) {
      child->draw(display);
    }
  }
};

// Specific Widgets
class Label : public Widget {
  String text_;
  uint8_t textSize_;
  bool centered_;
  
public:
  Label(const String& text, uint8_t size = 1, bool centered = false)
    : text_(text), textSize_(size), centered_(centered) {}
    
  void setText(const String& text) {
    text_ = text;
    setNeedsDisplay();
  }
  
  Size intrinsicSize() const override {
    return Size(text_.length() * 6 * textSize_, 8 * textSize_);
  }
  
  void draw(Adafruit_SSD1306& display) override {
    display.setTextSize(textSize_);
    if(centered_) {
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(text_.c_str(), 0, 0, &x1, &y1, &w, &h);
      display.setCursor(bounds_.origin.x + (bounds_.size.width - w)/2,
                       bounds_.origin.y + (bounds_.size.height - h)/2);
    } else {
      display.setCursor(bounds_.origin.x, bounds_.origin.y);
    }
    display.print(text_);
  }
};

class Graph : public Widget {
  std::vector<float> data_;
  float minValue_, maxValue_;
  bool autoScale_;
  
public:
  Graph(size_t points = Config::SCREEN_WIDTH)
    : data_(points), minValue_(0), maxValue_(1), autoScale_(true) {}
    
  void addPoint(float value) {
    data_.erase(data_.begin());
    data_.push_back(value);
    if(autoScale_) updateScale();
    setNeedsDisplay();
  }
  
  void setScale(float min, float max) {
    minValue_ = min;
    maxValue_ = max;
    autoScale_ = false;
    setNeedsDisplay();
  }
  
  void draw(Adafruit_SSD1306& display) override {
    if(data_.empty()) return;
    
    for(size_t i = 1; i < data_.size(); i++) {
      int16_t x1 = bounds_.origin.x + (i-1) * bounds_.size.width / data_.size();
      int16_t x2 = bounds_.origin.x + i * bounds_.size.width / data_.size();
      int16_t y1 = bounds_.bottom() - (data_[i-1] - minValue_) * 
                   bounds_.size.height / (maxValue_ - minValue_);
      int16_t y2 = bounds_.bottom() - (data_[i] - minValue_) * 
                   bounds_.size.height / (maxValue_ - minValue_);
      display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }
  }
  
private:
  void updateScale() {
    minValue_ = *std::min_element(data_.begin(), data_.end());
    maxValue_ = *std::max_element(data_.begin(), data_.end());
    float margin = (maxValue_ - minValue_) * 0.1;
    minValue_ -= margin;
    maxValue_ += margin;
  }
};

// Screen Base Class with Layout Support
class Screen {
protected:
  std::unique_ptr<Widget> rootWidget_;
  
public:
  virtual ~Screen() = default;
  virtual void handleInput(uint16_t key) = 0;
  
  void layout(const Rect& bounds) {
    if(rootWidget_) rootWidget_->layout(bounds);
  }
  
  void draw(Adafruit_SSD1306& display) {
    if(rootWidget_) rootWidget_->draw(display);
  }
};

// Example Channel Status Screen Implementation
class ChannelStatusScreen : public Screen {
  int channelIndex_;
  std::unique_ptr<Label> voltageLabel_;
  std::unique_ptr<Label> currentLabel_;
  std::unique_ptr<Graph> voltageGraph_;
  
public:
  ChannelStatusScreen(int channel) : channelIndex_(channel) {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 4);
    
    auto* headerLayout = new StackLayout(StackLayout::Direction::Horizontal, 2);
    voltageLabel_ = std::make_unique<Label>("0.0V");
    currentLabel_ = std::make_unique<Label>("0mA");
    headerLayout->addChild(voltageLabel_.get());
    headerLayout->addChild(currentLabel_.get());
    
    voltageGraph_ = std::make_unique<Graph>();
    
    mainLayout->addChild(headerLayout);
    mainLayout->addChild(voltageGraph_.get());
    
    rootWidget_.reset(mainLayout);
  }
  
  void handleInput(uint16_t key) override {
    // Handle input specific to channel status screen
  }
  
  void updateData(float voltage, float current) {
    voltageLabel_->setText(String(voltage, 1) + "V");
    currentLabel_->setText(String(current, 0) + "mA");
    voltageGraph_->addPoint(voltage);
  }
};

// Main Application Class
class BatteryChargerUI {
  Adafruit_SSD1306 display;
  IRrecv irrecv;
  std::unique_ptr<Screen> currentScreen_;
  Rect screenBounds_;
  
public:
  BatteryChargerUI()
    : display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, Config::OLED_RESET),
      irrecv(15),
      screenBounds_(0, 0, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT) {
  }
  
  void begin() {
    // Initialize hardware
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;);
    }
    irrecv.enableIRIn();
    
    // Set initial screen
    setScreen(std::make_unique<ChannelStatusScreen>(0));
  }
  
  void setScreen(std::unique_ptr<Screen> screen) {
    currentScreen_ = std::move(screen);
    currentScreen_->layout(screenBounds_);
  }
  
  void update() {
    // Handle IR input
    if (irrecv.decode(&results)) {
      if(currentScreen_) {
        currentScreen_->handleInput(results.value);
      }
      irrecv.resume();
    }
    
    // Update display
    display.clearDisplay();
    if(currentScreen_) {
      currentScreen_->draw(display);
    }
    display.display();
  }
};

// Global instance
BatteryChargerUI chargerUI;

void setup() {
  Serial.begin(115200);
  chargerUI.begin();
}

void loop() {
  chargerUI.update();
}
