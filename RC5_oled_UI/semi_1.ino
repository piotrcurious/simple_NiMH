// First include previous code (Widget system, Layout system, etc.)
// Then add these new components:

// Additional Widgets
class ProgressBar : public Widget {
  float progress_; // 0.0 to 1.0
  bool showPercentage_;
  
public:
  ProgressBar(bool showPercentage = true)
    : progress_(0), showPercentage_(showPercentage) {}
    
  void setProgress(float progress) {
    progress_ = std::max(0.0f, std::min(1.0f, progress));
    setNeedsDisplay();
  }
  
  void draw(Adafruit_SSD1306& display) override {
    display.drawRect(bounds_.origin.x, bounds_.origin.y,
                    bounds_.size.width, bounds_.size.height, SSD1306_WHITE);
    
    int16_t fillWidth = bounds_.size.width * progress_;
    display.fillRect(bounds_.origin.x + 1, bounds_.origin.y + 1,
                    fillWidth - 2, bounds_.size.height - 2, SSD1306_WHITE);
                    
    if(showPercentage_) {
      display.setTextSize(1);
      String text = String(int(progress_ * 100)) + "%";
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
      
      int16_t textX = bounds_.origin.x + (bounds_.size.width - w)/2;
      int16_t textY = bounds_.origin.y + (bounds_.size.height - h)/2;
      
      display.setTextColor(progress_ > 0.5 ? SSD1306_BLACK : SSD1306_WHITE);
      display.setCursor(textX, textY);
      display.print(text);
    }
  }
};

class IconButton : public Widget {
  const uint8_t* icon_;
  uint8_t iconWidth_, iconHeight_;
  bool pressed_;
  std::function<void()> onPress_;
  
public:
  IconButton(const uint8_t* icon, uint8_t w, uint8_t h)
    : icon_(icon), iconWidth_(w), iconHeight_(h), pressed_(false) {}
    
  void setOnPress(std::function<void()> callback) {
    onPress_ = callback;
  }
  
  void press() {
    if(onPress_) onPress_();
  }
  
  Size intrinsicSize() const override {
    return Size(iconWidth_ + 4, iconHeight_ + 4);
  }
  
  void draw(Adafruit_SSD1306& display) override {
    display.drawBitmap(bounds_.origin.x + 2, bounds_.origin.y + 2,
                      icon_, iconWidth_, iconHeight_, SSD1306_WHITE);
    if(pressed_) {
      display.drawRect(bounds_.origin.x, bounds_.origin.y,
                      bounds_.size.width, bounds_.size.height,
                      SSD1306_WHITE);
    }
  }
};

// Screen Implementations
class MainMenuScreen : public Screen {
  struct ChannelPreview {
    std::unique_ptr<Label> nameLabel;
    std::unique_ptr<Label> statusLabel;
    std::unique_ptr<ProgressBar> progressBar;
  };
  
  std::vector<ChannelPreview> channelPreviews_;
  
public:
  MainMenuScreen() {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 2);
    
    // Title
    auto* titleLabel = new Label("NiMH Charger", 1, true);
    titleLabel->setConstraints(LayoutConstraints(Size(0, 10)));
    mainLayout->addChild(titleLabel);
    
    // Channel previews
    for(int i = 0; i < 4; i++) {
      auto* channelLayout = new StackLayout(StackLayout::Direction::Horizontal, 2);
      
      ChannelPreview preview;
      preview.nameLabel = std::make_unique<Label>("Ch" + String(i + 1));
      preview.statusLabel = std::make_unique<Label>("Ready");
      preview.progressBar = std::make_unique<ProgressBar>(false);
      
      channelLayout->addChild(preview.nameLabel.get());
      channelLayout->addChild(preview.statusLabel.get());
      channelLayout->addChild(preview.progressBar.get());
      
      channelPreviews_.push_back(std::move(preview));
      mainLayout->addChild(channelLayout);
    }
    
    rootWidget_.reset(mainLayout);
  }
  
  void handleInput(uint16_t key) override {
    // Handle channel selection
  }
  
  void updateChannel(int index, float voltage, float current, float progress) {
    if(index < channelPreviews_.size()) {
      auto& preview = channelPreviews_[index];
      preview.statusLabel->setText(String(voltage, 1) + "V " + 
                                 String(current, 0) + "mA");
      preview.progressBar->setProgress(progress);
    }
  }
};

class ChannelStatusScreen : public Screen {
  int channelIndex_;
  std::unique_ptr<Label> voltageLabel_;
  std::unique_ptr<Label> currentLabel_;
  std::unique_ptr<Label> temperatureLabel_;
  std::unique_ptr<Graph> voltageGraph_;
  std::unique_ptr<Graph> currentGraph_;
  std::unique_ptr<ProgressBar> chargeProgress_;
  
public:
  ChannelStatusScreen(int channel) : channelIndex_(channel) {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 4);
    
    // Header with channel info
    auto* headerLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    headerLayout->addChild(new Label("Channel " + String(channel + 1), 1, true));
    
    // Metrics layout
    auto* metricsLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    
    voltageLabel_ = std::make_unique<Label>("0.0V");
    currentLabel_ = std::make_unique<Label>("0mA");
    temperatureLabel_ = std::make_unique<Label>("25°C");
    
    metricsLayout->addChild(voltageLabel_.get());
    metricsLayout->addChild(currentLabel_.get());
    metricsLayout->addChild(temperatureLabel_.get());
    
    // Graphs
    auto* graphsLayout = new StackLayout(StackLayout::Direction::Vertical, 2);
    
    voltageGraph_ = std::make_unique<Graph>();
    voltageGraph_->setConstraints(LayoutConstraints(Size(0, 20)));
    
    currentGraph_ = std::make_unique<Graph>();
    currentGraph_->setConstraints(LayoutConstraints(Size(0, 20)));
    
    graphsLayout->addChild(voltageGraph_.get());
    graphsLayout->addChild(currentGraph_.get());
    
    // Progress bar
    chargeProgress_ = std::make_unique<ProgressBar>();
    chargeProgress_->setConstraints(LayoutConstraints(Size(0, 10)));
    
    // Combine all layouts
    mainLayout->addChild(headerLayout);
    mainLayout->addChild(metricsLayout);
    mainLayout->addChild(graphsLayout);
    mainLayout->addChild(chargeProgress_.get());
    
    rootWidget_.reset(mainLayout);
  }
  
  void handleInput(uint16_t key) override {
    // Handle channel control input
  }
  
  void updateData(float voltage, float current, float temp, float progress) {
    voltageLabel_->setText(String(voltage, 2) + "V");
    currentLabel_->setText(String(current, 0) + "mA");
    temperatureLabel_->setText(String(temp, 1) + "°C");
    voltageGraph_->addPoint(voltage);
    currentGraph_->addPoint(current);
    chargeProgress_->setProgress(progress);
  }
};

class ChargeSettingsScreen : public Screen {
  struct Setting {
    std::unique_ptr<Label> nameLabel;
    std::unique_ptr<Label> valueLabel;
    float value;
    float min;
    float max;
    float step;
  };
  
  int channelIndex_;
  std::vector<Setting> settings_;
  size_t selectedSetting_;
  
public:
  ChargeSettingsScreen(int channel) : channelIndex_(channel), selectedSetting_(0) {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 2);
    
    // Title
    mainLayout->addChild(new Label("Ch" + String(channel + 1) + " Settings", 1, true));
    
    // Settings
    addSetting(mainLayout, "Current", 500, 100, 2000, 100, "mA");
    addSetting(mainLayout, "Term Volt", 1.45, 1.0, 1.6, 0.05, "V");
    addSetting(mainLayout, "Term Temp", 45, 35, 50, 1, "°C");
    addSetting(mainLayout, "Profile", 0, 0, 3, 1, "", 
               {"Normal", "Gentle", "Aggr.", "Recovery"});
    
    rootWidget_.reset(mainLayout);
  }
  
private:
  void addSetting(StackLayout* layout, const String& name,
                 float initial, float min, float max, float step,
                 const String& unit,
                 const std::vector<String>& options = std::vector<String>()) {
    auto* settingLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    
    Setting setting;
    setting.nameLabel = std::make_unique<Label>(name);
    setting.valueLabel = std::make_unique<Label>("");
    setting.value = initial;
    setting.min = min;
    setting.max = max;
    setting.step = step;
    
    settingLayout->addChild(setting.nameLabel.get());
    settingLayout->addChild(setting.valueLabel.get());
    
    if(options.empty()) {
      setting.valueLabel->setText(String(initial) + unit);
    } else {
      setting.valueLabel->setText(options[int(initial)]);
    }
    
    layout->addChild(settingLayout);
    settings_.push_back(std::move(setting));
  }
};

class GraphViewScreen : public Screen {
  int channelIndex_;
  std::unique_ptr<Graph> mainGraph_;
  std::unique_ptr<Label> timeLabel_;
  std::unique_ptr<Label> valueLabel_;
  bool showVoltage_; // false = show current
  
public:
  GraphViewScreen(int channel) : channelIndex_(channel), showVoltage_(true) {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 2);
    
    // Header
    auto* headerLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    headerLayout->addChild(new Label("Ch" + String(channel + 1) + " Graph"));
    
    auto* toggleButton = new IconButton(UI::VOLTAGE_ICON, 8, 8);
    toggleButton->setOnPress([this]() { showVoltage_ = !showVoltage_; });
    headerLayout->addChild(toggleButton);
    
    // Main graph
    mainGraph_ = std::make_unique<Graph>();
    mainGraph_->setConstraints(LayoutConstraints(Size(0, 40)));
    
    // Info bar
    auto* infoLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    timeLabel_ = std::make_unique<Label>("0:00");
    valueLabel_ = std::make_unique<Label>("0.0V");
    
    infoLayout->addChild(timeLabel_.get());
    infoLayout->addChild(valueLabel_.get());
    
    mainLayout->addChild(headerLayout);
    mainLayout->addChild(mainGraph_.get());
    mainLayout->addChild(infoLayout);
    
    rootWidget_.reset(mainLayout);
  }
};

class MaintenanceScreen : public Screen {
  int channelIndex_;
  std::unique_ptr<Graph> voltageGraph_;
  std::unique_ptr<Label> voltageLabel_;
  std::unique_ptr<Label> currentLabel_;
  std::unique_ptr<Label> pulseLabel_;
  bool pulsing_;
  
public:
  MaintenanceScreen(int channel) : channelIndex_(channel), pulsing_(false) {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 4);
    
    // Header
    mainLayout->addChild(new Label("Ch" + String(channel + 1) + " Maintenance", 1, true));
    
    // Status
    auto* statusLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    voltageLabel_ = std::make_unique<Label>("0.0V");
    currentLabel_ = std::make_unique<Label>("0mA");
    pulseLabel_ = std::make_unique<Label>("Ready");
    
    statusLayout->addChild(voltageLabel_.get());
    statusLayout->addChild(currentLabel_.get());
    statusLayout->addChild(pulseLabel_.get());
    
    // Graph
    voltageGraph_ = std::make_unique<Graph>();
    voltageGraph_->setConstraints(LayoutConstraints(Size(0, 40)));
    
    // Controls info
    auto* controlsLayout = new StackLayout(StackLayout::Direction::Horizontal, 2);
    controlsLayout->addChild(new Label("GREEN: Charge"));
    controlsLayout->addChild(new Label("YELLOW: Discharge"));
    
    mainLayout->addChild(statusLayout);
    mainLayout->addChild(voltageGraph_.get());
    mainLayout->addChild(controlsLayout);
    
    rootWidget_.reset(mainLayout);
  }
  
  void handleInput(uint16_t key) override {
    switch(key) {
      case RemoteKeys::KEY_GREEN:
        startChargePulse();
        break;
      case RemoteKeys::KEY_YELLOW:
        startDischargePulse();
        break;
    }
  }
  
  void updateData(float voltage, float current) {
    voltageLabel_->setText(String(voltage, 2) + "V");
    currentLabel_->setText(String(current, 0) + "mA");
    voltageGraph_->addPoint(voltage);
  }
  
private:
  void startChargePulse() {
    pulsing_ = true;
    pulseLabel_->setText("Charging");
    // Implement actual pulse control
  }
  
  void startDischargePulse() {
    pulsing_ = true;
    pulseLabel_->setText("Discharging");
    // Implement actual pulse control
  }
};

// Main UI Controller Update
void BatteryChargerUI::switchToScreen(Screen::Type type, int channel) {
  switch(type) {
    case Screen::Type::MAIN_MENU:
      setScreen(std::make_unique<MainMenuScreen>());
      break;
    case Screen::Type::CHANNEL_STATUS:
      setScreen(std::make_unique<ChannelStatusScreen>(channel));
      break;
// Continue from previous implementation...

class DiagnosticsScreen : public Screen {
  int channelIndex_;
  struct DiagnosticItem {
    std::unique_ptr<Label> nameLabel;
    std::unique_ptr<Label> valueLabel;
    std::function<String()> updateFunc;
  };
  std::vector<DiagnosticItem> items_;
  
public:
  DiagnosticsScreen(int channel) : channelIndex_(channel) {
    auto* mainLayout = new StackLayout(StackLayout::Direction::Vertical, 2);
    
    // Header
    mainLayout->addChild(new Label("Ch" + String(channel + 1) + " Diagnostics", 1, true));
    
    // Add diagnostic items
    addDiagItem(mainLayout, "Internal R", 
                [this]() { return String(getInternalResistance(), 3) + " Ω"; });
    
    addDiagItem(mainLayout, "Capacity", 
                [this]() { return String(getCapacity(), 0) + " mAh"; });
    
    addDiagItem(mainLayout, "Cycles", 
                [this]() { return String(getCycleCount()); });
    
    addDiagItem(mainLayout, "Health", 
                [this]() { return getBatteryHealth(); });
    
    addDiagItem(mainLayout, "Peak V", 
                [this]() { return String(getPeakVoltage(), 2) + " V"; });
    
    addDiagItem(mainLayout, "Trickle I", 
                [this]() { return String(getTrickleCurrent(), 0) + " mA"; });
    
    rootWidget_.reset(mainLayout);
  }
  
private:
  void addDiagItem(StackLayout* layout, const String& name, 
                   std::function<String()> updateFunc) {
    auto* itemLayout = new StackLayout(StackLayout::Direction::Horizontal, 4);
    
    DiagnosticItem item;
    item.nameLabel = std::make_unique<Label>(name);
    item.valueLabel = std::make_unique<Label>("");
    item.updateFunc = updateFunc;
    
    itemLayout->addChild(item.nameLabel.get());
    itemLayout->addChild(item.valueLabel.get());
    
    layout->addChild(itemLayout);
    items_.push_back(std::move(item));
  }
  
  void updateValues() {
    for(auto& item : items_) {
      item.valueLabel->setText(item.updateFunc());
    }
  }
  
  // Diagnostic measurement functions
  float getInternalResistance() {
    // Implement voltage drop measurement during load change
    return 0.150; // Example: 150mΩ
  }
  
  int getCapacity() {
    // Return accumulated charge
    return 2000; // Example: 2000mAh
  }
  
  int getCycleCount() {
    // Return stored cycle count
    return 10;
  }
  
  String getBatteryHealth() {
    float health = 95.5; // Calculate based on capacity and internal resistance
    if(health > 90) return "Good";
    if(health > 70) return "Fair";
    return "Poor";
  }
  
  float getPeakVoltage() {
    return 1.48; // Return highest voltage seen during charge
  }
  
  int getTrickleCurrent() {
    return 50; // Return maintenance current
  }
};

// Battery Analysis Feature
class BatteryAnalyzer {
public:
  struct AnalysisResult {
    float internalResistance;
    float capacity;
    float selfDischarge;
    bool hasShortCircuit;
    bool isReversed;
    String recommendation;
  };
  
  AnalysisResult analyze(int channel) {
    AnalysisResult result;
    
    // Perform analysis sequence
    result.internalResistance = measureInternalResistance(channel);
    result.capacity = measureCapacity(channel);
    result.selfDischarge = measureSelfDischarge(channel);
    result.hasShortCircuit = checkShortCircuit(channel);
    result.isReversed = checkReversedPolarity(channel);
    
    // Generate recommendation
    if(result.hasShortCircuit) {
      result.recommendation = "Replace battery - internal short detected";
    } else if(result.isReversed) {
      result.recommendation = "Check battery orientation";
    } else if(result.internalResistance > 0.5) {
      result.recommendation = "Battery showing high wear - consider replacement";
    } else if(result.capacity < 0.7 * getNominalCapacity(channel)) {
      result.recommendation = "Battery capacity degraded - consider replacement";
    } else if(result.selfDischarge > 20) {
      result.recommendation = "High self-discharge - check battery condition";
    } else {
      result.recommendation = "Battery in good condition";
    }
    
    return result;
  }
  
private:
  float measureInternalResistance(int channel) {
    // Apply load change and measure voltage response
    return 0.0;
  }
  
  float measureCapacity(int channel) {
    // Perform discharge test
    return 0.0;
  }
  
  float measureSelfDischarge(int channel) {
    // Monitor voltage drop over time
    return 0.0;
  }
  
  bool checkShortCircuit(int channel) {
    // Check for abnormal voltage/current behavior
    return false;
  }
  
  bool checkReversedPolarity(int channel) {
    // Check voltage polarity
    return false;
  }
  
  float getNominalCapacity(int channel) {
    // Get stored nominal capacity
    return 2000.0;
  }
};

// Enhanced UI Controller
class BatteryChargerUI {
  Adafruit_SSD1306 display;
  IRrecv irrecv;
  std::unique_ptr<Screen> currentScreen_;
  BatteryAnalyzer analyzer_;
  std::array<BatteryChannel, 4> channels_;
  Rect screenBounds_;
  bool analysisInProgress_;
  
public:
  BatteryChargerUI()
    : display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, Config::OLED_RESET),
      irrecv(Config::IR_PIN),
      screenBounds_(0, 0, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT),
      analysisInProgress_(false) {
  }
  
  void begin() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, Config::SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;);
    }
    
    irrecv.enableIRIn();
    setScreen(std::make_unique<MainMenuScreen>());
  }
  
  void update() {
    handleInput();
    updateChannels();
    updateDisplay();
  }
  
private:
  void handleInput() {
    if (irrecv.decode(&results)) {
      uint16_t key = results.value;
      
      if(key == RemoteKeys::KEY_MENU) {
        setScreen(std::make_unique<MainMenuScreen>());
      }
      else if(currentScreen_) {
        currentScreen_->handleInput(key);
      }
      
      irrecv.resume();
    }
  }
  
  void updateChannels() {
    for(size_t i = 0; i < channels_.size(); i++) {
      auto& channel = channels_[i];
      
      // Update measurements
      channel.updateMeasurements();
      
      // Check for charge completion
      if(channel.isCharging() && channel.checkChargeComplete()) {
        channel.stopCharging();
        showNotification("Channel " + String(i + 1) + " complete");
      }
      
      // Check for safety conditions
      if(channel.checkSafetyThresholds()) {
        channel.stopCharging();
        showNotification("Channel " + String(i + 1) + " safety stop");
      }
    }
  }
  
  void updateDisplay() {
    display.clearDisplay();
    
    if(currentScreen_) {
      currentScreen_->draw(display);
    }
    
    // Draw notification if active
    if(hasActiveNotification()) {
      drawNotification();
    }
    
    display.display();
  }
  
  void showNotification(const String& message, uint16_t duration = 2000) {
    // Implementation of notification system
  }
  
  void startBatteryAnalysis(int channel) {
    if(!analysisInProgress_) {
      analysisInProgress_ = true;
      
      // Show analysis progress screen
      setScreen(std::make_unique<ProgressScreen>("Analyzing Battery",
        [this, channel](ProgressScreen* screen) {
          screen->setProgress(0.2);
          screen->setStatus("Measuring internal resistance...");
          auto result = analyzer_.analyze(channel);
          
          // Show results
          setScreen(std::make_unique<DiagnosticsScreen>(channel));
          analysisInProgress_ = false;
        }
      ));
    }
  }
};

// Main program
BatteryChargerUI chargerUI;

void setup() {
  Serial.begin(115200);
  chargerUI.begin();
}

void loop() {
  chargerUI.update();
}
