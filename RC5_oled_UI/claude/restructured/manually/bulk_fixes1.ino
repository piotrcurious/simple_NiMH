// hardware.h additions
void ESP32Hardware::setChargeCurrent(uint8_t channel, uint16_t currentMa) {
    if (channel >= Config::NUM_CHANNELS) return;
    
    // Convert current in mA to PWM duty cycle (0-255)
    uint8_t duty = map(currentMa, 0, Config::MAX_CHARGE_CURRENT_MA, 0, 255);
    analogWrite(pwmPins[channel], duty);
}

float ESP32Hardware::readCurrent(uint8_t channel) {
    if (channel >= Config::NUM_CHANNELS) return 0.0f;
    
    float raw = analogRead(currentPins[channel]) * 3.3f / 4095.0f;
    // Assuming current sense amplifier with 0.1 ohm shunt and 20V/V gain
    return (raw / (0.1f * 20.0f)) * 1000.0f; // Convert to mA
}

float ESP32Hardware::readTemperature(uint8_t channel) {
    if (channel > Config::NUM_CHANNELS) return 0.0f; // +1 for ambient sensor
    
    float raw = analogRead(tempPins[channel]) * 3.3f / 4095.0f;
    // Assuming 10k NTC thermistor with 10k series resistor
    float resistance = 10000.0f * (3.3f / raw - 1.0f);
    
    // Steinhart-Hart equation coefficients for 10k NTC
    const float A = 0.001129148f;
    const float B = 0.000234125f;
    const float C = 0.0000000876741f;
    
    float logR = log(resistance);
    float tempK = 1.0f / (A + B * logR + C * logR * logR * logR);
    return tempK - 273.15f; // Convert to Celsius
}

// ui_manager.h additions
void UIManager::handleInput(IRCommand cmd) {
    if (!currentScreen) return;
    
    switch (cmd) {
        case IRCommand::BACK:
            // Return to main status screen
            currentScreen = screens[static_cast<int>(ScreenID::MAIN_STATUS)].get();
            break;
            
        case IRCommand::NUM_1:
        case IRCommand::NUM_2:
        case IRCommand::NUM_3:
        case IRCommand::NUM_4: {
            // Calculate channel index (0-3) from number commands
            uint8_t channel = static_cast<uint8_t>(cmd) - 
                            static_cast<uint8_t>(IRCommand::NUM_1);
                            
            if (channel >= Config::NUM_CHANNELS) break;
            
            // Switch to channel detail screen
            auto detailScreen = dynamic_cast<ChannelDetailScreen*>(
                screens[static_cast<int>(ScreenID::CHANNEL_DETAIL)].get());
            if (detailScreen) {
                detailScreen->setChannel(channel);
                currentScreen = detailScreen;
            }
            break;
        }
            
        default:
            // Pass other commands to current screen
            currentScreen->handleInput(cmd);
            break;
    }
}

void UIManager::switchScreen(ScreenID screenId) {
    if (static_cast<int>(screenId) < 5) {
        currentScreen = screens[static_cast<int>(screenId)].get();
    }
}

// Add remaining screen initializations in UIManager::begin()
void UIManager::begin() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(WHITE);
    
    // Initialize all screens
    screens[static_cast<int>(ScreenID::MAIN_STATUS)] = 
        std::make_unique<MainStatusScreen>(display, channelManager);
    
    screens[static_cast<int>(ScreenID::CHANNEL_DETAIL)] = 
        std::make_unique<ChannelDetailScreen>(display, channelManager);
    
    screens[static_cast<int>(ScreenID::CHARGE_GRAPH)] = 
        std::make_unique<ChargeGraphScreen>(display, channelManager);
    
    screens[static_cast<int>(ScreenID::MAINTENANCE)] = 
        std::make_unique<MaintenanceScreen>(display, channelManager);
    
    screens[static_cast<int>(ScreenID::SETTINGS)] = 
        std::make_unique<SettingsScreen>(display, channelManager);
    
    currentScreen = screens[static_cast<int>(ScreenID::MAIN_STATUS)].get();
}

// Add missing drawGraph implementation used by multiple screens
void Screen::drawGraph(const RollingGraph& graph, uint8_t x, uint8_t y, 
                      uint8_t width, uint8_t height) {
    if (graph.values.isEmpty()) return;
    
    // Draw axes
    display.drawFastVLine(x, y, height, WHITE);
    display.drawFastHLine(x, y + height - 1, width, WHITE);
    
    // Calculate points
    uint8_t numPoints = std::min(width - 2, (uint8_t)graph.values.size());
    uint8_t startIdx = graph.values.size() - numPoints;
    
    // Plot points and connect with lines
    for (uint8_t i = 0; i < numPoints; i++) {
        float value = graph.values[startIdx + i];
        float normalized = (value - graph.minValue) * graph.scale;
        uint8_t plotY = y + height - 2 - normalized;
        
        // Draw point
        display.drawPixel(x + 1 + i, plotY, WHITE);
        
        // Connect to previous point if not first point
        if (i > 0) {
            float prevValue = graph.values[startIdx + i - 1];
            float prevNormalized = (prevValue - graph.minValue) * graph.scale;
            uint8_t prevY = y + height - 2 - prevNormalized;
            
            display.drawLine(x + i, prevY, x + 1 + i, plotY, WHITE);
        }
    }
    
    // Draw scale values
    display.setTextSize(1);
    display.setCursor(x - 20, y);
    display.print(graph.maxValue, 0);
    display.setCursor(x - 20, y + height - 8);
    display.print(graph.minValue, 0);
}

// Battery drawing utility used by status screens
void Screen::drawBattery(uint8_t x, uint8_t y, uint8_t level, bool charging) {
    const uint8_t width = 12;
    const uint8_t height = 6;
    const uint8_t tipWidth = 2;
    const uint8_t tipHeight = 2;
    
    // Draw main battery outline
    display.drawRect(x, y, width, height, WHITE);
    
    // Draw battery tip
    display.fillRect(x + width, y + (height - tipHeight)/2, 
                    tipWidth, tipHeight, WHITE);
    
    // Draw fill level
    if (level > 0) {
        uint8_t fillWidth = ((width - 2) * level) / 100;
        display.fillRect(x + 1, y + 1, fillWidth, height - 2, WHITE);
    }
    
    // Draw charging indicator
    if (charging) {
        // Lightning bolt
        display.drawLine(x + 5, y + 1, x + 7, y + height/2, WHITE);
        display.drawLine(x + 7, y + height/2, x + 5, y + height - 1, WHITE);
    }
}

// Add optional update method implementation for ChannelDetailScreen
void ChannelDetailScreen::update() {
    // Update channel data at regular intervals
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    if (now - lastUpdate >= Config::SENSOR_UPDATE_MS) {
        auto& ch = channels.getChannel(channelIndex);
        
        // Update graph data
        ch.voltageHistory.add(ch.voltageMv);
        ch.currentHistory.add(ch.currentMa);
        
        lastUpdate = now;
    }
}

// Add charging control methods to ChannelManager
void ChannelManager::startCharging(uint8_t channel, uint16_t currentMa) {
    if (channel >= Config::NUM_CHANNELS) return;
    
    auto& ch = channels[channel];
    if (ch.state != ChargeState::IDLE) return;
    
    // Initialize charging parameters
    ch.state = ChargeState::CHARGING;
    ch.targetCurrentMa = std::min(currentMa, Config::MAX_CHARGE_CURRENT_MA);
    ch.startTime = millis();
    ch.lastUpdateTime = ch.startTime;
    ch.capacityMah = 0;
    ch.peakVoltage = 0;
    ch.negDeltaV = 0;
    
    // Clear history graphs
    ch.voltageHistory.values.clear();
    ch.currentHistory.values.clear();
    
    // Set initial current
    hardware.setChargeCurrent(channel, ch.targetCurrentMa);
}

void ChannelManager::stopCharging(uint8_t channel) {
    if (channel >= Config::NUM_CHANNELS) return;
    
    auto& ch = channels[channel];
    if (ch.state == ChargeState::IDLE) return;
    
    ch.state = ChargeState::IDLE;
    ch.targetCurrentMa = 0;
    hardware.setChargeCurrent(channel, 0);
}

void ChannelManager::enterMaintenance(uint8_t channel) {
    if (channel >= Config::NUM_CHANNELS) return;
    
    auto& ch = channels[channel];
    if (ch.state != ChargeState::DONE) return;
    
    ch.state = ChargeState::MAINTENANCE;
    ch.targetCurrentMa = Config::MIN_CHARGE_CURRENT_MA;
    hardware.setChargeCurrent(channel, ch.targetCurrentMa);
}
