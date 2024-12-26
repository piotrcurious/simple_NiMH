// hardware.h
#pragma once

class HardwareManager {
public:
    virtual ~HardwareManager() = default;
    virtual void begin() = 0;
    virtual float readVoltage(uint8_t channel) = 0;
    virtual float readCurrent(uint8_t channel) = 0;
    virtual float readTemperature(uint8_t channel) = 0;
    virtual void setChargeCurrent(uint8_t channel, uint16_t currentMa) = 0;
};

class ESP32Hardware : public HardwareManager {
private:
    const uint8_t voltagePins[Config::NUM_CHANNELS] = {32, 33, 34, 35};
    const uint8_t currentPins[Config::NUM_CHANNELS] = {36, 37, 38, 39};
    const uint8_t tempPins[Config::NUM_CHANNELS + 1] = {25, 26, 27, 28, 29}; // +1 for ambient
    const uint8_t pwmPins[Config::NUM_CHANNELS] = {16, 17, 18, 19};
    
public:
    void begin() override {
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            pinMode(pwmPins[i], OUTPUT);
            analogWrite(pwmPins[i], 0);
        }
    }
    
    float readVoltage(uint8_t channel) override {
        float raw = analogRead(voltagePins[channel]) * 3.3f / 4095.0f;
        return raw * 1000.0f; // Convert to mV
    }
    
    // ... similar implementations for other hardware methods
};
