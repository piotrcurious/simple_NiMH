// channel_manager.h
#pragma once

class ChannelManager {
private:
    HardwareManager& hardware;
    ChannelData channels[Config::NUM_CHANNELS];
    float ambientTemp = 0;
    
    void updateChannel(uint8_t channel) {
        auto& ch = channels[channel];
        if (ch.state == ChargeState::IDLE) return;
        
        // Update measurements
        ch.voltageMv = hardware.readVoltage(channel);
        ch.currentMa = hardware.readCurrent(channel);
        ch.batteryTemp = hardware.readTemperature(channel);
        ch.deltaTemp = ch.batteryTemp - ambientTemp;
        
        // Update histories
        ch.voltageHistory.add(ch.voltageMv);
        ch.currentHistory.add(ch.currentMa);
        
        // Update capacity
        uint32_t deltaTime = millis() - ch.lastUpdateTime;
        ch.capacityMah += (ch.currentMa * deltaTime) / 3600000.0f;
        
        // Peak voltage detection
        if (ch.voltageMv > ch.peakVoltage) {
            ch.peakVoltage = ch.voltageMv;
        } else if (ch.peakVoltage - ch.voltageMv > Config::DELTA_V_THRESHOLD_MV) {
            ch.negDeltaV = ch.peakVoltage - ch.voltageMv;
        }
        
        ch.lastUpdateTime = millis();
    }
    
    void checkTermination(uint8_t channel) {
        auto& ch = channels[channel];
        if (ch.state != ChargeState::CHARGING) return;
        
        bool terminate = false;
        
        // Check termination conditions
        if (ch.negDeltaV >= Config::DELTA_V_THRESHOLD_MV ||
            (ch.tempCheckEnabled && ch.deltaTemp > Config::DELTA_TEMP_THRESHOLD_C) ||
            (millis() - ch.startTime > Config::MAX_CHARGE_TIME_MS)) {
            terminate = true;
        }
        
        if (terminate) {
            stopCharging(channel);
            ch.state = ChargeState::DONE;
        }
    }
    
public:
    ChannelManager(HardwareManager& hw) : hardware(hw) {}
    
    void update() {
        ambientTemp = hardware.readTemperature(Config::NUM_CHANNELS); // Last sensor is ambient
        
        for (uint8_t i = 0; i < Config::NUM_CHANNELS; i++) {
            updateChannel(i);
            checkTermination(i);
            hardware.setChargeCurrent(i, channels[i].targetCurrentMa);
        }
    }
    
    void startCharging(uint8_t channel, uint16_t current) {
        if (channel >= Config::NUM_CHANNELS) return;
        
        auto& ch = channels[channel];
        ch.state = ChargeState::CHARGING;
        ch.startTime = ch.lastUpdateTime = millis();
        ch.capacityMah = 0;
        ch.peakVoltage = 0;
        ch.negDeltaV = 0;
        ch.targetCurrentMa = current;
    }
    
    void stopCharging(uint8_t channel) {
        if (channel >= Config::NUM_CHANNELS) return;
        
        auto& ch = channels[channel];
        ch.state = ChargeState::IDLE;
        ch.targetCurrentMa = 0;
    }
    
    ChannelData& getChannel(uint8_t channel) {
        return channels[channel];
    }
};
