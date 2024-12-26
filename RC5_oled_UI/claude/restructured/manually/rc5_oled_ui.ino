// main.cpp
#include "config.h"
#include "hardware.h"
#include "channel_manager.h"
#include "ui_manager.h"

ESP32Hardware hardware;
ChannelManager channelManager(hardware);
IRrecv irReceiver(Config::IR_RECEIVE_PIN);
UIManager uiManager(irReceiver, channelManager);

void setup() {
    Wire.begin();
    hardware.begin();
    irReceiver.enableIRIn();
    uiManager.begin();
}

void loop() {
    static uint32_t lastSensorUpdate = 0;
    uint32_t now = millis();
    
    // Update sensors at regular interval
    if (now - lastSensorUpdate >= Config::SENSOR_UPDATE_MS) {
        channelManager.update();
        lastSensorUpdate = now;
    }
    
    // Update UI
    uiManager.update();
    delay(Config::SCREEN_UPDATE_MS);
}
