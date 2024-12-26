// types.h
#pragma once
#include <CircularBuffer.h>

enum class ScreenID {
    MAIN_STATUS,
    CHANNEL_DETAIL,
    CHARGE_GRAPH,
    MAINTENANCE,
    SETTINGS
};

enum class ChargeState {
    IDLE,
    CHARGING,
    DONE,
    ERROR,
    MAINTENANCE
};

enum class IRCommand {
    UP = 0x01,
    DOWN = 0x02,
    LEFT = 0x03,
    RIGHT = 0x04,
    SELECT = 0x05,
    BACK = 0x06,
    NUM_1 = 0x11,
    NUM_2 = 0x12,
    NUM_3 = 0x13,
    NUM_4 = 0x14,
    PLUS = 0x1A,
    MINUS = 0x1B
};
