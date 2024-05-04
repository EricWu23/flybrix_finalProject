/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>
#include <i2c_t3.h>

#define BETA
#define BOARD_ADC_REF ADC_REFERENCE::REF_3V3  // ADC_REF_1V2

namespace board {
enum Pins : uint8_t {
    GREEN_LED = 13,  // 50
    RED_LED = 27,    // 54

    V0_DETECT = A13,  // ADC0_DM3
    I0_DETECT = A10,  // ADC0_DP0
    I1_DETECT = A11,  // ADC0_DM0

    RX_DAT = 3,  // 28  --- MUST BE PIN 3

    MPU_INTERRUPT = 17,  // 36
};

constexpr i2c_pins I2C_PINS{I2C_PINS_18_19};
constexpr i2c_pullup I2C_PULLUP{I2C_PULLUP_EXT};

constexpr uint8_t PWM[]{
    25,  // 42
    32,  // 41
    22,  // 44
    10,  // 49
    23,  // 45
    21,  // 63
    9,   // 46
    5,   // 64
};

constexpr uint8_t FTM[]{
    // TODO: properly consider right FTM pins
    25,  // 42 | PWM[0]
    22,  // 44 | PWM[2]
};

namespace spi {
enum SdCardPins : uint8_t {
    MOSI = 7,
    MISO = 8,
    SCK = 14,
};
enum ChipSelect : uint8_t {
    SD_CARD = 15,  // 43
    EXT = 2,       // 57
};
}  // namespace spi

namespace led {
enum PositionSimpleName : int8_t {
    FRONT = 1,
    BACK = -1,
    LEFT = -1,
    RIGHT = 1,
};
struct Position {
    static constexpr Position Min() {
        return Position{-128, -128};
    }
    static constexpr Position Max() {
        return Position{127, 127};
    }
    int8_t x;  // left < 0 < right
    int8_t y;  // back < 0 < front
};

constexpr uint8_t DATA_PIN{11};  // 51
constexpr uint8_t COUNT{4};
constexpr Position POSITION[]{
    {LEFT, BACK},
    {LEFT, FRONT},
    {RIGHT, FRONT},
    {RIGHT, BACK},
};
}  // namespace led

namespace bluetooth {
constexpr uint8_t RESET{28};
constexpr uint8_t MODE{30};
}  // namespace bluetooth
}  // namespace board

#endif
