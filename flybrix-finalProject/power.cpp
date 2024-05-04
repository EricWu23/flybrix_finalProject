/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "power.h"
#include "board.h"

PowerMonitor::PowerMonitor() {
    // REFERENCE: https://github.com/pedvide/ADC
    pinMode(board::V0_DETECT, INPUT);
    pinMode(board::I0_DETECT, INPUT);
    pinMode(board::I1_DETECT, INPUT);

    adc.adc0->setReference(ADC_REFERENCE::REF_1V2);
    adc.adc0->setAveraging(1);                                       // set number of averages
    adc.adc0->setResolution(16);                                     // set bits of resolution
    adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // change the conversion speed
    adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);      // change the sampling speed

    adc.adc1->setReference(BOARD_ADC_REF);
    adc.adc1->setAveraging(1);                                       // set number of averages
    adc.adc1->setResolution(16);                                     // set bits of resolution
    adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // change the conversion speed
    adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);      // change the sampling speed
}

void PowerMonitor::updateLevels(void) {
    V0_ = getV0Raw();
    I0_ = getI0Raw();
    I1_ = getI1Raw();
}

float PowerMonitor::totalPower(void) const {
    return I0() * V0();
}

float PowerMonitor::electronicsPower(void) const {
    return I1() * 3.7;
}

uint16_t PowerMonitor::getV0Raw(void) {
    return (uint16_t)adc.analogRead(board::V0_DETECT, ADC_1);
}

uint16_t PowerMonitor::getI0Raw(void) {
    return (uint16_t)adc.analogRead(board::I0_DETECT, ADC_0);
}

uint16_t PowerMonitor::getI1Raw(void) {
    return (uint16_t)adc.analogRead(board::I1_DETECT, ADC_0);
}

namespace {
constexpr float calculateVoltageScale(float v_ref, float r_in, float r_out, float int_max) {
    return v_ref * r_in / r_out / int_max;
}

constexpr float calculateCurrentScale(float v_ref, float ic_scaling, float r, float int_max) {
    return v_ref / ic_scaling / r / int_max;
}

constexpr float V0_SCALE = calculateVoltageScale(3.3, 150 + 150, 150, 65536);
constexpr float I0_SCALE = calculateCurrentScale(3.3, 50, 0.005, 65536);
constexpr float I1_SCALE = calculateCurrentScale(3.3, 50, 0.033, 65536);
}

float PowerMonitor::V0(void) const {
    return V0_SCALE * V0_;
}

float PowerMonitor::I0(void) const {
    return I0_SCALE * I0_;
}

float PowerMonitor::I1(void) const {
    return I1_SCALE * I1_;
}
