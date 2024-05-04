/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "config.h"

#include "systems.h"
#include "eepromcursor.h"

// Default Config ID
ConfigID::ConfigID() : ConfigID{0} {
}

Config::Config() {  // Default Settings
    // This function will only initialize data variables
    // data needs to be written manually to the EEPROM
}

template <std::size_t field>
inline decltype(std::get<field>(Config::Data())) & systemMapping(Systems& sys);

// Maps a subsystem configuration to a flag, whereby we state the subsystem we're mapping to
#define MAP_SYSTEM(name, type, variable)                      \
    template <>                                               \
    inline type& systemMapping<Config::name>(Systems & sys) { \
        return sys.variable;                                  \
    }

MAP_SYSTEM(VERSION, Version, version);
MAP_SYSTEM(ID, ConfigID, id);
MAP_SYSTEM(PCB, PcbTransform, imu.pcb_transform);
MAP_SYSTEM(MIX_TABLE, Airframe::MixTable, pilot.mix_table());
MAP_SYSTEM(MAG_BIAS, AK8963::MagBias, imu.magnetometer_bias());
MAP_SYSTEM(CHANNEL, Receiver::ChannelProperties, radioReceiver().channel);
MAP_SYSTEM(PID_PARAMETERS, Control::PIDParameters, control.pid_parameters);
MAP_SYSTEM(STATE_PARAMETERS, State::Parameters, state.parameters);
MAP_SYSTEM(LED_STATES, LED::States, led.states);
MAP_SYSTEM(DEVICE_NAME, DeviceName, name);
MAP_SYSTEM(VELOCITY_PID_PARAMETERS, Control::VelocityPIDParameters, control.velocity_pid_parameters);
MAP_SYSTEM(INERTIAL_BIAS, Imu::InertialBias, imu.bias);

#undef MAP_SYSTEM

bool isEmptyEEPROM() {
    return EEPROM.read(0) == 255;
}

Config readEEPROM() {
    Config CONFIG;
    if (isEmptyEEPROM()) {
        // No EEPROM values detected, re-initialize to default values
        Config().writeTo(EEPROMCursor());  // store the default values
        CONFIG = readEEPROM();
    } else {
        CONFIG.readFrom(EEPROMCursor());
        // Verify version and general settings
        if (!CONFIG.verify()) {
            // If the stored configuration isn't legal in any way, report it
            // via debug and reset it
            Config().writeTo(EEPROMCursor());  // store the default values
            CONFIG = readEEPROM();
        }
    }
    return CONFIG;
}

// No need to modify anything below this line

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type applyFieldsTo(const std::tuple<Tp...>& t, Systems& systems) {
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type applyFieldsTo(const std::tuple<Tp...>& t, Systems& systems) {
    systemMapping<I>(systems) = std::get<I>(t);
    applyFieldsTo<I + 1, Tp...>(t, systems);
    systems.parseConfig();
}

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type setFieldsFrom(std::tuple<Tp...>& t, Systems& systems) {
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type setFieldsFrom(std::tuple<Tp...>& t, Systems& systems) {
    std::get<I>(t) = systemMapping<I>(systems);
    setFieldsFrom<I + 1, Tp...>(t, systems);
}

Config::Config(Systems& sys) {
    setFieldsFrom(data, sys);
}

void Config::resetPartial(uint16_t submask, uint16_t led_mask) {
    resetFields(data, submask, led_mask);
}

void Config::applyTo(Systems& systems) const {
    applyFieldsTo(data, systems);
}

template <class T>
inline bool verifyArg(T& var) {
    return var.verify();
}

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), bool>::type verifyArgs(const std::tuple<Tp...>& t) {
    return true;
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), bool>::type verifyArgs(const std::tuple<Tp...>& t) {
    bool ok{verifyArg(std::get<I>(t))};
    return verifyArgs<I + 1, Tp...>(t) && ok;
}

bool Config::verify() const {
    return verifyArgs(data);
}
