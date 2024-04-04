// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "periphery/adc/adc.hpp"
#include <array>

int8_t AdcPeriphery::init() {
    _is_adc_already_inited = true;
    return 0;
}

uint16_t AdcPeriphery::get(AdcChannel channel) {
    if (!_is_adc_already_inited || channel >= AdcChannel::ADC_NUMBER_OF_CNANNELS) {
        return 0;
    }

    constexpr auto CHANNELS_AMOUNT = static_cast<uint16_t>(AdcChannel::ADC_NUMBER_OF_CNANNELS);
    constexpr std::array<uint16_t, CHANNELS_AMOUNT> adc_values = {
        320,    // vin
        10,     // gate2
        25,     // current
        10,     // gate3
        10,     // gate4
        3100,   // temperature
        10      // version
    };
    return adc_values[static_cast<uint8_t>(channel)];
}
