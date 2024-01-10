// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "periphery/pwm/pwm.hpp"

int8_t PwmPeriphery::init(PwmPin pwm_pin) {
    (void)pwm_pin;
    return 0;
}

void PwmPeriphery::set_duration(const PwmPin pwm_pin, uint32_t duration_us) {
    (void)pwm_pin;
    (void)duration_us;

}

uint32_t PwmPeriphery::get_duration(PwmPin pwm_pin) {
    (void)pwm_pin;
    return 0;
}
