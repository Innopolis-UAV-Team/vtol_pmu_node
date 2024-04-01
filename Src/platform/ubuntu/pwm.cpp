// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "periphery/pwm/pwm.hpp"

uint16_t pwm[(int)PwmPin::PWM_AMOUNT];
uint16_t pwm_freq[(int)PwmPin::PWM_AMOUNT];

int8_t PwmPeriphery::init(PwmPin pwm_pin) {
    (void)pwm_pin;
    return 0;
}

void PwmPeriphery::set_duration(const PwmPin pwm_pin, uint32_t duration_us) {
    pwm[(int) pwm_pin] = duration_us;
}

void PwmPeriphery::set_frequency(const PwmPin pwm_pin, uint32_t frequency_hz) {
    pwm_freq[(int) pwm_pin] = frequency_hz;
}

uint32_t PwmPeriphery::get_duration(PwmPin pwm_pin) {
    return pwm[(int)pwm_pin];
}

uint32_t PwmPeriphery::get_frequency(PwmPin pwm_pin) {
    return pwm_freq[(int)pwm_pin];
}
