// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "periphery/pwm/pwm.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim4;

int8_t PwmPeriphery::init(PwmPin pwm_pin) {
    switch (pwm_pin) {
        case PwmPin::PWM_BUZZER:
            if (HAL_OK != HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2)) {
                return -1;
            }
            TIM4->CCR2 = 1000;
            break;

        default:
            return -1;
    }

    return 0;
}

void PwmPeriphery::set_duration(const PwmPin pwm_pin, uint32_t duration_us) {
    switch (pwm_pin) {
        case PwmPin::PWM_BUZZER:
            TIM4->CCR2 = duration_us;
            break;

        default:
            break;
    }
}

void PwmPeriphery::set_frequency(const PwmPin pwm_pin, uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return;
    }

    volatile uint32_t* arr_reg;
    uint16_t period_us;

    switch (pwm_pin) {
        case PwmPin::PWM_BUZZER:
            arr_reg = &(htim4.Instance->ARR);
            period_us = 1000000 / frequency_hz;
            *arr_reg = period_us;
            break;

        default:
            break;
    }
}

uint32_t PwmPeriphery::get_duration(PwmPin pwm_pin) {
    uint32_t pwm_duration;

    switch (pwm_pin) {
        case PwmPin::PWM_BUZZER:
            pwm_duration = TIM4->CCR2;
            break;

        default:
            pwm_duration = 0;
            break;
    }

    return pwm_duration;
}

uint32_t PwmPeriphery::get_frequency(PwmPin pwm_pin) {
    uint32_t pwm_frequency;

    switch (pwm_pin) {
        case PwmPin::PWM_BUZZER:
            pwm_frequency = 1000000 / TIM4->ARR;
            break;

        default:
            pwm_frequency = 0;
            break;
    }

    return pwm_frequency;
}
