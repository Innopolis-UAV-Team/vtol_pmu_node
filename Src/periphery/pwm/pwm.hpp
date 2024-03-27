// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_APPLICATION_PERIPHERY_PWM_HPP_
#define SRC_APPLICATION_PERIPHERY_PWM_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @note PWM pinout related to VTOL PMU node
 */
enum class PwmPin {
    PWM_BUZZER,      // PB7, TIM4_CH2
    PWM_AMOUNT,
};


class PwmPeriphery {
public:
    static int8_t init(PwmPin pin);
    static void set_duration(const PwmPin pin, uint32_t duration_us);
    static void set_frequency(const PwmPin pin, uint32_t frequency_hz);
    static uint32_t get_duration(PwmPin pin);
    static uint32_t get_frequency(PwmPin pin);
};

#ifdef __cplusplus
}
#endif

#endif  // SRC_APPLICATION_PERIPHERY_PWM_HPP_
