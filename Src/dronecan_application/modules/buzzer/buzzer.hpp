// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_MODULES_BUZZER_HPP_
#define SRC_MODULES_BUZZER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

class Buzzer {
public:
    Buzzer();
    int8_t init();
    void set_params(uint32_t frequency, uint32_t duration);
    void process();
};

#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_BUZZER_HPP_
