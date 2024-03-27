// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "buzzer.hpp"
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "storage.h"
#include "periphery/pwm/pwm.hpp"

Buzzer::Buzzer() {
    
}

int8_t Buzzer::init() {
    return 0;
}

void set_params(uint32_t frequency, uint32_t duration) {

}

void process() {

}
