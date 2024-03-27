// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "buzzer.hpp"


#define BEAP_SIZE (6*2 + 11)

Logger Buzzer::logger = Logger("Buzzer");
Buzzer::Buzzer() {
}

int8_t Buzzer::init() {
    pwm_pin = PwmPin::PWM_BUZZER;
    return 0;
}

void Buzzer::buzzerSet(uint32_t frequency, uint32_t duration) {
    if (HAL_GetTick() % 100) {
        static char buffer[90];
        sprintf(buffer, "%d %d", frequency, duration);
        logger.log_debug(buffer);
    } 
    PwmPeriphery::set_duration(pwm_pin, duration);
    PwmPeriphery::set_frequency(pwm_pin, frequency);
}

void Buzzer::process(bool error_flag) {
    if (error_flag != 0) {
        switch (error_melody) {
        case 127:
            buzzerSet(buzzer_frequency, buzzer_duration);
            break;
        case 0:
            buzzerBeapAnnoying();
            break;
        case 1:
            buzzerBeapTolerable();
            break;
        case 2:
            buzzerBeapBimmer();
            break;
        default:
            break;
        }
    }
}

void Buzzer::update_params() {
    error_melody = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_ERROR_MELODY);
    arm_melody = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_ARM_MELODY);
    buzzer_frequency = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_FREQUENCY);
    buzzer_duration = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_DURATION);
}

static uint32_t bummer_volume[BEAP_SIZE] = {
    1, 1,
    1, 1,
    0, 0,
    1, 1,
    1, 1,
    0, 0,

    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1
};

static uint32_t bummer_freq[BEAP_SIZE] = {
    987, 987,
    1175, 1175,
    0, 0,
    1175, 1175,
    987, 987,
    0, 0,

    1318,
    1175,
    1318,
    1175,
    1318,
    1175,
    1318,
    1175,
    1318,
    1480,
    1480,
};

void Buzzer::buzzerBeapBimmer() {
    uint8_t seq = (HAL_GetTick() % 7000) / 200;
    if (seq < BEAP_SIZE) {
        buzzerSet(bummer_freq[seq], bummer_volume[seq]);
    } else {
        buzzerSet(0, 0);
    }
}

void Buzzer::buzzerBeapAnnoying() {
    const uint32_t beap_frequency = 2000;
    const uint32_t beap_duration =  (HAL_GetTick() % 1000 < 500) ? 200000 : 0;
    buzzerSet(beap_frequency, beap_duration);
}

void Buzzer::buzzerBeapTolerable() {
    const uint32_t beap_frequency = 432;
    const uint32_t beap_duration =  (HAL_GetTick() % 3000 < 500) ? 200000 : 0;
    buzzerSet(beap_frequency, beap_duration);
}
