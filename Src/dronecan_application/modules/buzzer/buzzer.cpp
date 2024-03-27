// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "buzzer.hpp"


#define BEAP_SIZE (6*2 + 11)

Logger Buzzer::logger = Logger("Buzzer");
Buzzer::Buzzer() {
}

int8_t Buzzer::init() {
    pwm_pin = PwmPin::PWM_BUZZER;
    PwmPeriphery::init(pwm_pin);
    PwmPeriphery::set_duration(pwm_pin, 0);
    return 0;
}

void Buzzer::buzzerSet(uint32_t frequency, uint32_t duration) {
    if (frequency != PwmPeriphery::get_frequency(pwm_pin))
    PwmPeriphery::set_frequency(pwm_pin, frequency);

    if (duration != PwmPeriphery::get_duration(pwm_pin))
    PwmPeriphery::set_duration(pwm_pin, duration);
}

void Buzzer::process(uint8_t error_flag) {
    static uint32_t next_upd_ms = 0;
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
    if (HAL_GetTick() > next_upd_ms) {
        update_params();
        next_upd_ms += 200;
    }
    static uint32_t next_logging_ms = 100;
    if (HAL_GetTick() > next_logging_ms) {
        static char buffer[90];
        sprintf(buffer, "%ld %ld %ld %ld", PwmPeriphery::get_duration(pwm_pin), PwmPeriphery::get_frequency(pwm_pin), TIM4->ARR, TIM4->CCR2);
        logger.log_debug(buffer);
        next_logging_ms += 500;
    }
    PwmPeriphery::set_frequency(pwm_pin, HAL_GetTick() % 1500);
}

void Buzzer::update_params() {
    auto new_error_melody = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_ERROR_MELODY);
    arm_melody = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_ARM_MELODY);
    buzzer_frequency = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_FREQUENCY);
    buzzer_duration = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_DURATION);
    if (new_error_melody != error_melody){
        logger.log_debug("Melody updated");
        error_melody = new_error_melody;
    }
}

// static uint32_t bummer_freq[BEAP_SIZE] = {
//     987, 987,
//     1175, 1175,
//     0, 0,
//     1175, 1175,
//     987, 987,
//     0, 0,

//     1318,
//     1175,
//     1318,
//     1175,
//     1318,
//     1175,
//     1318,
//     1175,
//     1318,
//     1480,
//     1480,
// };


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


static uint32_t bummer_delay[BEAP_SIZE] = {
    207,
    1241,
    207,
    1241,

    207,
    207, 
    207, 
    207, 

    207, 
    207,
    207, 
    207, 

    207, 
    1241,
    207, 
    1241,

    207, 
    1241,
    207, 
    207,

    207, 
    207,
    207, 
    // 207

    // 207, 
    // 207,
    // 207, 
    // 207,
};

static uint32_t bummer_freq[BEAP_SIZE] = {
  
  659,
  784,
  784,
  659,

  880,
  784,
  880,
  784,

  880,
  784,
  880,
  784,

  880,
  988,
  659,
  784,

  784,
  659,
  880,
  784,

  880,
  784,
  880,
//   784,

//   880,
//   784,
//   880,
};


void Buzzer::buzzerBeapBimmer() {
    uint8_t seq = (HAL_GetTick() % 7000) / 200;
    if (seq < BEAP_SIZE) {
        buzzerSet(bummer_freq[seq], bummer_delay[seq]*1000);
    } else {
        buzzerSet(0, 0);
    }
}
