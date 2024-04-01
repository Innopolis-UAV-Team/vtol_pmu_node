// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "buzzer.hpp"


#define BEAP_SIZE (6*2 + 11)

uint32_t Buzzer::crnt_time_ms = 0;
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
    if (frequency != PwmPeriphery::get_frequency(pwm_pin)) {
        PwmPeriphery::set_frequency(pwm_pin, frequency);
    }
    
    if (duration != PwmPeriphery::get_duration(pwm_pin))
    PwmPeriphery::set_duration(pwm_pin, duration);
}

void Buzzer::process(uint8_t error_flag) {
    crnt_time_ms = HAL_GetTick();
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
    if (crnt_time_ms > next_upd_ms) {
        update_params();
        next_upd_ms += 200;
    }
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

void Buzzer::buzzerBeapAnnoying() {
    const uint32_t beap_frequency = 2000;
    bool beap_flag =  (crnt_time_ms % 1000 < 500) ? true : false;
    buzzerSet(beap_frequency, (beap_flag? beap_frequency / 8 : 0));
}

void Buzzer::buzzerBeapTolerable() {
    const uint32_t beap_frequency = 432;
    bool beap_flag =  (crnt_time_ms % 3000 < 500) ? true : false;
    buzzerSet(beap_frequency, (beap_flag? beap_frequency / 2 : 0));
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
};

void Buzzer::buzzerBeapBimmer() {
    static uint8_t n_note = 0;
    static uint32_t last_note_start_time_ms = crnt_time_ms;

    if (n_note < BEAP_SIZE) {
        buzzerSet(bummer_freq[n_note], bummer_freq[n_note]/2);
        if (crnt_time_ms - last_note_start_time_ms > bummer_delay[n_note]) {
            last_note_start_time_ms = crnt_time_ms;
            n_note++;
        } 
    } else {
        n_note = 0;
        buzzerSet(0, 0);
    }
}
