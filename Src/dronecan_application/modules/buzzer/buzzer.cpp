// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "buzzer.hpp"
#define BEAP_SIZE (18)


uint32_t Buzzer::crnt_time_ms = 0;
Logger Buzzer::logger = Logger("Buzzer");
uint32_t Buzzer::cmd_end_time_ms = 0;
BeepCommand_t command = {};
uint32_t Buzzer::buzzer_frequency = 1;
uint32_t Buzzer::buzzer_duration = 0;

const uint32_t quarter = 207;

static uint32_t bummer_delay[BEAP_SIZE] = {
    quarter *3/2,
    quarter * 13 / 2,
    quarter / 2,        // pause
    quarter *3/2,
    quarter * 13 / 2,
    quarter / 2,        // pause
    quarter,
    quarter,
    quarter,
    quarter,
    quarter,
    quarter,
    quarter,
    quarter,

    quarter,
    quarter * 13 / 2,
    quarter * 2,
};

static uint32_t bummer_freq[BEAP_SIZE] = {
  659,
  784,
  1,
  784,
  659,
  1,

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
  1
};


int8_t Buzzer::init() {
    PwmPeriphery::init(pwm_pin);
    PwmPeriphery::set_duration(pwm_pin, 0);

    auto sub_id = uavcanSubscribe(UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_SIGNATURE,
                                  UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_ID,
                                  callback);
    if (sub_id < 0) {
        logger.log_info("subscribe failed");
        return -1;
    }
    return 0;
}

void Buzzer::buzzerSet(uint32_t frequency) {
    PwmPeriphery::set_frequency(pwm_pin, frequency);
    uint32_t arr_ctr = PwmPeriphery::get_period(pwm_pin);
    PwmPeriphery::set_duration(pwm_pin, arr_ctr / 2 - 1);
}

void Buzzer::process(uint8_t curr_error_flag) {
    crnt_time_ms = HAL_GetTick();
    if (crnt_time_ms < 5000) {
        return;
    }

    error_flag = error_flag || curr_error_flag;
    static uint32_t next_upd_ms = 0;
    if (error_flag != 0) {
        play_melody();
    } else {
        if (crnt_time_ms > cmd_end_time_ms) {
            command.duration = 0;
            PwmPeriphery::set_duration(pwm_pin, 0);
        }
    }

    if (crnt_time_ms > next_upd_ms) {
        update_params();
        next_upd_ms += 200;
    }

    if (verbose) {
        publish_command();
    }
}

void Buzzer::update_params() {
    error_melody = (Melody)paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_ERROR_MELODY);
    arm_melody = (Melody)paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_ARM_MELODY);
    error_buzzer_frequency = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_FREQUENCY);
    verbose = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_VERBOSE);
    error_buzzer_period = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_BEEP_PERIOD);
    float fraction = paramsGetIntegerValue(IntParamsIndexes::PARAM_BUZZER_BEEP_FRACTION) * 0.01f;
    error_buzzer_sound_duration = error_buzzer_period * fraction;
}

void Buzzer::play_melody() {
    bool beep_flag = false;
    switch (error_melody) {
    case Melody::MANUAL:
        buzzerSet(error_buzzer_frequency);
        beep_flag = (crnt_time_ms % error_buzzer_period < error_buzzer_sound_duration);
        if (!beep_flag) PwmPeriphery::set_duration(pwm_pin, 0);
        break;
    case Melody::ANOYING:
        buzzerBeepAnnoying();
        break;
    case Melody::TOLERABLE:
        buzzerBeepTolerable();
        break;
    case Melody::BUMER:
        buzzerBeepBummer();
        break;
    default:
        break;
    }
}

void Buzzer::callback(CanardRxTransfer* transfer) {
    int8_t res = dronecan_equipment_indication_beep_command_deserialize(transfer, &command);

    if (res >= 0) {
        buzzerSet(command.frequency);
        cmd_end_time_ms = transfer->timestamp_usec / 1000.0 + command.duration * 1000;
    }
}

void Buzzer::buzzerBeepAnnoying() {
    const uint32_t beep_frequency = 2000;
    buzzerSet(beep_frequency);
    bool beep_flag =  (crnt_time_ms % 1000 < 500) ? true : false;
    if (!beep_flag) PwmPeriphery::set_duration(pwm_pin, 0);
}

void Buzzer::buzzerBeepTolerable() {
    const uint32_t beep_frequency = 432;
    buzzerSet(beep_frequency);
    bool beep_flag =  (crnt_time_ms % 3000 < 500) ? true : false;
    if (!beep_flag) PwmPeriphery::set_duration(pwm_pin, 0);
}

void Buzzer::buzzerBeepBummer() {
    static uint8_t n_note = 0;
    static uint32_t last_note_start_time_ms = crnt_time_ms;

    if (n_note < BEAP_SIZE) {
        buzzerSet(bummer_freq[n_note]);
        if (bummer_freq[n_note] == 1) {
            PwmPeriphery::set_duration(pwm_pin, 0);
        }
        if (crnt_time_ms - last_note_start_time_ms > bummer_delay[n_note]) {
            last_note_start_time_ms = crnt_time_ms;
            n_note++;
        }
    } else {
        n_note = 0;
        PwmPeriphery::set_duration(pwm_pin, 0);
    }
}

void Buzzer::publish_command(){
    static uint32_t next_pub_ms = 100;
    static uint8_t transfer_id = 0;

    if (next_pub_ms < crnt_time_ms) {
        next_pub_ms += 100;

        float frequency = PwmPeriphery::get_frequency(pwm_pin);
        float duration = command.duration;

        BeepCommand_t cmd {};
        cmd.duration = duration;
        cmd.frequency = frequency;

        dronecan_equipment_indication_beep_command_publish(&cmd, &transfer_id);
        transfer_id++;
    }
}
