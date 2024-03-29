// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "gate_monitor.hpp"


uint16_t GateMonitor::gate_threshold = 0;
GateMonitor::GateMonitor(): logger("GateMonitor") {}

void GateMonitor::init(const char* logger_source) {
    logger.init(logger_source);
    is_gate_broken[n_gates] = {};
}

ModuleStatus GateMonitor::process() {
    uint32_t crnt_time_ms = HAL_GetTick();
    static uint32_t next_error_publish_ms = 0;

    if (crnt_time_ms > 5000) {

        update_params();
        check_gates();

        if (crnt_time_ms > next_error_publish_ms){
            uint32_t shift_next_publish_ms = 0;
            for (int i = 0; i < n_gates; i++) {
                if (is_gate_broken[i] == 1 && 
                    crnt_time_ms > next_error_publish_ms) {
                    static char gate_msg[32];
                    sprintf(gate_msg, "Gate failure (%d)\n", i+2);
                    logger.log_error(gate_msg);
                    shift_next_publish_ms = 1000;
                }
            }

            next_error_publish_ms = crnt_time_ms + shift_next_publish_ms;
        }
    }
    return error_flag;
}

void GateMonitor::update_params() {
    gate_threshold = 
        paramsGetIntegerValue(IntParamsIndexes::PARAM_GATE_THRESHOLD);
}


void GateMonitor::check_gates() {
    for (int i = 0; i < n_gates; i++) {
        if (AdcPeriphery::get(gate_channels[i]) < gate_threshold) {
            is_gate_broken[i] = 1; 
            error_flag = ModuleStatus::MODULE_ERROR;
        }
    }
}
