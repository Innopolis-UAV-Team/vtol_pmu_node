// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "gate_monitor.hpp"
#include "algorithms.hpp"


uint16_t GateMonitor::gate_threshold = 0;
GateMonitor::GateMonitor(): logger("GateMonitor") {}

void GateMonitor::init(const char* logger_source) {
    logger.init(logger_source);
}

ModuleStatus GateMonitor::process() {
    uint32_t crnt_time_ms = HAL_GetTick();
    if (crnt_time_ms < _last_spin_time_ms + 20) {
        return error_flag;
    }
    _last_spin_time_ms = crnt_time_ms;

    static uint32_t next_error_publish_ms = 0;

    if (crnt_time_ms > 5000) {
        update_params();
        check_gates();

        if (crnt_time_ms > next_error_publish_ms) {
            char buffer[] = "Gate failure (   )";
            for (int i = 0; i < n_gates;) {
                if (gates_info[i].is_broken) {
                    buffer[14 + i] = gates_info[i].name;
                }
            }

            if (error_flag != ModuleStatus::MODULE_OK) {
                logger.log_error(buffer);
                next_error_publish_ms = crnt_time_ms + 10000;
            }
        }
    }
    return error_flag;
}

void GateMonitor::update_params() {
    gate_threshold = paramsGetIntegerValue(IntParamsIndexes::PARAM_GATE_THRESHOLD);
    for (auto gate : gates_info) {
        paramsSetIntegerValue(gate.parameter, AdcPeriphery::get(gate.adc_channel));
    }
}

void GateMonitor::check_gates() {
    for (auto& gate : gates_info) {
        movingAverage(&gate.filtered, AdcPeriphery::get(gate.adc_channel), 50);
        if (gate.filtered > gate_threshold) {
            gate.is_broken = true;
            error_flag = ModuleStatus::MODULE_CRITICAL;
        }
    }
}
