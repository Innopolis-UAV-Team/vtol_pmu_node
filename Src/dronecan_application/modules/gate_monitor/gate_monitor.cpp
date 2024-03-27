// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "gate_monitor.hpp"


GateStatus GateMonitor::gate_2_status = GateStatus::GateStatusOK;
GateStatus GateMonitor::gate_3_status = GateStatus::GateStatusOK;
GateStatus GateMonitor::gate_4_status = GateStatus::GateStatusOK;

uint16_t GateMonitor::gate_threshold = 0;
uint16_t GateMonitor::number_of_broken_gates = 0;

GateMonitor::GateMonitor(): logger("GateMonitor") {}

int8_t GateMonitor::init() {
}

int8_t GateMonitor::process() {
    uint32_t crnt_time_ms = HAL_GetTick();

    if (crnt_time_ms < _last_spin_time_ms + 200) {
        return error_flag;
    }
    spin_once();
    return error_flag;

}

void GateMonitor::spin_once(){
    static uint32_t next_broken_error_publish_ms = 10000;
    static uint32_t next_threshold_error_publish_ms = 10000;

    uint32_t crnt_time_ms = HAL_GetTick();

    update_params();
    
    char buffer[90];
    if (crnt_time_ms > 5000) {
        check_gate(AdcChannel::ADC_GATE_2);
        check_gate(AdcChannel::ADC_GATE_3);
        check_gate(AdcChannel::ADC_GATE_4);
        number_of_broken_gates = gate_2_status | gate_3_status | gate_3_status;
        if (number_of_broken_gates != 0 & crnt_time_ms > next_broken_error_publish_ms) {
            sprintf(buffer, "BROKEN GATES N: %d", number_of_broken_gates);
            logger.log_error(buffer);
            next_broken_error_publish_ms += 10000;
            error_flag = 1;
        }

        if (gate_threshold > 3 & crnt_time_ms > next_threshold_error_publish_ms) {
            sprintf(buffer, "Threshold: %d, N gates: %d", gate_threshold, 3);
            logger.log_error(buffer);
            next_threshold_error_publish_ms += 10000;
            error_flag = 1;
        }
    }
    _last_spin_time_ms = crnt_time_ms;

}


void GateMonitor::update_params() {
    gate_threshold = paramsGetIntegerValue(IntParamsIndexes::PARAM_GATE_THRESHOLD);
}


int8_t GateMonitor::check_gate(AdcChannel gate_adc_channel) {
    GateStatus gate_status = GateStatus::GateStatusOK;
    switch (gate_adc_channel) {
    case AdcChannel::ADC_GATE_2:
        gate_status = gate_2_status;
        break;
    
    case AdcChannel::ADC_GATE_3:
        gate_status = gate_3_status;
        break;
    
    case AdcChannel::ADC_GATE_4:
        gate_status = gate_4_status;
        break;
    
    default:
        // logger.log_debug("No such gate specified in ADC");
        return -1;
    }
    if (gate_status != GateStatus::GateStatusERROR) {
        if (AdcPeriphery::get(gate_adc_channel) / 64.0 < 1.4) {
            gate_status = GateStatus::GateStatusERROR;
        }
    }
    return 0;
}