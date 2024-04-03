// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_MODULES_GATE_MONITOR_HPP_
#define SRC_MODULES_GATE_MONITOR_HPP_

#include <array>
#include <stdint.h>
#include "periphery/adc/adc.hpp"
#include "params.hpp"
#include "logger.hpp"

#ifdef __cplusplus
extern "C" {
#endif

enum class ModuleStatus: uint8_t {
    MODULE_OK        = 0,
    MODULE_WARN      = 1,
    MODULE_CRITICAL  = 2,
    MODULE_ERROR     = 3
};

struct GateInfo {
    const AdcChannel adc_channel;
    const ParamIndex_t parameter;
    const char name;
    bool is_broken{false};
};

class GateMonitor {
public:
    GateMonitor();
    void init(const char* logger_source = "GateMonitor");
    ModuleStatus process();

private:
    void check_gates();
    void update_params();
    void spin_once();

    static const uint8_t n_gates = 3;

    std::array<GateInfo, n_gates> gates_info{{
        {AdcChannel::ADC_GATE_2, PARAM_GATE_2_RAW_ADC, '2'},
        {AdcChannel::ADC_GATE_3, PARAM_GATE_3_RAW_ADC, '3'},
        {AdcChannel::ADC_GATE_4, PARAM_GATE_4_RAW_ADC, '4'}
    }};

    Logger logger;
    static uint16_t gate_threshold;
    ModuleStatus error_flag = ModuleStatus::MODULE_OK;

};

#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_GATE_MONITOR_HPP_
