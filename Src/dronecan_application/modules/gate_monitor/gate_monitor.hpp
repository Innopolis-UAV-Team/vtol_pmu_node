// Copyright (C) 2024 Anastasiia Stepanova <asiiapine@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_MODULES_GATE_MONITOR_HPP_
#define SRC_MODULES_GATE_MONITOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "periphery/adc/adc.hpp"
#include "params.hpp"

#include "../../logger.hpp"

enum class ModuleStatus: uint8_t {
    MODULE_OK        = 0,
    MODULE_WARN      = 1,
    MODULE_CRITICAL  = 2,
    MODULE_ERROR     = 3
};

class GateMonitor {
public:
    ModuleStatus process();

    static const uint8_t n_gates = 3;
    uint8_t is_gate_broken[3];

    GateMonitor();
    void init(const char* logger_source = "GateMonitor");
    static const char gate_names[3];
    
private:
    static constexpr AdcChannel gate_channels[] = { 
                                        AdcChannel::ADC_GATE_2, 
                                        AdcChannel::ADC_GATE_3, 
                                        AdcChannel::ADC_GATE_4};
    
    void check_gates();
    void update_params();
    void spin_once();

    Logger logger;
    static uint16_t gate_threshold;
    ModuleStatus error_flag = ModuleStatus::MODULE_OK;

};

#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_GATE_MONITOR_HPP_
