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
    ModuleOK        = 0,
    ModuleWARN      = 1,
    ModuleCRITICAL  = 2,
    ModuleERROR     = 3
};

enum GateStatus : bool {
    GateStatusOK      = 0,
    GateStatusERROR   = 1,
};

class GateMonitor {
public:
    // static GateMonitor &get_instance();
    ModuleStatus process();

    static uint16_t number_of_broken_gates;

    static GateStatus gate_2_status;
    static GateStatus gate_3_status;
    static GateStatus gate_4_status;

// protected:
    GateMonitor();
    int8_t init();
    
private:
    int8_t check_gate(AdcChannel channel);
    Logger logger;
    void update_params();
    void spin_once();

    uint32_t _last_spin_time_ms;
    static uint16_t gate_threshold;
    ModuleStatus error_flag = ModuleStatus::ModuleOK;

    // GateMonitor& operator = (const GateMonitor&) = delete;
    // GateMonitor(GateMonitor &other) = delete;
};

#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_GATE_MONITOR_HPP_
