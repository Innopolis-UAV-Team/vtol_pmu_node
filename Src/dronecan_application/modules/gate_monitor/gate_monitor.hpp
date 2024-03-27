// Copyright (C) 2024 Dmitry Ponomarev <ponomarevda96@gmail.com>
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

enum GateStatus : bool {
    GateStatusOK      = 0,
    GateStatusERROR   = 1,
};

class GateMonitor {
public:
    // static GateMonitor &get_instance();
    void process();

    static GateStatus gate_2_status;
    static GateStatus gate_3_status;
    static GateStatus gate_4_status;

// protected:
    GateMonitor();
    int8_t init();
    
private:
    int8_t check_gate(AdcChannel channel);
    static Logger logger;
    void update_params();

    static uint8_t gate_threshold;

    // GateMonitor& operator = (const GateMonitor&) = delete;
    // GateMonitor(GateMonitor &other) = delete;
};

#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_GATE_MONITOR_HPP_
