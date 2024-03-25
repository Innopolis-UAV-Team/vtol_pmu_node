// Copyright (C) 2024 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_MODULES_GATE_MONITOR_HPP_
#define SRC_MODULES_GATE_MONITOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

class GateMonitor {
public:
    GateMonitor();
    int8_t init();
    void process();
};

#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_GATE_MONITOR_HPP_