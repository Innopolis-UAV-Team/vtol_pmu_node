// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_MODULES_PMU_HPP_
#define SRC_MODULES_PMU_HPP_

#include <stdint.h>
#include "uavcan/equipment/power/BatteryInfo.h"

#ifdef __cplusplus
extern "C" {
#endif


class VtolPmu {
public:
    VtolPmu();
    int8_t init();
    void process();
private:
    void _spin_once();
    uint32_t _last_spin_time_ms{0};
    BatteryInfo_t _battery_info{};
    uint8_t _transfer_id = 0;
};


#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_PMU_HPP_
