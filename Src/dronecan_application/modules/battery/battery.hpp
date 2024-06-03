// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#ifndef SRC_MODULES_BATTERY_HPP_
#define SRC_MODULES_BATTERY_HPP_

#include <stdint.h>
#include "uavcan/equipment/power/BatteryInfo.h"

#ifdef __cplusplus
extern "C" {
#endif

struct BatteryParameters {
    float full_voltage;
    float empty_voltage;
    float voltage_range;
    float current_offset;
    float resistance{0.032};
    uint32_t max_current;
    int8_t pmu_soc_pct;
    bool enable_thermistor_up;
    bool enable_thermistor_down;
    bool correct{false};
};

class VtolBattery {
public:
    VtolBattery() = default;
    int8_t init();
    void process();
private:
    void _update_params();
    void _spin_once();
    void _update_voltage_and_current();
    void _update_temperature();
    uint8_t _update_soc(float voltage, float current) const;
    void _update_remaining();
    void _update_gate_info();

    BatteryParameters _params;
    uint32_t _last_spin_time_ms{0};
    BatteryInfo_t _battery_info{};
    uint8_t _transfer_id = 0;
};


#ifdef __cplusplus
}
#endif

#endif  // SRC_MODULES_BATTERY_HPP_
