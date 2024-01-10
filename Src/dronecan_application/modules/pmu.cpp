// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "pmu.hpp"
#include <math.h>
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "string_params.hpp"
#include "string_params.hpp"
#include "storage.h"
#include "periphery/adc/adc.hpp"


VtolPmu::VtolPmu() {

}

int8_t VtolPmu::init() {
    _battery_info.current = NAN;
    _battery_info.voltage = NAN;
    _battery_info.temperature = NAN;
    _battery_info.average_power_10sec = 0;
    _battery_info.remaining_capacity_wh = NAN;
    _battery_info.full_charge_capacity_wh = NAN;
    _battery_info.hours_to_full_charge = NAN;
    _battery_info.state_of_health_pct = 0;
    _battery_info.state_of_charge_pct_stdev = 0;

    return 0;
}

void VtolPmu::process() {
    uint32_t crnt_time_ms = HAL_GetTick();
    if (crnt_time_ms < _last_spin_time_ms + 20) {
        return;
    }

    _spin_once();
}

void VtolPmu::_spin_once() {
    _last_spin_time_ms = HAL_GetTick();

    float voltage = AdcPeriphery::get(AdcChannel::ADC_VIN) * 19.0 * 3.3 / 4096.0;
    _battery_info.voltage = voltage;

    uint16_t adc_temperature = AdcPeriphery::get(AdcChannel::ADC_TEMPERATURE);
    static const uint16_t TEMP_REF = 25;
    static const uint16_t ADC_REF = 1750;   ///< v_ref / 3.3 * 4095
    static const uint16_t AVG_SLOPE = 5;    ///< avg_slope/(3.3/4096)
    float kelvin = (ADC_REF - adc_temperature) / AVG_SLOPE + TEMP_REF + 273.15;
    _battery_info.temperature = kelvin;

    float current = AdcPeriphery::get(AdcChannel::ADC_CRNT) * 600.0 * (3.3 / 3.0 / 4096.0);
    _battery_info.current = current;

    dronecan_equipment_battery_info_publish(&_battery_info, &_transfer_id);
    _transfer_id++;
}
