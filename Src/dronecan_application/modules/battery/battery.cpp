// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "battery.hpp"
#include <math.h>
#include <algorithm>
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "storage.h"
#include "periphery/adc/adc.hpp"


int8_t VtolBattery::init() {
    _battery_info.current = 0.0;
    _battery_info.voltage = 0.0;
    _battery_info.temperature = 0.0;
    _battery_info.average_power_10sec = 0;
    _battery_info.hours_to_full_charge = 0;
    _battery_info.state_of_health_pct = 127;
    _battery_info.state_of_charge_pct = 0;
    _battery_info.state_of_charge_pct_stdev = 0;

    return 0;
}

void VtolBattery::process() {
    uint32_t crnt_time_ms = HAL_GetTick();
    if (crnt_time_ms < _last_spin_time_ms + 200) {
        return;
    }

    _last_spin_time_ms = HAL_GetTick();
    _update_params();
    _spin_once();
}

void VtolBattery::_update_params() {
    _battery_info.battery_id = paramsGetIntegerValue(PARAM_BATTERY_ID);
    _battery_info.model_instance_id = paramsGetIntegerValue(PARAM_BATTERY_MODEL_INSTANCE_ID);

    _params.full_voltage = 0.001f * paramsGetIntegerValue(PARAM_BATTERY_FULL_VOLTAGE_MV);
    _params.empty_voltage = 0.001f * paramsGetIntegerValue(PARAM_BATTERY_EMPTY_VOLTAGE_MV);
    _params.current_offset = 0.001f * paramsGetIntegerValue(PARAM_BATTERY_CURRENT_OFFSET_MA);
    _params.pmu_soc_pct = paramsGetIntegerValue(PARAM_BATTERY_SOC_PCT);

    _params.correct = _params.full_voltage > _params.empty_voltage;
    auto capacity_mah = (float)paramsGetIntegerValue(PARAM_BATTERY_CAPACITY_MAH);
    _battery_info.full_charge_capacity_wh = 1.0e-3f * _params.full_voltage * capacity_mah;
}

void VtolBattery::_spin_once() {
    _update_voltage_and_current();
    _update_temperature();
    _update_soc();
    _update_remaining();
    _update_gate_info();

    dronecan_equipment_battery_info_publish(&_battery_info, &_transfer_id);
    _transfer_id++;
}

void VtolBattery::_update_voltage_and_current() {
    constexpr float ADC_VOLTAGE_MULTIPLIER = (19.0 * 3.3 / 4096.0);
    auto raw_vin_adc = AdcPeriphery::get(AdcChannel::ADC_VIN);
    float voltage = raw_vin_adc * ADC_VOLTAGE_MULTIPLIER;
    _battery_info.voltage = voltage;

    constexpr float ADC_CURRENT_MULTIPLIER = 200.0 / 4095.0;
    auto raw_current_adc = AdcPeriphery::get(AdcChannel::ADC_CRNT);
    float current = _params.current_offset + raw_current_adc * ADC_CURRENT_MULTIPLIER;
    _battery_info.current = std::clamp(current, 0.0f, 200.0f);

    _battery_info.average_power_10sec = _battery_info.voltage * _battery_info.current;
}

void VtolBattery::_update_temperature() {
    // B57861-S 103-F40, 10 kilohm, 1%, NTCthermistor
    float raw = AdcPeriphery::get(AdcChannel::ADC_VERSION);
    float celcius = 0.00000267f*raw*raw + 0.02734039f*raw + 5.48039378f;
    _battery_info.temperature = celcius + 273.15f;
}

void VtolBattery::_update_soc() {
    uint8_t& soc_pct = _battery_info.state_of_charge_pct;

    if (_params.pmu_soc_pct >= 0) {
        soc_pct = _params.pmu_soc_pct;
    } else if (!_params.correct) {
        soc_pct = 0;
    } else {
        float full_voltage = _params.full_voltage;
        float empty_voltage = _params.empty_voltage;
        auto voltage_clamped = std::clamp(_battery_info.voltage, empty_voltage, full_voltage);
        soc_pct = 100.0f * (voltage_clamped - empty_voltage) / (full_voltage - empty_voltage);
    }
}

void VtolBattery::_update_remaining() {
    float remaining_capacity_wh;
    if (_params.correct) {
        float soc = _battery_info.state_of_charge_pct * 0.01;
        remaining_capacity_wh = soc * _battery_info.full_charge_capacity_wh;
    } else {
        remaining_capacity_wh = NAN;
    }
    _battery_info.remaining_capacity_wh = remaining_capacity_wh;
}

void VtolBattery::_update_gate_info() {
    auto gate2 = AdcPeriphery::get(AdcChannel::ADC_GATE_2);
    auto gate3 = AdcPeriphery::get(AdcChannel::ADC_GATE_3);
    auto gate4 = AdcPeriphery::get(AdcChannel::ADC_GATE_4);
    _battery_info.hours_to_full_charge = std::max({gate2, gate3, gate4});
}
