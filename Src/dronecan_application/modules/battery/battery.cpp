// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "battery.hpp"
#include <algorithm>
#include <math.h>
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "storage.h"
#include "periphery/adc/adc.hpp"

#define ADC_VOLTAGE_MULTIPLIER (19.0 * 3.3 / 4096.0) // 62.7  Volts when ADC is 3.3V (4095)
#define ADC_CURRENT_MULTIPLIER 0.04884004884         // 200.0 Amper when ADC is 3.3V (4095)

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

    dronecan_equipment_battery_info_publish(&_battery_info, &_transfer_id);
    _transfer_id++;
}

void VtolBattery::_update_voltage_and_current() {
    float voltage = AdcPeriphery::get(AdcChannel::ADC_VIN) * ADC_VOLTAGE_MULTIPLIER;
    _battery_info.voltage = voltage;

    float current = AdcPeriphery::get(AdcChannel::ADC_CRNT) * ADC_CURRENT_MULTIPLIER;
    _battery_info.current = current;

    _battery_info.average_power_10sec = voltage * current;
}

void VtolBattery::_update_temperature() {
    uint16_t adc_temperature = AdcPeriphery::get(AdcChannel::ADC_TEMPERATURE);
    static const uint16_t TEMP_REF = 25;
    static const uint16_t ADC_REF = 1750;   ///< v_ref / 3.3 * 4095
    static const uint16_t AVG_SLOPE = 5;    ///< avg_slope/(3.3/4096)
    float kelvin = (ADC_REF - adc_temperature) / AVG_SLOPE + TEMP_REF + 273.15;
    _battery_info.temperature = kelvin;
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
