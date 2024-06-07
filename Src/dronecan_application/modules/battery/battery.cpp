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
    _battery_info.state_of_health_pct = 0;
    _battery_info.state_of_charge_pct = 0;
    _battery_info.state_of_charge_pct_stdev = 0;

#ifdef THERMISTOR_DOWN_GPIO_Port
    HAL_GPIO_WritePin(THERMISTOR_DOWN_GPIO_Port, THERMISTOR_DOWN_Pin, GPIO_PIN_RESET);
#endif

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
    _params.voltage_range = _params.full_voltage - _params.empty_voltage;
    _params.current_offset = 0.001f * paramsGetIntegerValue(PARAM_BATTERY_CURRENT_OFFSET_MA);
    _params.max_current = paramsGetIntegerValue(PARAM_BATTERY_MAX_CURRENT);

    _params.enable_thermistor_up = paramsGetIntegerValue(PARAM_BATTERY_ENABLE_THERMISTOR_UP);
    _params.enable_thermistor_down = paramsGetIntegerValue(PARAM_BATTERY_ENABLE_THERMISTOR_DOWN);

    _params.resistance = 0.001 * paramsGetIntegerValue(PARAM_BATTERY_RESISTOR_MILLI_OHM);

    _params.pmu_soc_pct = paramsGetIntegerValue(PARAM_BATTERY_SOC_PCT);

    _params.correct = _params.full_voltage > _params.empty_voltage;
    auto capacity_mah = (float)paramsGetIntegerValue(PARAM_BATTERY_CAPACITY_MAH);
    _battery_info.full_charge_capacity_wh = 1.0e-3f * _params.full_voltage * capacity_mah;
}

void VtolBattery::_spin_once() {
    _update_voltage_and_current();
    _update_temperature();
    _battery_info.state_of_charge_pct = _update_soc(_battery_info.voltage, _battery_info.current);
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

    float ADC_CURRENT_MULTIPLIER = _params.max_current / 4095.0;
    auto raw_current_adc = AdcPeriphery::get(AdcChannel::ADC_CRNT);
    float current = _params.current_offset + raw_current_adc * ADC_CURRENT_MULTIPLIER;
    _battery_info.current = std::clamp(current, 0.0f, 200.0f);

    _battery_info.average_power_10sec = _battery_info.voltage * _battery_info.current;
}

void VtolBattery::_update_temperature() {
    float raw;
    float celcius;

    // B57861-S 103-F40, 10 kilohm, 1%, NTCthermistor
    if (_params.enable_thermistor_up) {
        raw = AdcPeriphery::get(AdcChannel::ADC_THERMISTOR_UP);
        celcius = 0.00000267f*raw*raw + 0.02734039f*raw + 5.48039378f;
        _battery_info.temperature = celcius;
    } else {
        _battery_info.temperature = 0.0;
    }

    // B57861-S 103-F40, 10 kilohm, 1%, NTCthermistor
    if (_params.enable_thermistor_down) {
        raw = AdcPeriphery::get(AdcChannel::ADC_THERMISTOR_DOWN);
        celcius = 0.00000267f*raw*raw + 0.02734039f*raw + 5.48039378f;
        _battery_info.state_of_health_pct = celcius;
    } else {
        _battery_info.state_of_health_pct = 0;
    }

    // STM32 temperature
    raw = AdcPeriphery::get(AdcChannel::ADC_TEMPERATURE);
    static const uint16_t TEMP_REF = 25;
    static const uint16_t ADC_REF = 1750;   ///< v_ref / 3.3 * 4095
    static const uint16_t AVG_SLOPE = 5;    ///< avg_slope/(3.3/4096)
    celcius = (ADC_REF - raw) / AVG_SLOPE + TEMP_REF;
    _battery_info.state_of_charge_pct_stdev = celcius;
}

uint8_t VtolBattery::_update_soc(float voltage, float current) const {
    uint8_t soc_pct;

    if (_params.pmu_soc_pct >= 0) {
        soc_pct = _params.pmu_soc_pct;
    } else if (!_params.correct) {
        soc_pct = 0;
    } else {
        float v_adjasted = voltage + current * _params.resistance;
        float v_clamped = std::clamp(v_adjasted, _params.empty_voltage, _params.full_voltage);
        soc_pct = 100.0f * (v_clamped - _params.empty_voltage) / _params.voltage_range;
    }

    return soc_pct;
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
