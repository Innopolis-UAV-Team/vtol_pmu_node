// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "battery.hpp"
#include <math.h>
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "storage.h"
#include "periphery/adc/adc.hpp"
#include "uavcan/protocol/debug/LogMessage.h"

#define ADC_VOLTAGE_MULTIPLIER (19.0 * 3.3 / 4096.0) // 62.7  Volts when ADC is 3.3V (4095)
#define ADC_CURRENT_MULTIPLIER 0.04884004884         // 200.0 Amper when ADC is 3.3V (4095)

VtolBattery::VtolBattery() {

}

int8_t VtolBattery::init() {
    _battery_info.current = 0.0;
    _battery_info.voltage = 0.0;
    _battery_info.temperature = 0.0;
    _battery_info.average_power_10sec = 0;
    _battery_info.remaining_capacity_wh = NAN;
    _battery_info.full_charge_capacity_wh = NAN;
    _battery_info.hours_to_full_charge = 0;
    _battery_info.state_of_health_pct = 127;
    _battery_info.state_of_charge_pct = 0;
    _battery_info.state_of_charge_pct_stdev = 0;

    DebugLogMessage_t msg{};
    msg.source_size = 3;
    msg.source[0] = 'p';
    msg.source[1] = 'm';
    msg.source[2] = 'u';
    msg.text[0] = 'i';
    msg.text[1] = 'n';
    msg.text[2] = 'i';
    msg.text[3] = 't';
    msg.text_size = 4;
    uint8_t transfer_id = 0;
    dronecan_protocol_debug_log_message_publish(&msg, &transfer_id);

    return 0;
}

void VtolBattery::process() {
    uint32_t crnt_time_ms = HAL_GetTick();
    if (crnt_time_ms < _last_spin_time_ms + 200) {
        return;
    }

    _spin_once();
}

void VtolBattery::_spin_once() {
    _last_spin_time_ms = HAL_GetTick();

    int8_t pmu_soc_pct = paramsGetIntegerValue(PARAM_BATTERY_SOC_PCT);
    if (pmu_soc_pct >= 0) {
        _battery_info.state_of_charge_pct = pmu_soc_pct;
    }

    float voltage = AdcPeriphery::get(AdcChannel::ADC_VIN) * ADC_VOLTAGE_MULTIPLIER;
    _battery_info.voltage = voltage;

    uint16_t adc_temperature = AdcPeriphery::get(AdcChannel::ADC_TEMPERATURE);
    static const uint16_t TEMP_REF = 25;
    static const uint16_t ADC_REF = 1750;   ///< v_ref / 3.3 * 4095
    static const uint16_t AVG_SLOPE = 5;    ///< avg_slope/(3.3/4096)
    float kelvin = (ADC_REF - adc_temperature) / AVG_SLOPE + TEMP_REF + 273.15;
    _battery_info.temperature = kelvin;

    float current = AdcPeriphery::get(AdcChannel::ADC_CRNT) * ADC_CURRENT_MULTIPLIER;
    _battery_info.current = current;

    dronecan_equipment_battery_info_publish(&_battery_info, &_transfer_id);
    _transfer_id++;
}
