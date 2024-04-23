// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "application.hpp"
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "storage.h"
#include "periphery/led/led.hpp"
#include "periphery/adc/adc.hpp"
#include "modules/battery/battery.hpp"
#include "modules/buzzer/buzzer.hpp"
#include "modules/gate_monitor/gate_monitor.hpp"

static bool isCharacter(uint8_t byte) {
    return ((byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z'));
}

void application_entry_point() {
    paramsInit((ParamIndex_t)IntParamsIndexes::INTEGER_PARAMS_AMOUNT, NUM_OF_STR_PARAMS, -1, 1);
    paramsLoad();

    auto node_id = paramsGetIntegerValue(PARAM_UAVCAN_NODE_ID);
    auto node_name_idx = static_cast<ParamIndex_t>(IntParamsIndexes::INTEGER_PARAMS_AMOUNT);
    auto node_name = (const char*)paramsGetStringValue(node_name_idx);

    LedPeriphery::reset();
    AdcPeriphery::init();

    uavcanInitApplication(node_id);
    uavcanSetNodeName(isCharacter(node_name[0]) ? node_name : "arl.pmu");

    VtolBattery battery;
    battery.init();

    GateMonitor gate_monitor;
    gate_monitor.init("PMU");

    Buzzer buzzer;
    buzzer.init();

    while(true) {
        battery.process();
        ModuleStatus gate_monitor_status = gate_monitor.process();
        buzzer.process((uint8_t)gate_monitor_status);

        uavcanSetNodeHealth((NodeStatusHealth_t)gate_monitor_status);
        LedPeriphery::toggle();
        uavcanSpinOnce();
    }
}
