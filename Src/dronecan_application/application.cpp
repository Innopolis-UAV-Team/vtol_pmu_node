// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include "application.hpp"
#include "dronecan.h"
#include "main.h"
#include "params.hpp"
#include "string_params.hpp"
#include "storage.h"
#include "periphery/led/led.hpp"
#include "modules/pmu.hpp"

void application_entry_point() {
    paramsInit(static_cast<uint8_t>(IntParamsIndexes::INTEGER_PARAMS_AMOUNT), NUM_OF_STR_PARAMS);
    paramsLoadFromFlash();

    auto node_id = paramsGetIntegerValue(UAVCAN_NODE_ID);
    auto node_name = (const char*)paramsGetStringValue(static_cast<uint8_t>(IntParamsIndexes::INTEGER_PARAMS_AMOUNT));

    LedPeriphery::reset();
    uavcanInitApplication(node_id);
    uavcanSetNodeName(node_name);

    VtolPmu pmu;
    pmu.init();

    while(true) {
        LedPeriphery::toggle();
        uavcanSpinOnce();
        pmu.process();
    }
}
