// Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
// Distributed under the terms of the GPL v3 license, available in the file LICENSE.

#include <string.h>
#include <chrono>  // NOLINT
#include <iostream>
#include "main.h"
#include "application.hpp"
#include "rom.h"

int main() {
    std::cout << "The app has been started." << std::endl;
    romInit(0, 1);
    application_entry_point();
    return 0;
}

void uavcanReadUniqueID(uint8_t out_uid[4]) {
    const uint32_t UNIQUE_ID_16_BYTES[4] = {0, 0, 0, 0};
    memset(out_uid, 0x00, 16);
}

void uavcanRestartNode() {
}

uint32_t uavcanGetTimeMs() {
    return HAL_GetTick();
}

void HAL_NVIC_SystemReset() {
    std::cout << "The app has been receved Restart command. Ignore." << std::endl;
}

uint32_t HAL_GetTick() {
    static auto start_time = std::chrono::high_resolution_clock::now();
    auto crnt_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(crnt_time - start_time).count();
}
