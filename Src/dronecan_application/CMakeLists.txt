# Copyright (C) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

set(APPLICATION_DIR ${CMAKE_CURRENT_LIST_DIR})
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH SRC_DIR)
cmake_path(GET SRC_DIR PARENT_PATH ROOT_DIR)

add_definitions(-DAPP_VERSION_MAJOR=0)
add_definitions(-DAPP_VERSION_MINOR=8)
add_definitions(-DHW_VERSION_MAJOR=2)
add_definitions(-DHW_VERSION_MINOR=1)
add_definitions(-DFLASH_DRIVER_STORAGE_FILE="${CMAKE_CURRENT_LIST_DIR}/default_params.yml")

include(${ROOT_DIR}/Libs/Dronecan/CMakeLists.txt)

set(applicationSourceCode
    ${libparamsSrc}
    ${DRONECAN_SOURCES}
    ${APPLICATION_DIR}/application.cpp
    ${APPLICATION_DIR}/logger.cpp
    ${APPLICATION_DIR}/modules/battery/battery.cpp
    ${APPLICATION_DIR}/modules/buzzer/buzzer.cpp
    ${APPLICATION_DIR}/modules/gate_monitor/gate_monitor.cpp
)
set(applicationHeaders
    ${libparamsHeaders}
    ${DRONECAN_HEADERS}
    ${ROOT_DIR}/Src
    ${APPLICATION_DIR}
)

set(LIBPARAMS_PARAMS
    ${APPLICATION_DIR}/params.yaml
    ${APPLICATION_DIR}/modules/battery/params.yaml
    ${APPLICATION_DIR}/modules/buzzer/params.yaml
    ${APPLICATION_DIR}/modules/gate_monitor/params.yaml
)
