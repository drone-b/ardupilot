cmake_minimum_required(VERSION 3.20)
project(DroneFirmware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(drone_firmware
    src/app/main.cpp
    src/comms/AiControlLink.cpp
    src/comms/AiStatePublisher.cpp
    src/comms/TelemetryPublisher.cpp
    src/config/Parameters.cpp
    src/logging/SdLogStorage.cpp
    src/runtime/Scheduler.cpp
    src/estimation/AttitudeEstimator.cpp
    src/control/AttitudeController.cpp
    src/control/FlightModeManager.cpp
    src/control/InputMapper.cpp
    src/control/RateController.cpp
    src/control/ControlAllocator.cpp
    src/control/Mixer.cpp
    src/control/ControlLoop.cpp
    src/safety/SafetySupervisor.cpp
    src/sensing/SensorManager.cpp
)

target_include_directories(drone_firmware PRIVATE src)
