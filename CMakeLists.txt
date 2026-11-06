cmake_minimum_required(VERSION 3.20)
project(LeapPulse LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(leappulse_core core/leappulse.c core/workout.c)
target_include_directories(leappulse_core PUBLIC core)
if(NOT MSVC)
    target_link_libraries(leappulse_core PUBLIC m)
    target_compile_options(leappulse_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_executable(test_core tests/test_core.c)
target_link_libraries(test_core PRIVATE leappulse_core)
add_test(NAME sensor_algorithms COMMAND test_core)

add_executable(test_workout tests/test_workout.c)
target_link_libraries(test_workout PRIVATE leappulse_core)
add_test(NAME workout_modes COMMAND test_workout)

add_executable(test_wireless tests/test_wireless.c firmware/src/wireless.c)
target_include_directories(test_wireless PRIVATE firmware/src)
add_test(NAME wireless_and_ota_protocol COMMAND test_wireless)
