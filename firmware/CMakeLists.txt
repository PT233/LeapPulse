# Add this directory from an STM32CubeF4 CMake project after defining its app target.
set(LP_BMI270_DIR "" CACHE PATH "Pinned Bosch BMI270 SensorAPI checkout")
set(LP_LVGL_DIR "" CACHE PATH "LVGL v9.6.0 checkout")
if(NOT EXISTS "${LP_BMI270_DIR}/bmi270.c" OR NOT EXISTS "${LP_LVGL_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Set LP_BMI270_DIR and LP_LVGL_DIR (see docs/stm32f407.md)")
endif()

set(LV_BUILD_CONF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h" CACHE PATH "" FORCE)
set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_THORVG OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_THORVG_INTERNAL OFF CACHE BOOL "" FORCE)
add_subdirectory("${LP_LVGL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/lvgl" EXCLUDE_FROM_ALL)

function(leappulse_attach_app target)
    set(dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
    target_sources(${target} PRIVATE
        "${dir}/../core/leappulse.c"
        "${dir}/../core/workout.c"
        "${dir}/src/app.c"
        "${dir}/src/peripherals.c"
        "${dir}/src/sensors.c"
        "${dir}/src/wireless.c"
        "${dir}/stm32f407/flash_slots.c"
        "${dir}/stm32f407/stm32_port.c"
        "${dir}/stm32f407/rtc_stop.c"
        "${LP_BMI270_DIR}/bmi2.c"
        "${LP_BMI270_DIR}/bmi270.c"
    )
    target_include_directories(${target} PRIVATE
        "${dir}/../core" "${dir}/src" "${dir}/stm32f407" "${LP_BMI270_DIR}"
    )
    target_compile_features(${target} PRIVATE c_std_11)
    target_link_libraries(${target} PRIVATE lvgl::lvgl m)
endfunction()
