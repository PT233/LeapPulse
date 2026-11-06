# Include in a separate CubeF4 bootloader target, replacing the generated main.c.
function(leappulse_attach_boot target)
    set(dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
    target_sources(${target} PRIVATE "${dir}/boot_main.c" "${dir}/flash_slots.c")
    target_include_directories(${target} PRIVATE "${dir}")
    target_compile_features(${target} PRIVATE c_std_11)
endfunction()
