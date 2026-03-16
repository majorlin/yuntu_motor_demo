cmake_minimum_required(VERSION 3.16)



set(sources
    ${PROJ_DIR}/board/clock_config.c
    ${PROJ_DIR}/board/pin_mux.c
    ${PROJ_DIR}/board/startup.S
    ${PROJ_DIR}/board/vector.S
    ${PROJ_DIR}/board/vector_table_copy.c
    ${PROJ_DIR}/board/RamInit0.S
    ${PROJ_DIR}/board/RamInit1.c
    ${PROJ_DIR}/board/RamInit2.c
    ${PROJ_DIR}/board/interrupt_config.c
    ${PROJ_DIR}/board/dma_config.c
    ${PROJ_DIR}/board/spi_config.c
    ${PROJ_DIR}/board/linflexd_uart_config.c
    ${PROJ_DIR}/board/can_config.c
    ${PROJ_DIR}/board/i2c_config.c
    ${PROJ_DIR}/board/tmu_config.c
    ${PROJ_DIR}/board/ptmr_config.c
    ${PROJ_DIR}/board/lptmr_config.c
    ${PROJ_DIR}/board/etmr_config.c
    ${PROJ_DIR}/board/adc_config.c
    ${PROJ_DIR}/board/acmp_config.c
    ${PROJ_DIR}/board/wdg_config.c
    ${PROJ_DIR}/board/flash_config.c
)
set(includes
    ${PROJ_DIR}/board
)
set(priIncludes
)

add_library(GENERATED_CONFIG_TARGET STATIC ${sources})

target_include_directories(GENERATED_CONFIG_TARGET PUBLIC ${includes})


target_include_directories(GENERATED_CONFIG_TARGET PRIVATE ${priIncludes})
configcore(GENERATED_CONFIG_TARGET ${CMAKE_SOURCE_DIR})

target_compile_definitions(GENERATED_CONFIG_TARGET PUBLIC
    YTM32B1ME0
    CPU_YTM32B1ME0
)
target_compile_options(GENERATED_CONFIG_TARGET PUBLIC
    -fdiagnostics-color=always
)



target_link_libraries(GENERATED_CONFIG_TARGET
    GENERATED_SDK_TARGET
)
