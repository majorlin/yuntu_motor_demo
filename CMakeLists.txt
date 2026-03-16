cmake_minimum_required(VERSION 3.16)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# cmake -DCMAKE_PREFIX_PATH="." -DCMAKE_TOOLCHAIN_FILE="../cmake/gcc.cmake" -DARM_CPU="cortex-m33" -G "Ninja" ..


project(yuntu_motor_demo)

# USER CODE BEGIN start

# USER CODE END start

set(PROJ_DIR ${CMAKE_SOURCE_DIR})
set(SDK_VER dev)
set(SDK_DIR C:/work/1_Yuntu/yt_sdk/YTM32B1ME0)

include(${CMAKE_SOURCE_DIR}/cmake/config.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/targets/GENERATED_CONFIG_TARGET.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/targets/GENERATED_SDK_TARGET.cmake)

# USER CODE BEGIN include
# include(...)
# USER CODE END include

set(project_elf yuntu_motor_demo.elf)
add_executable(${project_elf} app/main.c)
#add app as include path
target_include_directories(${project_elf} PRIVATE app)
#add all source files in app folder 
file(GLOB dir_sources "app/*.c" "app/*.cpp" "app/*.S")
if(dir_sources)
    foreach(src ${dir_sources})
        target_sources(${project_elf} PRIVATE ${src})
    endforeach()
endif()

# USER CODE BEGIN add_executable
# target_include_directories()
# target_sources(${project_elf} PRIVATE ..)
# USER CODE END add_executable

configcore(${project_elf} ${CMAKE_SOURCE_DIR})


# USER CODE BEGIN target_compile_definitions
# target_compile_definitions(...)
# USER CODE END target_compile_definitions

target_compile_definitions(${project_elf} PUBLIC
    "YTM32B1ME0"
    "CPU_YTM32B1ME0"
)

# USER CODE BEGIN target_compile_options
# target_compile_options(...)
# USER CODE END target_compile_options


target_link_libraries(${project_elf} "-Wl,--whole-archive" GENERATED_CONFIG_TARGET GENERATED_SDK_TARGET  "-Wl,--no-whole-archive")

# USER CODE BEGIN target_link_libraries
# target_link_libraries(...)
# USER CODE END target_link_libraries


if (CORTEXM)
    compilerSpecificPlatformConfigAppForM(${project_elf} ${CMAKE_SOURCE_DIR} )
elseif(CORTEXA)
    compilerSpecificPlatformConfigAppForA(${project_elf} ${CMAKE_SOURCE_DIR} )
else()
    compilerSpecificPlatformConfigAppForR(${project_elf} ${CMAKE_SOURCE_DIR} )
endif()

target_link_libraries(${project_elf} "--entry=Reset_Handler;-T${PROJ_DIR}/board/yt_linker.ld")
target_link_libraries(${project_elf} "-Wl,-Map=yuntu_motor_demo.map;-Wl,--end-group")
add_custom_command(TARGET ${project_elf} POST_BUILD COMMAND arm-none-eabi-size --format=berkeley ${project_elf})
add_custom_target(genbin
    DEPENDS ${project_elf}
    COMMAND ${CMAKE_OBJCOPY} -F elf32-littlearm -O binary ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
)
add_custom_target(genhex
    DEPENDS ${project_elf}
    COMMAND ${CMAKE_OBJCOPY} -F elf32-littlearm -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
)
add_custom_command(
    TARGET ${project_elf} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -F elf32-littlearm -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
)
add_custom_target(gens19
    DEPENDS ${project_elf}
    COMMAND ${CMAKE_OBJCOPY} -F elf32-littlearm -O srec ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.s19
)
add_custom_target(genlist
    DEPENDS ${project_elf}
    COMMAND ${CMAKE_OBJDUMP}  --source --all-headers --demangle --line-numbers --wide ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf > ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.lst
)

# ozone target 
if (CMAKE_HOST_WIN32)
    set(OZONE_PATHS 
        "$ENV{PROGRAMFILES}/SEGGER/Ozone"
        "$ENV{PROGRAMFILES\(X86\)}/SEGGER/Ozone"
    )
    if(EXISTS "D:/")
        # 如果D盘存在,将其添加到搜索路径中
        list(APPEND OZONE_PATHS 
            "D:/Program Files/SEGGER/Ozone"
            "D:/Program Files (x86)/SEGGER/Ozone"
        )
    endif()
    list(APPEND OZONE_PATHS ".")
endif()
find_program(OZONE_EXECUTABLE 
    NAMES Ozone
    PATHS ${OZONE_PATHS}
)
if(OZONE_EXECUTABLE)
add_custom_target(Ozone
    DEPENDS ${project_elf} 
    COMMAND ${CMAKE_SOURCE_DIR}/.scripts/ozone ${OZONE_EXECUTABLE} ${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}.jdebug
)
else()
message(WARNING "Ozone not found, additional search paths: ${OZONE_PATHS}")
endif()


# yct file as depend
file(GLOB yct_files "${CMAKE_SOURCE_DIR}/*.yct")
if(yct_files)
    list(APPEND CMAKE_CONFIGURE_DEPENDS ${yct_files})
endif()

# USER CODE BEGIN others

# USER CODE END others