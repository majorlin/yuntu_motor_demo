# Setting Linux is forcing th extension to be .o instead of .obj when building on WIndows.
# It is important because armlink is failing when files have .obj extensions (error with
# scatter file section not found)
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)



find_program(CMAKE_C_COMPILER NAMES ccarm ccarm)
find_program(CMAKE_CXX_COMPILER NAMES cxarm cxarm)
find_program(CMAKE_ASM_COMPILER NAMES ccarm ccarm)


if (NOT ("${tools}" STREQUAL ""))
message(STATUS "Tools path is set")
SET(CMAKE_AR "${tools}/ccarm")
SET(CMAKE_CXX_COMPILER_AR "${tools}/cxarm")
SET(CMAKE_C_COMPILER_AR "${tools}/ccarm")
else()
find_program(CMAKE_AR NAMES ccarm ccarm )
find_program(CMAKE_CXX_COMPILER_AR NAMES cxarm cxarm)
find_program(CMAKE_C_COMPILER_AR NAMES ccarm ccarm)
endif()

find_program(CMAKE_LINKER NAMES cxarm cxarm)

SET(CMAKE_C_LINK_EXECUTABLE "<CMAKE_LINKER> <LINK_FLAGS> -o <TARGET> <OBJECTS> <LINK_LIBRARIES>")
SET(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_LINKER> <LINK_FLAGS> -o <TARGET> <OBJECTS> <LINK_LIBRARIES>")
SET(CMAKE_C_OUTPUT_EXTENSION .o)
SET(CMAKE_CXX_OUTPUT_EXTENSION .o)
SET(CMAKE_ASM_OUTPUT_EXTENSION .o)
# When library defined as STATIC, this line is needed to describe how the .a file must be
# create. Some changes to the line may be needed.
SET(CMAKE_C_CREATE_STATIC_LIBRARY "<CMAKE_AR> <LINK_FLAGS> <OBJECTS> -archive -o <TARGET> ")
SET(CMAKE_CXX_CREATE_STATIC_LIBRARY "<CMAKE_AR> <LINK_FLAGS> <OBJECTS> -archive -o <TARGET> ")

set(GHS ON)
# default core

if(NOT ARM_CPU)
    set(
            ARM_CPU "cortex-a5"
            CACHE STRING "Set ARM CPU. Default : cortex-a5"
    )
endif(NOT ARM_CPU)

#replcae - in ARM_CPU
string(REPLACE "-" "" ARM_CPU ${ARM_CPU})


SET(CMAKE_C_FLAGS "-cpu=${ARM_CPU}" CACHE INTERNAL "C compiler common flags")
SET(CMAKE_CXX_FLAGS "-cpu=${ARM_CPU}" CACHE INTERNAL "C compiler common flags")
SET(CMAKE_ASM_FLAGS "-cpu=${ARM_CPU}" CACHE INTERNAL "ASM compiler common flags")
SET(CMAKE_EXE_LINKER_FLAGS "-cpu=${ARM_CPU}"  CACHE INTERNAL "linker flags")

# Where is the target environment
#SET(CMAKE_FIND_ROOT_PATH "${tools}")
# Search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# For libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

