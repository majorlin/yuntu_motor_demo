function(compilerVersion)
  execute_process(COMMAND "${CMAKE_C_COMPILER}" --version -c t.c
     OUTPUT_VARIABLE CVERSION
     ERROR_VARIABLE CVERSION
    )
  SET(COMPILERVERSION ${CVERSION} PARENT_SCOPE)
  #cmake_print_variables(CVERSION)
  #cmake_print_variables(CMAKE_C_COMPILER)
  #MESSAGE( STATUS "CMD_OUTPUT:" ${CVERSION})
endfunction()

function(compilerSpecificCompileOptions PROJECTNAME ROOT)
  if (LITTLEENDIAN)
    target_compile_options(${PROJECTNAME} PUBLIC "-littleendian")
  endif()

  if (CORTEXM OR CORTEXR)
    target_compile_options(${PROJECTNAME} PUBLIC "-thumb")
  endif()
  
  # Core specific config
  if (ARM_CPU STREQUAL "cortexm33" )
    if(DEVICE_NAME STREQUAL "YTM32B1MC0")
      target_compile_options(${PROJECTNAME} PUBLIC "-fsoft" "-disable_cortexm_dsp")  
    else()
      target_compile_options(${PROJECTNAME} PUBLIC "-fhard" "-fpu=vfpv5_d16" "-fsingle")
    endif()
  endif()

  if (ARM_CPU STREQUAL "cortexm7" )
    target_compile_options(${PROJECTNAME} PUBLIC "-fhard" "-fpu=vfpv5_d16" "-fsingle")
  endif()

  if (ARM_CPU STREQUAL "cortexm4" )
    target_compile_options(${PROJECTNAME} PUBLIC  "-fhard" "-mfpu=fpv4_d16" "-fsingle")
  endif()

  if (ARM_CPU STREQUAL "cortexm0plus" )
    target_compile_options(${PROJECTNAME} PUBLIC "-fsoft")
  endif()
  
  target_compile_options(${PROJECTNAME} PRIVATE "-preprocess_assembly_files" "--gnu_asm" "-G" "-dual_debug")

  target_compile_options(${PROJECTNAME} PUBLIC 
    "--diag_suppress=96"
    "$<$<COMPILE_LANGUAGE:C>:-O1>"
    "$<$<COMPILE_LANGUAGE:CXX>:-O1>"
  )

  if(EXPERIMENTAL)
    experimentalCompilerSpecificCompileOptions(${PROJECTNAME} ${ROOT})
  endif()
endfunction()


function(toolchainSpecificLinkForCortexM PROJECTNAME ROOT CORE PLATFORMFOLDER HASCSTARTUP)
endfunction()

function(toolchainSpecificLinkForCortexA PROJECTNAME ROOT CORE PLATFORMFOLDER)
endfunction()

function(toolchainSpecificLinkForCortexR PROJECTNAME ROOT CORE PLATFORMFOLDER)
endfunction()

function(compilerSpecificPlatformConfigLibForM PROJECTNAME ROOT)
endfunction()

function(compilerSpecificPlatformConfigLibForA PROJECTNAME ROOT)
endfunction()

function(compilerSpecificPlatformConfigLibForR PROJECTNAME ROOT)
endfunction()

function(compilerSpecificPlatformConfigAppForM PROJECTNAME ROOT)
  target_link_libraries(${PROJECTNAME} 
  "-nostartfiles"
  )
  
endfunction()

function(compilerSpecificPlatformConfigAppForA PROJECTNAME ROOT)
  
endfunction()

function(compilerSpecificPlatformConfigAppForR PROJECTNAME ROOT)
 
endfunction()