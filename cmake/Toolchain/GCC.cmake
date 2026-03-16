function(compilerVersion)
  execute_process(COMMAND "${CMAKE_C_COMPILER}" -dumpversion
     OUTPUT_VARIABLE CVERSION
     ERROR_VARIABLE CVERSION
    )
  SET(COMPILERVERSION ${CVERSION} PARENT_SCOPE)
  #cmake_print_variables(CVERSION)
  #cmake_print_variables(CMAKE_C_COMPILER)
  #MESSAGE( STATUS "CMD_OUTPUT:" ${CVERSION})
endfunction()

function(compilerSpecificCompileOptions PROJECTNAME ROOT)
  get_target_property(DISABLEOPTIM ${PROJECTNAME} DISABLEOPTIMIZATION)

  if (LITTLEENDIAN)
    target_compile_options(${PROJECTNAME} PUBLIC "-mlittle-endian")
  endif()

  if (CORTEXM OR CORTEXR)
    target_compile_options(${PROJECTNAME} PUBLIC "-mthumb")
  endif()

  target_link_options(${PROJECTNAME} PUBLIC "-mcpu=${ARM_CPU}")


  target_compile_options(${PROJECTNAME} PUBLIC "-mfloat-abi=hard")
  target_link_options(${PROJECTNAME} PUBLIC "-mfloat-abi=hard")

  # Need to add other gcc config for other cortex-m cores
  if (ARM_CPU STREQUAL "cortex-m55" )
     target_compile_options(${PROJECTNAME} PUBLIC "-march=armv8.1-m.main+mve.fp+fp.dp")
     target_compile_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-d16")
     target_link_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-d16")
  endif()

  if (ARM_CPU STREQUAL "cortex-m55+nomve.fp+nofp" )
     target_compile_options(${PROJECTNAME} PUBLIC "-march=armv8.1-m.main+dsp+fp.dp")
     target_compile_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-d16")
     target_link_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-d16")
  endif()
  

  if (ARM_CPU STREQUAL "cortex-m33" )
     target_compile_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-sp-d16")
     target_link_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-sp-d16")
  endif()

  if (ARM_CPU STREQUAL "cortex-m7" )
     target_compile_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-sp-d16")
     target_link_options(${PROJECTNAME} PUBLIC "-mfpu=fpv5-sp-d16")
  endif()

  if (ARM_CPU STREQUAL "cortex-m4" )
     target_compile_options(${PROJECTNAME} PUBLIC "-mfpu=fpv4-sp-d16")
     target_link_options(${PROJECTNAME} PUBLIC "-mfpu=fpv4-sp-d16")
  endif()

  #if (ARM_CPU STREQUAL "cortex-m0" )
  #   target_compile_options(${PROJECTNAME} PUBLIC "")
  #   target_link_options(${PROJECTNAME} PUBLIC "")
  #endif()
  
  if (ARM_CPU STREQUAL "cortex-a32" )
      if (NOT (NEON OR NEONEXPERIMENTAL))
        target_compile_options(${PROJECTNAME} PUBLIC "-march=armv8-a;-mfpu=vfpv3-d16")
        target_link_options(${PROJECTNAME} PUBLIC "-march=armv8-a;-mfpu=vfpv3-d16")
      endif()
  endif()
  
  if (ARM_CPU STREQUAL "cortex-a9" )
      if (NOT (NEON OR NEONEXPERIMENTAL))
        target_compile_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=vfpv3-d16")
        target_link_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=vfpv3-d16")
      endif()
  endif()
  
  if (ARM_CPU STREQUAL "cortex-a7" )
      if (NOT (NEON OR NEONEXPERIMENTAL))
          target_compile_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=vfpv3-d16")
          target_link_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=vfpv3-d16")
      endif()
  endif()
  
  if (ARM_CPU STREQUAL "cortex-a5" )
      if ((NEON OR NEONEXPERIMENTAL))
        target_compile_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=neon-vfpv4")
        target_link_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=neon-vfpv4")
      else()
        target_compile_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=vfpv3-d16")
        target_link_options(${PROJECTNAME} PUBLIC "-march=armv7-a;-mfpu=vfpv3-d16")
      endif()
  endif()
  target_link_options(${PROJECTNAME} PUBLIC "-specs=nosys.specs" )
  target_compile_options(${PROJECTNAME} PUBLIC "-specs=nosys.specs" )


  target_compile_options(${PROJECTNAME} PUBLIC 
    "-Wall"
    "-Werror=all"
    "-Wno-error=comment"
    "-g"
    "$<$<COMPILE_LANGUAGE:C>:-O1>"
    "$<$<COMPILE_LANGUAGE:CXX>:-O1>"
    "-ffunction-sections"
    "-fdata-sections"
    "-Wno-error=unused-variable"
    "-Wno-error=unused-parameter"
    "-Wno-error=unused-function"
    "-Wno-error=unused-but-set-variable"
    "-Wno-error=unused-result"
    "-Wno-error=maybe-uninitialized"
    "-Wno-error=sign-compare"
    "-Wno-error=strict-aliasing"
    "-Wno-error=unknown-pragmas"
    "-Wno-error=format"
  )

 
  



endfunction()


function(compilerSpecificPlatformConfigLibForM PROJECTNAME ROOT)
 
endfunction()

function(compilerSpecificPlatformConfigLibForA PROJECTNAME ROOT)
  
endfunction()

function(compilerSpecificPlatformConfigLibForR PROJECTNAME ROOT)
 
endfunction()

function(compilerSpecificPlatformConfigAppForM PROJECTNAME ROOT)
  target_link_libraries(${PROJECTNAME} 
    "-Xlinker --gc-sections"
    "-nostartfiles"
  )
endfunction()

function(compilerSpecificPlatformConfigAppForA PROJECTNAME ROOT)
  
endfunction()

function(compilerSpecificPlatformConfigAppForR PROJECTNAME ROOT)
 
endfunction()