cmake_minimum_required(VERSION 3.18)

include(CheckIPOSupported)

#[[
Clang Based Compiler and GNU:
  Syntax:
  -Wall -Wextra -Wpedantic = enables most compiler errors
  -Wfloat-equal = catches direct float comparison (e.g 0.1 + 0.2 == 0.3)
  -Wundef = catches undefined macros used in an #if context
  -Winit-self = catches e.g `int i = i`
  -Wunitalized -Winit-self = catches undefined variables
  -Wpointer-arith = UB from arithmentic operations on void* or function ptr
  -Wcast-align = casting to type with a larger alignment (e.g `reinterpret_cast<double*>(char*)`)
  -Wstrict-overflow = catch assumptions about signed overflow which are actually UB, (eg. `x + 1 < x`)
  -Wwrite-strings = assinging a string-literal a char* without const
  -Wswitch-enum -Wswitch-default = missing variants of an enum, missing default case
  -Wformat=2 = invalid usage of printf/RCLCPP_*

  Others:
  -O3 = full optimizations (without breaking standard compliance, e.g Ofast)
  -Os = optimize for size
  -O0 = no optimizations
  -fsanitize = add sanitizers; useful for debugging undefined behavior
  -fsanitize-recover = allow sanitizers to continue execution upon an error
  -g3 = add all debugging symbols, including macros

MSVC:
    /Wall = enable all warnings
    /EHa = allow C++ exceptions
    /fp:strict = do not perform optimizations can break c++ floating
                 point compliance
    /Ot = optimize for speed
    /Os = optimize for size
    /Od = no optimizations
    /Z7 = export debugging info within executable
    /RTC1 = unitialized and stack variable checks
    /analyse = script analysation
    /Ot = optimize for speed
    /Gy = omit frame pointer (must not do this in debugging but is useful for release)
    /Qpar = automatically parallelize loops

Defintions and Properties:
  INTERPRODEDUAL_OPTIMIZATION = perform link time optimization
  _FORTIFY_SOURCE = crash program if stack protection is detected
                    (MUST NOT use with sanitizers)
  NDEBUG = enable asserts information

Omitted Flags:
  -march=native = Executable will not run on other machines (also makes profiling harder)
  -Ofast / fp:fast = Allows floating-point optimization beyond what the C++ standard allows
  -Waggregate-return = irrelevent, would dissallow a common c++ practice
  -Wunreachable-code = poor compiler support
  -Wshadow = would invalidate standard compliant code, best to opt in
  -ftrapv/-fmunmap = obsolite with sanitizers
  -Wl, * linker flags = lack of portability
  --fstack-protector-all = not designed for debugging, little gain from use
  glib assertion definitions = issues when linking with other executables with those flags off.
]]#


set(BTOF_GNULIKE_MATCH_REGEX "(GNU|Clang|IntelLLVM|AppleClang|ARMClang|XLClang|IBMClang)")

function (btof_default_release_ipo TARGET VISIBILITY BUILD_TYPE)
  check_ipo_supported(RESULT IS_LTO_SUPPORTED)
  if (NOT IS_LTO_SUPPORTED)
    return()
  endif()

  if (BUILD_TYPE MATCHES "(release|minsizerel|relwithdebinfo)")
    set(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
    return()
  else()
    set_target_properties(${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    set_target_properties(${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON)
    set_target_properties(${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
  endif()
endfunction()

function(btof_default_debug_definitions TARGET VISIBILITY BUILD_TYPE)
  target_compile_definitions(${TARGET} ${VISIBILITY}
    "$<$<STREQUAL:${BUILD_TYPE},debug>: NDEBUG _FORTIFY_SOURCE=3>"
    "$<$<STREQUAL:${BUILD_TYPE},relwithdebinfo>: NDEBUG _FORTIFY_SOURCE=3>"
  )
endfunction()

function(btof_default_gnulike_pedantic TARGET VISIBILITY)
  if (NOT CMAKE_CXX_COMPILER_ID MATCHES ${BTOF_GNULIKE_MATCH_REGEX})
    return()
  endif()

  message("btof_default_gnulike_pedantic")

  target_compile_options(${TARGET} ${VISIBILITY}
    -Wall -Wextra -Wpedantic
    -Wfloat-equal -Wundef -Wshadow
    -Winit-self -Wpointer-arith -Wcast-align
    -Wstrict-overflow=3 -Wwrite-strings -Wswitch-enum
    -Wswitch-default -Wformat=2 -Wcast-qual)
endfunction()

function(btof_default_gnulike_optimize TARGET VISIBILITY BUILD_TYPE_GEN)
  if (CMAKE_CXX_COMPILER_ID MATCHES ${BTOF_GNULIKE_MATCH_REGEX})
    message(btof_default_gnulike_optimize)
    BTOF_IF_BUILD_TYPE(${BUILD_TYPE}, release, -O3)

    set (OPTIONS
      "$<$<STREQUAL:${BUILD_TYPE},release>: -O3>"
      "$<$<STREQUAL:${BUILD_TYPE},minsizerel>: -Os>"
      "$<$<STREQUAL:${BUILD_TYPE},relwithdebinfo>: -O3>")

    message("${TARGET} ${VISIBILITY} ${OPTIONS} ${BUILD_TYPE}")

    target_compile_options(${TARGET} ${VISIBILITY} ${OPTIONS})
    target_link_options(${TARGET} ${VISIBILITY} ${OPTIONS})
  endif()
endfunction()

function(btof_default_gnulike_debug TARGET VISIBILITY BUILD_TYPE_GEN)
  if (CMAKE_CXX_COMPILER_ID MATCHES ${BTOF_GNULIKE_MATCH_REGEX})
    message("btof_default_gnulike_debug")

  BTOF_IF_BUILD_TYPE(${BUILD_TYPE}, release, -g3)
  BTOF_IF_BUILD_TYPE(${BUILD_TYPE}, relwithdebinfo, -g3)

  target_compile_options(${TARGET} ${VISIBILITY}
    "$<$<STREQUAL:${BUILD_TYPE},debug>: -g3>"
    "$<$<STREQUAL:${BUILD_TYPE},relwithdebinfo>: -g3>")
  endif()
endfunction()

function (btof_default_msvclike_syntax TARGET VISIBILITY)
  if (CXX_COMPILER_ID STREQUAL "MSVC")
    message("btof_default_msvclike_syntax")
    add_compile_options(${TARGET} ${VISIBILITY}
      /Wall /fp:strict /EHa)
  endif()
endfunction()

function (btof_default_msvclike_optimize TARGET VISIBILITY BUILD_TYPE)
  if (CXX_COMPILER_ID STREQUAL "MSVC")
    message("btof_default_msvclike_optimize")
    target_compile_options(${TARGET} ${VISIBILITY}
      "$<$<STREQUAL:${BUILD_TYPE},relminsize>: /Os /Gy /Qpar>"
      "$<$<STREQUAL:${BUILD_TYPE},release>: /Od /Gy /Qpar>"
      "$<$<STREQUAL:${BUILD_TYPE},relwithdebinfo>: /Od /Qpar>"
    )
  endif()
endfunction()

function (btof_default_msvclike_debug TARGET VISIBILITY BUILD_TYPE)
  if (CXX_COMPILER_ID STREQUAL "MSVC")
    message("btof_default_msvclike_debug")
    target_compile_options(${TARGET} ${VISIBILITY}
      "$<$<STREQUAL:${BUILD_TYPE},minsizerel>: /Os /Gy /Qpar>"
      "$<$<STREQUAL:${BUILD_TYPE},release>: /Ot /Gy /Qpar>"
      "$<$<STREQUAL:${BUILD_TYPE},relwithdebinfo>: /Od /Qpar>"
      "$<$<STREQUAL:${BUILD_TYPE},debug>: /Od>")
  endif()
endfunction()

# define which functions will be called in which order
# order is important
set(BTOF_DEFAULT_COMPILE
  btof_default_gnulike_pedantic btof_default_msvclike_syntax)
set(BTOF_DEFAULT_DEBUG btof_default_debug_definitions
  btof_default_gnulike_debug btof_default_msvclike_debug)
set(BTOF_DEFAULT_RELEASE btof_default_release_ipo
  btof_default_gnulike_optimize btof_default_msvclike_optimize)

# These options are designed to trigger warnings for unknown compilers and releases
set(BTOF_KNOWN_BUILD_TYPES relwithdebinfo release minsizerel debug)
set(BTOF_KNOWN_COMPILERS Clang MSVC IntelLLVM GLU)

# Default options for building targets
set(BTOF_DEFAULT ${BTOF_DEFAULT_COMPILE} ${BTOF_DEFAULT_DEBUG} ${BTOF_DEFAULT_RELEASE})

function (target_add_sanitizers TARGET TYPE)
  if ("${ARGN}" STREQUAL "")
    return()
  endif()

  string(REGEX REPLACE "[\n \t\r]" "" SANITIZERS ${ARGN})
  string(REGEX REPLACE "[:; ,]+" "," SANITIZERS ${SANITIZERS})
  string(TOLOWER "${SANITIZERS}" SANITIZERS)

  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES ${BTOF_GNULIKE_MATCH_REGEX})
    target_compile_options(${TARGET} ${TYPE}
      -fsanitize=${SANITIZERS} -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0)
    target_link_options(${TARGET} ${TYPE} -fsanitize=${SANITIZERS})
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_options(${TARGET} ${TYPE}
    /fsanitize=${SANITIZERS} /U_FORTIFY_SOURCE /D_FORTIFY_SOURCE)
    target_link_options(${TARGET} ${TYPE} /fsanitize=${SANITIZERS})
  else()
    message(WARNING
      "Sanitizers could not be added to the target, '${TARGET}'"
      "due to an unsupported compiler. Only MSVC and CLANG are supported"
    )
  endif()
endfunction()

function(target_add_configuration_from_build_type TARGET VISIBILITY BUILD_TYPE_OR_GEN)
  if (BUILD_TYPE_OR_GEN MATCHES "DEFAULT")
    set(BUILD_TYPE_OR_GEN "$<LOWER_CASE:$<CONFIG>>")
  else()
    string(TOLOWER "${BUILD_TYPE}" BUILD_TYPE)
    list(FIND BUILD_TYPE "${BTOF_KNOWN_BUILD_TYPES}" INDEX_KNOWN_BUILD_TYPE)
    list(FIND CMAKE_CXX_COMPILER_ID "${BTOF_KNOWN_COMPILERS}" INDEX_KNOWN_COMPILER)

    if (INDEX_KNOWN_BUILD_TYPE EQUAL -1)
      message(WARNING "An unknown build type was provided '${BUILD_TYPE}' "
      "Parsing functions may not be designed to handle  this build type "
      "if this error occurs by mistake, add '${BUILD_TYPE}' to BTOF_KNOWN_BUILD_TYPES ")
    endif()
  endif()

  if (INDEX_KNOWN_COMPILER EQUAL -1)
    message(WARNING "An unknown compiler was provided '${CMAKE_CXX_COMPILER_ID}' "
      "Parsing functions may not be designed to handle this compiler "
      "if this error occurs by mistake, add '${CMAKE_CXX_COMPILER_ID}' to BTOF_KNOWN_COMPILERS")
  endif()

  foreach(callback ${ARGN})
    cmake_language(CALL ${callback} ${TARGET} ${VISIBILITY} ${BUILD_TYPE})
  endforeach()
endfunction()
