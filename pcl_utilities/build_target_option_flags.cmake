cmake_minimum_required(VERSION 3.18)
include_guard(GLOBAL)

# Use new policy with interprocedural optimization in this file
cmake_policy(SET CMP0069 NEW)
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
  -Wshadow = would invalidate standard compliant code, best to allow code to opt in
  -ftrapv/-fmunmap = obsolete with sanitizers
  -Wl, * linker flags = lack of portability
  --fstack-protector-all = not designed for debugging, little gain from use
  glib assertion definitions = issues when linking with other executables with those flags off.
]]#

set(BTOF_CLANGLIKE_MATCH_REGEX "(Clang|IntelLLVM|AppleClang|ARMClang|XLClang|IBMClang)")
set(BTOF_GNULIKE_MATCH_REGEX "(GNU|Clang|IntelLLVM|AppleClang|ARMClang|XLClang|IBMClang)")

# Experimental Code
# macro(btof_match_gnulike LANG REGEX_MATCHER)
#   ((DEFINED CMAKE_${LANG}_COMPILER_FRONTEND_VARIANT AND
#    CMAKE_${LANG}_COMPILER_FRONTEND_VARIANT MATCHES ${REGEX_MATCHER}) OR
#   CMAKE_${LANG}_COMPILER_ID MATCHES ${REGEX_MATCHER})
# endmacro()

function (btof_default_release_ipo TARGET VISIBILITY LANG)
  check_ipo_supported(RESULT IS_LTO_SUPPORTED OUTPUT UNUSED LANGUAGES ${LANG})

  if (IS_LTO_SUPPORTED)
    set_target_properties(${TARGET} PROPERTIES
      INTERPROCEDURAL_OPTIMIZATION_RELEASE ON
      INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON
      INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
  endif()
endfunction()

function(btof_default_debug_definitions TARGET VISIBILITY LANG)
  target_compile_definitions(${TARGET} ${VISIBILITY}
    "$<$<CONFIG:debug,relwithdebinfo>:NDEBUG;_FORTIFY_SOURCE=3>")
endfunction()

function(btof_default_gnulike_pedantic TARGET VISIBILITY LANG)
  if (CMAKE_${LANG}_COMPILER_ID MATCHES "${BTOF_GNULIKE_MATCH_REGEX}")
    target_compile_options(${TARGET} ${VISIBILITY}
      -Wall;-Wextra;-Wpedantic
      -Wfloat-equal;-Wundef;-Wshadow
      -Winit-self;-Wpointer-arith;-Wcast-align
      -Wstrict-overflow=3;-Wwrite-strings;-Wswitch-enum
      -Wswitch-default;-Wformat=2;-Wcast-qual)
  endif()
endfunction()

function(btof_default_gnulike_optimize TARGET VISIBILITY LANG)
  if (CMAKE_${LANG}_COMPILER_ID MATCHES "${BTOF_GNULIKE_MATCH_REGEX}")
    set (OPTIONS
      "$<$<CONFIG:Release,RelWithDebInfo>:-O3>"
      "$<$<CONFIG:MinSizeRel>:-Os>")

    target_compile_options(${TARGET} ${VISIBILITY} ${OPTIONS})
    target_link_options(${TARGET} ${VISIBILITY} ${OPTIONS})
  endif()
endfunction()

function(btof_default_gnulike_debug TARGET VISIBILITY LANG)
  if (CMAKE_${LANG}_COMPILER_ID MATCHES "${BTOF_GNULIKE_MATCH_REGEX}")
    target_compile_options(${TARGET} ${VISIBILITY}
      "$<$<CONFIG:Debug,RelWithDebInfo>:-g3>")
  endif()
endfunction()

function (btof_add_sanitizers TARGET VISIBILITY LANG)
  # Would be good to support sanitizers per project level, ie. ${PROJECT_NAME}_SANITIZERS=...

  set (SANITIZERS)
  if (NOT "${ARGN}" STREQUAL "")
    list(APPEND SANITIZERS ${ARGN})
  elseif(DEFINED ${PROJECT_NAME}_SANITIZERS)
    list(APPEND SANITIZERS ${${PROJECT_NAME}_SANITIZERS})
  elseif(DEFINED ENV{${PROJECT_NAME}_SANITIZERS})
    list(APPEND SANITIZERS $ENV{${PROJECT_NAME}_SANITIZERS})
  elseif(DEFINED SANITIZERS)
    list(APPEND SANITIZERS ${SANITIZERS})
  elseif(DEFINED ENV{SANITIZERS})
    list(APPEND SANITIZERS ${SANITIZERS})
  endif()

  # format the provided sanitizers so they can be passed to clang
  string(REGEX REPLACE "[\n \t\r]" "" SANITIZERS "${SANITIZERS}")
  string(REGEX REPLACE "[:; ,]+" "," SANITIZERS "${SANITIZERS}")
  string(TOLOWER "${SANITIZERS}" SANITIZERS)
  message(WARNING "SANITIZERS are ${SANITIZERS}")

  if ("${SANITIZERS}" STREQUAL "")
    return()
  endif()

  if (NOT CMAKE_${LANG}_COMPILER_ID MATCHES "${BTOF_GNULIKE_MATCH_REGEX}")
    message(WARNING
      "Sanitizers could not be added to the target, '${TARGET}'"
      "due to an unsupported compiler. Only GCC and CLANG-like compilers are supported"
    )
    return()
  endif()

  # CFI sanitizer requires use with LTO
  if (SANITIZERS STREQUAL "cfi")
    check_ipo_supported(RESULT IS_IPO_SUPPORTED OUTPUT REASON LANGUAGES ${LANG})
    if (NOT IS_IPO_SUPPORTED)
      message(WARNING "The CFI sanitizer cannot be used if lto is not enabled. LTO Could not be enabled for ${REASON}")
      return()
    endif()

    set_target_properties(${TARGET} PROPERTIES
      INTERPROCEDURAL_OPTIMIZATION ON
    )

    set_property(
      TARGET ${TARGET} APPEND PROPERTY STATIC_LIBRARY_OPTIONS
      -flto-visibility=hidden)
  endif()

  set(SANITIZER_FLAGS -fsanitize=${SANITIZERS} -fsanitize-recover)

  # fortify source cannot be higher than 0 when using sanitizers
  target_compile_options(${TARGET} ${VISIBILITY}
    -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 ${SANITIZER_FLAGS})

  target_link_options(${TARGET} ${VISIBILITY} ${SANITIZER_FLAGS})
  set_property(
    TARGET ${TARGET} APPEND PROPERTY STATIC_LIBRARY_OPTIONS
    ${SANITIZER_FLAGS})

endfunction()

# define which functions will be called in which order
# order is important
set(BTOF_DEFAULT_COMPILE
  btof_default_gnulike_pedantic)
set(BTOF_DEFAULT_DEBUG btof_default_debug_definitions
  btof_default_gnulike_debug)
set(BTOF_DEFAULT_RELEASE
  btof_default_release_ipo
  btof_default_gnulike_optimize)

# Default options for building targets
set(BTOF_DEFAULT ${BTOF_DEFAULT_COMPILE} ${BTOF_DEFAULT_DEBUG} ${BTOF_DEFAULT_RELEASE})

function(botf_target_configurations TARGET VISIBILITY LANG)
  string(TOUPPER ${LANG} LANG)

  if (NOT DEFINED CMAKE_${LANG}_COMPILER)
    message(WARNING "A compiler unknown to CMake was specified, options include C, CXX, Fortran")
  endif()

  foreach(CALLBACK ${ARGN})
    cmake_language(CALL ${CALLBACK} ${TARGET} ${VISIBILITY} ${LANG})
  endforeach()
endfunction()
