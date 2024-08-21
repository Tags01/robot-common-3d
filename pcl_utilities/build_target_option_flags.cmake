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

function (btof_get_scoped_global OUTPUT POSTFIX)
  if (DEFINED ${PROJECT_NAME}_${POSTFIX})
    set(${OUTPUT} ${PROJECT_NAME}_${POSTFIX} PARENT_SCOPE)
  elseif(DEFINED ENV{${PROJECT_NAME}_${POSTFIX}})
    set(${OUTPUT} $ENV{${PROJECT_NAME}_${POSTFIX}} PARENT_SCOPE)
  elseif(DEFINED DEFAULT_${POSTFIX})
    set(${OUTPUT} ${DEFAULT_${POSTIX}} PARENT_SCOPE)
  elseif(DEFINED ENV{DEFAULT_${POSTFIX}})
    set(${OUTPUT} $ENV{DEFAULT_${POSTFIX}} PARENT_SCOPE)
  endif()
endfunction()

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

function(btof_gnulike_use_linker TARGET VISIBILITY LANG)
  cmake_parse_arguments("ARGV" "" "LINKER;FAILURE" "" ${ARGN})

  if (NOT DEFINED ARGV_LINKER)
    btof_get_scoped_global(ARGV_LINKER "LINKER")
  endif()

  if (NOT DEFINED ARGV_LINKER AND DEFINED ARGV_FAILURE)
    set(${ARGV_FAILURE}
      "Must be provided to use `-fuse-ld` on the target '${TARGET}'\n"
      "Please provide an argument 'LINKER' or specify a environment/build "
      "variable named ${PROJECT_NAME}_LINKER or GLOBAL_LINKER" PARENT_SCOPE)
  endif()

  if (DEFINED ARGV_LINKER AND CMAKE_${LANG}_COMPILER_ID MATCHES "${BTOF_GNULIKE_MATCH_REGEX}")
    target_link_options(${TARGET} ${VISIBILITY} -fuse-ld=${ARGV_LINKER})
    set_property(
      TARGET ${TARGET} APPEND PROPERTY STATIC_LIBRARY_OPTIONS -fuse-ld=${LINKER})
  endif()
endfunction()

function (btof_add_sanitizers TARGET VISIBILITY LANG)
  cmake_parse_arguments("ARGV" "" "FAILURE" "SANITIZERS" ${ARGN})
  if (NOT DEFINED ARGV_SANITIZERS)
    btof_get_scoped_global(ARGV_SANITIZERS "SANITIZERS")
  endif()

  string(REGEX REPLACE "[,\t\n\r;:]+" "," SANITIZERS "${ARGV_SANITIZERS}")
  string(TOLOWER "${SANITIZERS}" SANITIZERS)

  if (NOT SANITIZERS OR SANITIZERS STREQUAL "")
    if (DEFINED ARGV_FAILURE OR ON)
      message(WARN #${ARGV_FAILURE}
        "No Sanitizers are specified, please provide the 'SANITIZERS' named argument "
        "or set the value as a build or environment variable")
    endif()
    return()
  endif()

  if (NOT CMAKE_${LANG}_COMPILER_ID MATCHES "${BTOF_GNULIKE_MATCH_REGEX}")
    if (DEFINED ARGV_FAILURE OR ON)
      message(WARN #${ARGV_FAILURE}
        "Sanitizers could not be added to the target, '${TARGET}'"
        "due to an unsupported compiler. Only GCC and CLANG-like compilers are "
        "supported" PARENT_SCOPE)
    endif()
    return()
  endif()

  set(SANITIZER_FLAGS)

  # CFI sanitizer requires use with LTO and a proper linker
  # This requires setting the visibility to hidden where possible and using a linker that is not lld
  if (SANITIZERS MATCHES ".*cfi.*")
    check_ipo_supported(RESULT IS_IPO_SUPPORTED OUTPUT REASON LANGUAGES ${LANG})
    btof_gnulike_use_linker(${TARGET} ${VISIBILITY} ${LANG} ${LINKER_PLUGIN} FAILURE LINK_FAIL_REASON)

    if (NOT IS_IPO_SUPPORTED)
      message(WARNING ${ARGV_FAILURE}
        "Sanitizers could not be added to the target '${target}' "
        "Since LTO is not supported for the following reason: \n ${REASON}")
      string(REGEX REPLACE "cfi[A-Za-z_\\-]*,?" "" SANITIZERS "${SANITIZERS}")
    elseif (DEFINED LINK_FAIL_REASON)
      message(WARNING
        "A linker must be specified to use the 'cfi' sanitizer. "
        "Skipping CFI SANITIZER: \n${REASON}")
      string(REGEX REPLACE "cfi[A-Za-z_\\-]*,?" "" SANITIZERS "${SANITIZERS}")
    else()
      set_target_properties(${TARGET} PROPERTIES INTERPROCEDURAL_OPTIMIZATION ON)
      list(APPEND SANITIZER_FLAGS -fvisibility=hidden -fsanitize-cfi-cross-dso)
    endif()
  endif()

  message(WARNING "SANITIZERS: ${SANITIZERS}")

  if (SANITIZERS STREQUAL "")
    return()
  endif()

  list(APPEND SANITIZER_FLAGS -fsanitize=${SANITIZERS} -fsanitize-recover -fno-sanitize-trap)

  # fortify source cannot be higher than 0 when using sanitizers
  target_compile_options(${TARGET} ${VISIBILITY}
    -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -fno-omit-frame-pointer
    ${SANITIZER_FLAGS})

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
  btof_gnulike_use_linker
  btof_default_gnulike_optimize)

# Default options for building targets
set(BTOF_DEFAULT ${BTOF_DEFAULT_DEBUG} ${BTOF_DEFAULT_RELEASE})

function(botf_target_configurations TARGET VISIBILITY)
  cmake_parse_arguments("ARGV" "" "LANG" "FUNCTIONS;ARGS;FAILURE" ${ARGN})

  # Set the langauage of the executable if not CXX
  if (NOT DEFINED ARGV_LANG)
    set(ARGV_LANG CXX)
  endif()

  get_property(LANGUAGES_STR GLOBAL PROPERTY ENABLED_LANGUAGES)
  set(LANGUAGES ${LANGUAGES_STR})
  if (NOT ARGV_LANG IN_LIST LANGUAGES)
    message(SEND_ERROR
      "The language '${ARGV_LANG}' on the target '${TARGET}' is not one "
      "of the listed enabled languages: '${LANGUAGES}'"
    )
  endif()

  if (NOT DEFINED ARGV_FUNCTIONS)
    message(SEND_ERROR
      "The FUNCTIONS argument must be provided")
  endif()

  if (NOT DEFINED ARGV_ARGS)
    set(ARGV_ARGS)
  endif()

  set(FAILURE_REASONS)
  foreach(CALLBACK ${ARGV_FUNCTIONS})
    if (DEFINED)
      unset(LOCAL_FAILURES)
    endif()

    if (COMMAND ${CALLBACK})
      cmake_language(CALL ${CALLBACK} ${TARGET} ${VISIBILITY} ${ARGV_LANG} FAILURE LOCAL_FAILURES)
    else()
      message(SEND_ERROR "The FUNCTIONS argument '${CALLBACK}' is not a function or macro")
    endif()

    if (DEFINED LOCAL_FAILURES)
      list(APPEND FAILURE_REASONS ${LOCAL_FAILURES})
    endif()
  endforeach()

  set(ARGV_FAILURE ${FAILURE_REASONS})
endfunction()
