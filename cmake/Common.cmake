#[[
General CMake options for all cepton repositories.
]]
# Test endianness
include(TestBigEndian)
test_big_endian(IS_BIG_ENDIAN)
if(IS_BIG_ENDIAN)
  message(FATAL_ERROR "Big endian is not supported!")
endif()

# Find ccache
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

# ------------------------------------------------------------------------------
# Macros
# ------------------------------------------------------------------------------
# Evaluate logical expression
macro(LOGICAL result predicate)
  # Split into list
  string(REGEX REPLACE " +" ";" logical_p "${predicate}")

  if(${logical_p})
    set(${result} TRUE)
  else()
    set(${result} FALSE)
  endif()
endmacro()

# Add cmake option
macro(CREATE_OPTION type key init_value docstring #[[predicate default_value]])
  set(create_option_enabled TRUE)
  if(${ARGC} EQUAL 6)
    logical(create_option_enabled ${ARGV4})
  endif()

  if(${create_option_enabled})
    if(DEFINED ${key})
      set(${key} "${${key}}" CACHE ${type} "${docstring}" FORCE)
    else()
      set(${key} "${init_value}" CACHE ${type} "${docstring}" FORCE)
    endif()
  else()
    set(${key} "${ARGV5}" CACHE INTERNAL "" FORCE)
  endif()
endmacro()

# Set cmake option
macro(SET_OPTION key value)
  set(${key} ${value} CACHE INTERNAL "" FORCE)
endmacro()

# Add subdirectory
macro(ADD_EXTERNAL_SUBDIRECTORY)
  message(STATUS "================================================================================")
  if(CEPTON_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_BUILD_TYPE "Release")
  else()
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  endif()
  get_filename_component(BASENAME "${ARGV0}" NAME)
  message(STATUS "External: " ${BASENAME})
  message(STATUS "--------------------------------------------------------------------------------")
  add_subdirectory(${ARGN})
  set(CMAKE_BUILD_TYPE ${CEPTON_BUILD_TYPE})
  message(STATUS "--------------------------------------------------------------------------------")
endmacro()

# Add compiler flags
macro(ADD_C_FLAGS flags)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${flags}")
endmacro()

macro(ADD_CXX_FLAGS flags)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flags}")
endmacro()

macro(ADD_FLAGS flags)
  ADD_C_FLAGS("${flags}")
  ADD_CXX_FLAGS("${flags}")
endmacro()

macro(ADD_RELEASE_FLAGS flags)
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${flags}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${flags}")
endmacro()

# ------------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------------
# Detect architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH_64 TRUE)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set(ARCH_32 TRUE)
else()
  message(FATAL_ERROR "Unsupported architecture!")
endif()

# Detect OS
if(WIN32)
  set(WINDOWS TRUE)
  add_definitions(-DIS_WINDOWS)
elseif(UNIX)
  add_definitions(-DIS_UNIX)
  if(APPLE)
    add_definitions(-DIS_APPLE)
  else()
    set(LINUX TRUE)
    add_definitions(-DIS_LINUX)
  endif()
else()
  message(FATAL_ERROR "Unsupported OS!")
endif()
if(NOT DEFINED WINDOWS)
  set(WINDOWS FALSE)
endif()
if(NOT DEFINED LINUX)
  set(LINUX FALSE)
endif()

# Detect OS name
if(WINDOWS)
  set(DEFAULT_OS_NAME "win64")
elseif(APPLE)
  set(DEFAULT_OS_NAME "osx")
elseif(LINUX)
  execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
  if("${MACHINE_ARCH}" MATCHES "^armv")
    set(DEFAULT_OS_NAME "linux-arm")
  else()
    set(DEFAULT_OS_NAME "linux-${MACHINE_ARCH}")
  endif()
endif()

# Detect compiler
if(MSVC)
  add_definitions(-DIS_MSVC)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(GCC TRUE)
  add_definitions(-DIS_GCC)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CLANG TRUE)
  add_definitions(-DIS_CLANG)
else()
  message(STATUS "Compiler: " ${CMAKE_CXX_COMPILER_ID})
  message(FATAL_ERROR "Unsupported compiler!")
endif()

# Detect number of processors
include(ProcessorCount)
ProcessorCount(N_PROCESSORS)
math(EXPR N_BUILD_JOBS "${N_PROCESSORS} - 2")

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
create_option(STRING OS_NAME "${DEFAULT_OS_NAME}"
  "OS name")
if(NOT DEFINED OS_NAME)
  set(OS_NAME "${DEFAULT_OS_NAME}")
endif()
message(STATUS "OS name: ${OS_NAME}")

# ------------------------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------------------------
if(MSVC)
  add_flags("/wd4996")
  add_flags("/wd4100") # Disable unused parameter warning
  add_flags("/wd4800 /wd4267 /wd4244 /wd4018") # Disable conversion warnings
elseif(GCC OR CLANG)
  ADD_C_FLAGS("-std=c11")
  ADD_CXX_FLAGS("-std=c++11") # C++11
  add_flags("-pthread")
  add_flags("-Wno-sign-compare") # Disable conversion warnings
  add_flags("-Wno-missing-field-initializers") # Disable struct initialization warning
  add_flags("-Wno-unused-parameter -Wno-unused-function -Wno-attributes") # Disable unused warnings
  if(CLANG)
    add_flags("-Wno-unused-command-line-argument") # Disable extra arguments warning
    add_flags("-Wno-inconsistent-missing-override")
    add_flags("-Wno-missing-braces") # Bug in clang
  endif()
endif()
