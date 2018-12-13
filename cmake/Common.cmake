#[[
General CMake options for all cepton repositories.
]]

get_filename_component(CEPTON_SDK_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)

# Test endianness
include(TestBigEndian)
test_big_endian(IS_BIG_ENDIAN)
if(IS_BIG_ENDIAN)
  message(FATAL_ERROR "Big endian is not supported!")
endif()

# ------------------------------------------------------------------------------
# Macros
# ------------------------------------------------------------------------------
# Print list with newlines
function(MESSAGE_LIST l)
  foreach(line IN LISTS ${l})
    message("${line}")
  endforeach() 
endfunction()

macro(STRING_APPEND key value)
  set(${key} "${${key}} ${value}")
endmacro()

# Expand path
macro(FIX_PATH result input)
  get_filename_component(${result} "${input}" ABSOLUTE)
endmacro()

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
    if(DEFINED ${key}_OPTION)
      set(${key}_OPTION "${${key}_OPTION}" CACHE ${type} "${docstring}" FORCE)
    elseif(DEFINED ${key})
      set(${key}_OPTION "${${key}}" CACHE ${type} "${docstring}" FORCE)
    else()
      set(${key}_OPTION "${init_value}" CACHE ${type} "${docstring}" FORCE)
    endif()
  else()
    set(${key}_OPTION "${ARGV5}" CACHE INTERNAL "" FORCE)
  endif()

  # Internally modifiable value
  set(${key} "${${key}_OPTION}" CACHE INTERNAL "" FORCE)
endmacro()

# Set cmake option
macro(SET_OPTION key value)
  set(${key} ${value} CACHE INTERNAL "" FORCE)
  set(${key}_OPTION ${value} CACHE INTERNAL "" FORCE)
endmacro()

# Add compiler flags
macro(ADD_C_FLAGS flags)
  string_append(CMAKE_C_FLAGS "${flags}")
endmacro()

macro(ADD_CXX_FLAGS flags)
  string_append(CMAKE_CXX_FLAGS "${flags}")
endmacro()

macro(ADD_FLAGS flags)
  add_c_flags("${flags}")
  add_cxx_flags("${flags}")
endmacro()

macro(ADD_FLAGS_RELEASE flags)
  string_append(CMAKE_C_FLAGS_RELEASE "${flags}")
  string_append(CMAKE_CXX_FLAGS_RELEASE "${flags}")
endmacro()

macro(ADD_LINKER_FLAGS flags)
  string_append(CMAKE_EXE_LINKER_FLAGS "${flags}")
endmacro()

macro(ADD_LINKER_FLAGS_RELEASE flags)
  string_append(CMAKE_EXE_LINKER_FLAGS_RELEASE "${flags}")
endmacro()

# Add subdirectory
macro(ADD_EXTERNAL_SUBDIRECTORY source_dir)
  message(STATUS "================================================================================")
  get_filename_component(add_external_subdirectory_name "${source_dir}" NAME)
  message(STATUS "External: " ${add_external_subdirectory_name})
  message(STATUS "--------------------------------------------------------------------------------")
  set(add_external_subdirectory_args "${source_dir}" ${ARGN})
  add_subdirectory("${CEPTON_SDK_SOURCE_DIR}/cmake/subdirectory" 
    "${PROJECT_BINARY_DIR}/third_party/cepton_subdirectory_${add_external_subdirectory_name}")
  message(STATUS "--------------------------------------------------------------------------------")
endmacro()

# ------------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------------
# Detect subdirectory
get_directory_property(parent_directory PARENT_DIRECTORY)
if(parent_directory)
  set(IS_SUBDIRECTORY TRUE)
else()
  set(IS_SUBDIRECTORY FALSE)
endif()

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
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang$")
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

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release")
endif()

# ------------------------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------------------------
if(MSVC)
  add_flags("/wd4996")
  add_flags("/wd4100") # Disable unused parameter warning
  add_flags("/wd4800 /wd4267 /wd4244 /wd4018") # Disable conversion warnings
elseif(GCC OR CLANG)
  add_c_flags("-std=c11") # Enable c11
  add_cxx_flags("-std=c++11") # Enable c++11 
  add_flags("-pthread") # Enable threading
  add_flags("-Wall") # Enable warnings
  add_flags("-Wno-sign-compare") # Disable conversion warnings
  add_flags("-Wno-missing-field-initializers") # Disable struct initialization warning
  add_flags("-Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-attributes") # Disable unused warnings
  if(CLANG)
    add_flags("-Wno-unused-command-line-argument") # Disable extra arguments warning
    add_flags("-Wno-inconsistent-missing-override")
    add_flags("-Wno-missing-braces") # Bug in clang
  endif()
endif()