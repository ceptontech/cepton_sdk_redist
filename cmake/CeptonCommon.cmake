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

set(CEPTON_DEFINITIONS "")
set(CEPTON_FLAGS "")

# ------------------------------------------------------------------------------
# Macros
# ------------------------------------------------------------------------------
# Print list with newlines
function(MESSAGE_LIST l)
  foreach(line IN LISTS ${l})
    message("${line}")
  endforeach() 
endfunction()

# Append to string
function(STRING_APPEND result other)
  set(${result} "${${result}} ${other}" PARENT_SCOPE)
endfunction()

# Join list
function(STRING_JOIN result delimiter)
  string(REPLACE ";" "${delimiter}" ${result} "${ARGN}")
  set(${result} "${${result}}" PARENT_SCOPE)
endfunction()

# Expand path
function(FIX_PATH result input)
  get_filename_component(${result} "${input}" ABSOLUTE)
  set(${result} "${${result}}" PARENT_SCOPE)
endfunction()

# Evaluate logical expression
function(LOGICAL result predicate)
  # Split into list
  string(REGEX REPLACE " +" ";" p "${predicate}")
  if(${p})
    set(${result} TRUE PARENT_SCOPE)
  else()
    set(${result} FALSE PARENT_SCOPE)
  endif()
endfunction()

# Add cmake option
function(CREATE_OPTION type key init_value docstring #[[predicate default_value]])
  set(enabled TRUE)
  if(${ARGC} EQUAL 6)
    logical(enabled ${ARGV4})
  endif()

  if(${enabled})
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
endfunction()

# Set cmake option
function(SET_OPTION key value)
  set(${key} ${value} CACHE INTERNAL "" FORCE)
  set(${key}_OPTION ${value} CACHE INTERNAL "" FORCE)
endfunction()

# Add compiler flags
function(ADD_C_FLAGS)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_C_FLAGS ${flags})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" PARENT_SCOPE)
endfunction()

function(ADD_CXX_FLAGS)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_CXX_FLAGS ${flags})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" PARENT_SCOPE)
endfunction()

function(ADD_FLAGS)
  add_c_flags(${ARGN})
  add_cxx_flags(${ARGN})
endfunction()

function(ADD_FLAGS_RELEASE flags)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_C_FLAGS_RELEASE ${flags})
  string_append(CMAKE_CXX_FLAGS_RELEASE ${flags})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" PARENT_SCOPE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" PARENT_SCOPE)
endfunction()

function(ADD_LINKER_FLAGS)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_EXE_LINKER_FLAGS ${flags})
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}" PARENT_SCOPE)
endfunction()

function(ADD_LINKER_FLAGS_RELEASE)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_EXE_LINKER_FLAGS_RELEASE ${flags})
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" PARENT_SCOPE)
endfunction()

if(GCC)
  set(DISABLE_WARNINGS_FLAGS -w)
elseif(CLANG)
  set(DISABLE_WARNINGS_FLAGS -Wno-everything)
elseif(MSVC)
  set(DISABLE_WARNINGS_FLAGS /W0)
endif()

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

# This is a major hack to treat subdirectory target as imported, so that include
# directories are treated as system.
# https://github.com/conan-io/conan/issues/2125
# function(ADD_CLONED_IMPORTED_TARGET result lib)
# 	get_property(_INTERFACE_LINK_LIBRARIES TARGET ${lib} PROPERTY INTERFACE_LINK_LIBRARIES)
# 	get_property(_INTERFACE_INCLUDE_DIRECTORIES TARGET ${lib} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
# 	get_property(_INTERFACE_COMPILE_DEFINITIONS TARGET ${lib} PROPERTY INTERFACE_COMPILE_DEFINITIONS)
#   get_property(_INTERFACE_COMPILE_OPTIONS TARGET ${lib} PROPERTY INTERFACE_COMPILE_OPTIONS)

#   get_property(lib_path TARGET ${lib} PROPERTY LOCATION)
#   message(${lib_path})

#   add_library(${result} UNKNOWN IMPORTED)
#   add_dependencies(${result} ${lib})
#   set_property(TARGET ${result} PROPERTY IMPORTED_LOCATION $<TARGET_FILE:${lib}>)
# 	set_property(TARGET ${result} PROPERTY INTERFACE_LINK_LIBRARIES ${_INTERFACE_LINK_LIBRARIES})
# 	set_property(TARGET ${result} PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${_INTERFACE_INCLUDE_DIRECTORIES})
# 	set_property(TARGET ${result} PROPERTY INTERFACE_COMPILE_DEFINITIONS ${_INTERFACE_COMPILE_DEFINITIONS})
# 	set_property(TARGET ${result} PROPERTY INTERFACE_COMPILE_OPTIONS ${_INTERFACE_COMPILE_OPTIONS})
# endfunction()

function(CEPTON_GET_SHARED_LIBRARY result root lib_name)
  if(WINDOWS)
    set(${result} "${root}/bin/${OS_NAME}/${lib_name}.dll" PARENT_SCOPE)
  elseif(APPLE)
    set(${result} "${root}/lib/${OS_NAME}/${lib_name}.dylib" PARENT_SCOPE)
  elseif(LINUX)
    set(${result} "${root}/lib/${OS_NAME}/lib${lib_name}.so" PARENT_SCOPE)
  endif()
endfunction()

function(CEPTON_GET_STATIC_LIBRARY result root lib_name)
  if(WINDOWS)
    set(${result} "${root}/lib/${OS_NAME}/${lib_name}.lib" PARENT_SCOPE)
  elseif(APPLE OR LINUX)
    set(${result} "${root}/lib/${OS_NAME}/lib${lib_name}.a" PARENT_SCOPE)
  endif()
endfunction()

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
  list(APPEND CEPTON_DEFINITIONS IS_WINDOWS)
elseif(UNIX)
  list(APPEND CEPTON_DEFINITIONS IS_UNIX)
  if(APPLE)
    list(APPEND CEPTON_DEFINITIONS IS_APPLE)
  else()
    set(LINUX TRUE)
    list(APPEND CEPTON_DEFINITIONS IS_LINUX)
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
  list(APPEND CEPTON_DEFINITIONS IS_MSVC)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(GCC TRUE)
  list(APPEND CEPTON_DEFINITIONS IS_GCC)
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang$")
  set(CLANG TRUE)
  list(APPEND CEPTON_DEFINITIONS IS_CLANG)
else()
  message(STATUS "Compiler: " ${CMAKE_CXX_COMPILER_ID})
  message(FATAL_ERROR "Unsupported compiler!")
endif()

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
set(CMAKE_CXX_STANDARD 11)

if(MSVC)
  list(APPEND CEPTON_FLAGS /wd4996) # Disable depricated warning
  list(APPEND CEPTON_FLAGS /wd4100) # Disable unused parameter warning
  list(APPEND CEPTON_FLAGS /wd4800 /wd4267 /wd4244 /wd4018) # Disable type conversion warnings
elseif(GCC OR CLANG)
  list(APPEND CEPTON_FLAGS -Wall) # Enable warnings
  list(APPEND CEPTON_FLAGS -Wno-sign-compare) # Disable type conversion warnings
  list(APPEND CEPTON_FLAGS -Wno-missing-field-initializers) # Disable struct initialization warning
  list(APPEND CEPTON_FLAGS -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-attributes) # Disable unused warnings
  if(CLANG)
    list(APPEND CEPTON_FLAGS -Wno-unused-command-line-argument) # Disable extra arguments warning
    list(APPEND CEPTON_FLAGS -Wno-inconsistent-missing-override)
    list(APPEND CEPTON_FLAGS -Wno-missing-braces) # Bug in clang
  endif()
endif()