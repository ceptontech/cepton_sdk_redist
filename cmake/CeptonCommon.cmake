#[[
General CMake options for all cepton repositories.
]]
get_filename_component(CEPTON_SDK_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../"
                       ABSOLUTE)

set(CEPTON_DEFINITIONS "")
set(CEPTON_FLAGS "")

# ------------------------------------------------------------------------------
# Platform
# ------------------------------------------------------------------------------
# Detect endianness
include(TestBigEndian)
test_big_endian(IS_BIG_ENDIAN)
if(IS_BIG_ENDIAN)
  message(FATAL_ERROR "Big endian is not supported!")
endif()

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
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SIZEOF_VOID_P}!")
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
  message(FATAL_ERROR "Unsupported OS")
endif()
if(NOT DEFINED WINDOWS)
  set(WINDOWS FALSE)
endif()
if(NOT DEFINED LINUX)
  set(LINUX FALSE)
endif()

# Detect OS name
if(WINDOWS)
  set(OS_NAME "win64")
elseif(APPLE)
  set(OS_NAME "osx")
elseif(LINUX)
  set(OS_NAME "linux-${CMAKE_SYSTEM_PROCESSOR}")
  if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64")
    set(DEBIAN_ARCHITECTURE "amd64")
  elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64")
    set(DEBIAN_ARCHITECTURE "arm64")
  elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
    set(DEBIAN_ARCHITECTURE "armhf")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
  endif()
endif()
list(APPEND CEPTON_DEFINITIONS OS_NAME="${OS_NAME}")

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
  message(FATAL_ERROR "Unsupported compiler: ${CMAKE_CXX_COMPILER_ID}!")
endif()

# ------------------------------------------------------------------------------
# Macros
# ------------------------------------------------------------------------------
# Print list with newlines
macro(MESSAGE_LIST l)
  foreach(line
          IN
          LISTS
          ${l})
    message("${line}")
  endforeach()
endmacro()

# Append to string
macro(STRING_APPEND result other)
  set(${result} "${${result}} ${other}")
endmacro()

# Join list
macro(STRING_JOIN result delimiter)
  string(REPLACE ";"
                 "${delimiter}"
                 ${result}
                 "${ARGN}")
endmacro()

# Expand path
macro(FIX_PATH result input)
  get_filename_component(${result} "${input}" ABSOLUTE)
endmacro()

# Evaluate logical expression
macro(LOGICAL result predicate)
  # Split into list
  string(REGEX
         REPLACE " +"
                 ";"
                 logical_p
                 "${predicate}")
  if(${logical_p})
    set(${result} TRUE)
  else()
    set(${result} FALSE)
  endif()
endmacro()

# Add cmake option
macro(CREATE_OPTION
      type
      key
      init_value
      docstring)
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
macro(ADD_C_FLAGS)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_C_FLAGS ${flags})
endmacro()

macro(ADD_CXX_FLAGS)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_CXX_FLAGS ${flags})
endmacro()

macro(ADD_FLAGS)
  add_c_flags(${ARGN})
  add_cxx_flags(${ARGN})
endmacro()

macro(ADD_FLAGS_RELEASE flags)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_C_FLAGS_RELEASE ${flags})
  string_append(CMAKE_CXX_FLAGS_RELEASE ${flags})
endmacro()

macro(ADD_LINKER_FLAGS)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_EXE_LINKER_FLAGS ${flags})
endmacro()

macro(ADD_LINKER_FLAGS_RELEASE)
  string_join(flags " " ${ARGN})
  string_append(CMAKE_EXE_LINKER_FLAGS_RELEASE ${flags})
endmacro()

if(GCC)
  set(DISABLE_WARNINGS_FLAGS -w)
elseif(CLANG)
  set(DISABLE_WARNINGS_FLAGS -Wno-everything)
elseif(MSVC)
  set(DISABLE_WARNINGS_FLAGS /W0)
endif()

# Add subdirectory
macro(ADD_EXTERNAL_SUBDIRECTORY source_dir)
  message(
    STATUS
      "================================================================================"
    )
  get_filename_component(add_external_subdirectory_name "${source_dir}" NAME)
  message(STATUS "External: " ${add_external_subdirectory_name})
  message(
    STATUS
      "--------------------------------------------------------------------------------"
    )
  set(add_external_subdirectory_args "${source_dir}" ${ARGN})
  add_subdirectory(
    "${CEPTON_SDK_SOURCE_DIR}/cmake/subdirectory"
    "${PROJECT_BINARY_DIR}/third_party/cepton_subdirectory_${add_external_subdirectory_name}"
    EXCLUDE_FROM_ALL)
  message(
    STATUS
      "--------------------------------------------------------------------------------"
    )
endmacro()

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release")
endif()

# ------------------------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)

# ------------------------------------------------------------------------------
# Libraries
# ------------------------------------------------------------------------------
if(WINDOWS)
  set(CEPTON_SHARED_LIBRARY_EXTENSION ".dll")
  set(CEPTON_STATIC_LIBRARY_EXTENSION ".lib")
elseif(APPLE)
  set(CEPTON_SHARED_LIBRARY_EXTENSION ".dylib")
  set(CEPTON_STATIC_LIBRARY_EXTENSION ".a")
elseif(LINUX)
  set(CEPTON_SHARED_LIBRARY_EXTENSION ".so")
  set(CEPTON_STATIC_LIBRARY_EXTENSION ".a")
endif()

macro(CEPTON_GET_SHARED_LIBRARY
      result
      root
      lib_name)
  if(WINDOWS)
    set(${result} "${root}/bin/${OS_NAME}/${lib_name}")
  elseif(APPLE)
    set(${result} "${root}/lib/${OS_NAME}/${lib_name}")
  elseif(LINUX)
    set(${result} "${root}/lib/${OS_NAME}/lib${lib_name}")
  endif()
endmacro()

macro(CEPTON_GET_STATIC_LIBRARY
      result
      root
      lib_name)
  if(WINDOWS)
    set(${result} "${root}/lib/${OS_NAME}/${lib_name}")
  elseif(APPLE OR LINUX)
    set(${result} "${root}/lib/${OS_NAME}/lib${lib_name}")
  endif()
endmacro()

macro(CEPTON_IMPORT_SHARED_LIBRARY
      lib
      root
      lib_name)
  cepton_get_shared_library(cepton_import_shared_library_path "${root}"
                            "${lib_name}")
  set_target_properties(
    ${lib}
    PROPERTIES
      IMPORTED_LOCATION
      "${cepton_import_shared_library_path}${CEPTON_SHARED_LIBRARY_EXTENSION}")
  if(WINDOWS)
    set_target_properties(
      ${lib}
      PROPERTIES IMPORTED_IMPLIB "${cepton_import_shared_library_path}.imp.lib")
  endif()
endmacro()

macro(CEPTON_IMPORT_STATIC_LIBRARY
      lib
      root
      lib_name)
  cepton_get_static_library(cepton_import_static_library_path "${root}"
                            "${lib_name}")
  set_target_properties(
    ${lib}
    PROPERTIES
      IMPORTED_LOCATION
      "${cepton_import_static_library_path}${CEPTON_STATIC_LIBRARY_EXTENSION}")
endmacro()
