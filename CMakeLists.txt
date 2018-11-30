#[[
CMake file for including cepton_sdk in a CMake project.

To use, add the following to your CMake

```
add_subdirectory(<path_to_cepton_sdk>)
include_directories(SYSTEM ${CEPTON_SDK_INCLUDE_DIRS})
add_definitions(${CEPTON_SDK_DEFINITIONS})
target_link_libraries(<target> ${CEPTON_SDK_LIBRARIES})
```
]]
cmake_minimum_required(VERSION 2.8)
project(cepton_sdk)

include("${PROJECT_SOURCE_DIR}/cmake/CommonRedist.cmake")

create_option(BOOL CEPTON_SDK_USE_STATIC TRUE
    "Use static cepton sdk libraries.")

set(CEPTON_SDK_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include" PARENT_SCOPE)
if(CEPTON_SDK_USE_STATIC)
    CEPTON_GET_STATIC_LIBRARY(CEPTON_SDK_LIBRARIES "${PROJECT_SOURCE_DIR}" "cepton_sdk")
    list(APPEND CEPTON_SDK_DEFINITIONS -DCEPTON_SDK_STATIC)
else()
    CEPTON_GET_SHARED_LIBRARY(CEPTON_SDK_LIBRARIES "${PROJECT_SOURCE_DIR}" "cepton_sdk")
endif()
set(CEPTON_SDK_DEFINITIONS ${CEPTON_SDK_DEFINITIONS} PARENT_SCOPE)
set(CEPTON_SDK_LIBRARIES ${CEPTON_SDK_LIBRARIES} PARENT_SCOPE)