#[[
CMake scoping project for setting subdirectory options without changing parent.
]]
project(cepton_subdirectory_${add_external_subdirectory_name})

if(CMAKE_BUILD_TYPE_OPTION STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "Release")
else()
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

if(GCC)
  add_flags("-w")
endif()

add_subdirectory(${add_external_subdirectory_args})
