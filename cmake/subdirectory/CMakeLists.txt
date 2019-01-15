#[[
CMake scoping project for setting subdirectory options without changing parent.
]]
project(cepton_subdirectory_${add_external_subdirectory_name})

set(CMAKE_BUILD_TYPE "Release")

# Disable all warnings
if(GCC)
  add_flags("-w")
elseif(CLANG)
  add_flags("-Wno-everything")
elseif(MSVC)
  add_flags("/W0")
endif()

add_subdirectory(${add_external_subdirectory_args})
