#[[
General CMake options for all redist cepton repositories.
]]
set(CEPTON_SDK_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../")
include("${CEPTON_SDK_SOURCE_DIR}/cmake/Common.cmake")

macro(CEPTON_GET_SHARED_LIBRARY key root lib_name)
    if(WINDOWS)
        set(${key} "${root}/bin/${OS_NAME}/${lib_name}.dll")
    elseif(APPLE)
        set(${key} "${root}/lib/${OS_NAME}/${lib_name}.dylib")
    elseif(LINUX)
        set(${key} "${root}/lib/${OS_NAME}/${lib_name}.so")
    endif()
endmacro()

macro(CEPTON_GET_STATIC_LIBRARY key root lib_name)
    if(WINDOWS)
        set(${key} "${root}/lib/${OS_NAME}/${lib_name}.lib")
    elseif(APPLE OR LINUX)
        set(${key} "${root}/lib/${OS_NAME}/${lib_name}.a")
    endif()
endmacro()