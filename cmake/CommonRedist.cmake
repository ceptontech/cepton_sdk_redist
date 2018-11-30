#[[
General CMake options for all redist cepton repositories.
]]
include("${CMAKE_CURRENT_LIST_DIR}/Common.cmake")

macro(CEPTON_GET_SHARED_LIBRARY key root lib_name)
    if(WINDOWS)
        set(${key} "${root}/bin/${OS_NAME}/${lib_name}.dll")
    elseif(APPLE)
        set(${key} "${root}/lib/${OS_NAME}/${lib_name}.dylib")
    elseif(LINUX)
        set(${key} "${root}/lib/${OS_NAME}/lib${lib_name}.so")
    endif()
endmacro()

macro(CEPTON_GET_STATIC_LIBRARY key root lib_name)
    if(WINDOWS)
        set(${key} "${root}/lib/${OS_NAME}/${lib_name}.lib")
    elseif(APPLE OR LINUX)
        set(${key} "${root}/lib/${OS_NAME}/lib${lib_name}.a")
    endif()
endmacro()