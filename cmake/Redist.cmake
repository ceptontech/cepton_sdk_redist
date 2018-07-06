if(WIN32)
  set(OS_NAME "win64")
elseif(APPLE)
  set(OS_NAME "osx")
elseif(UNIX)
  execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
  if("${MACHINE_ARCH}" MATCHES "^armv")
    set(OS_NAME "linux-arm")
  else()
    set(OS_NAME "linux-${MACHINE_ARCH}")
  endif()
else()
  message(FATAL_ERROR "Unsupported OS!")
endif()