cmake_minimum_required(VERSION 2.8)
project(cepton_sdk_redist)

include("${PROJECT_SOURCE_DIR}/cmake/Redist.cmake")

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
if(WIN32)
    set(EXE_EXTENSION ".exe")
else()
    set(EXE_EXTENSION "")
endif()
install(PROGRAMS
    "bin/${OS_NAME}/CeptonViewer${EXE_EXTENSION}"
	DESTINATION .
)

set(CPACK_PACKAGE_NAME "CeptonViewer")
set(CPACK_PACKAGE_VENDOR "Cepton Technologies")
set(CPACK_PACKAGE_VERSION_MAJOR 1)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "${CPACK_PACKAGE_NAME}")
set(CPACK_PACKAGE_DESCRIPTION "Cepton LiDAR Viewer")
set(CPACK_PACKAGE_CONTACT "Cepton Technologies")

if(WIN32)
  set(CPACK_GENERATOR "WIX")
elseif(APPLE)
  set(CPACK_GENERATOR "DragNDrop")
elseif(UNIX)
  set(CPACK_GENERATOR "DEB")
endif()

include(CPack)
