cmake_minimum_required(VERSION 2.8)
project(cepton_sdk_redist)

include("${PROJECT_SOURCE_DIR}/cmake/Redist.cmake")

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(PROGRAMS
  "bin/${OS_NAME}/CeptonViewer"
	DESTINATION .
)

set(CPACK_PACKAGE_NAME "cepton_sdk")
set(CPACK_PACKAGE_VENDOR "Cepton Technologies")
set(CPACK_PACKAGE_DESCRIPTION "Cepton SDK")
set(CPACK_PACKAGE_CONTACT "Cepton Technologies")

if(WIN32)
  set(CPACK_GENERATOR "WIX")
elseif(APPLE)
  set(CPACK_GENERATOR "DragNDrop")
elseif(UNIX)
  set(CPACK_GENERATOR "DEB")
endif()

include(CPack)