MESSAGE(STATUS "\n_____ rox_open/cmake/install.cmake ____________________________________________\n" )

### INSTALL SCRIPTS ####
set(OPENROX_RELEASE_DIR ${OPENROX_SOURCE_DIR}/release/)

#Per platform install
if(OPENROX_IS_IOS)
   # IOS
   include(${OPENROX_SOURCE_DIR}/install/install_ios.cmake)
elseif(OPENROX_IS_ANDROID)
   # Android
   include(${OPENROX_CMAKE_DIR}/install/install_android.cmake)
elseif(OPENROX_IS_MACOSX)
   # MacOSX
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_LINUX)
   # Linux
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_WINDOWS AND MSVC10)
   # Windows
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_WINDOWS AND MSVC11)
   # Windows
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_WINDOWS AND MSVC12)
   # Windows
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_WINDOWS AND MSVC14)
   # Windows
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_WINDOWS AND MSVC16)
   # Windows
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
elseif(OPENROX_IS_UWP)
   # Windows
   include (${OPENROX_CMAKE_DIR}/install/install_desktop.cmake)
   # Unknown platform
else()
   message(FATAL_ERROR "Unknown platform for installation")
endif()

include(CPack)

