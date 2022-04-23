#Universal Windows Project toolchain

# Check cmake is enough recent to handle Universal Windows Projects
if( CMAKE_VERSION VERSION_LESS "3.4.0" )
   MESSAGE("Your cmake version is not recent enough to handle Universal Windows Projects.")
   MESSAGE("Please download the last version or use the manual steps described in this page: https://msdn.microsoft.com/en-US/library/mt186162.aspx.")
endif()

SET (CMAKE_SYSTEM_NAME WindowsStore)
SET (CMAKE_SYSTEM_VERSION 10.0)
SET (WINDOWSSTORE 1)