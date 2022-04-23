MESSAGE( "--------------------------------------------- use_OpenRox --------------------------------------------" )
include( ExternalProject )

if ( CMAKE_EXTRA_GENERATOR )
  set( generator "${CMAKE_EXTRA_GENERATOR} - ${CMAKE_GENERATOR}" )
else()
  set( generator "${CMAKE_GENERATOR}" )
endif()

set ( external_project_dir ${PROJECT_BINARY_DIR}/external )

if ( CMAKE_BUILD_TYPE MATCHES "Debug" )
   set( USEROX_CONFIG_CMAKE_C_FLAG   ${CMAKE_C_FLAGS_DEBUG} )
   set( USEROX_CONFIG_CMAKE_CXX_FLAG ${CMAKE_CXX_FLAGS_DEBUG} )
else()
   if ( CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo" )
      set( USEROX_CONFIG_CMAKE_C_FLAG   ${CMAKE_C_FLAGS_RELWITHDEBINFO} )
      set( USEROX_CONFIG_CMAKE_CXX_FLAG ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} )
   else()
      #release by default
      set( USEROX_CONFIG_CMAKE_C_FLAG   ${CMAKE_C_FLAGS_RELEASE} )
      set( USEROX_CONFIG_CMAKE_CXX_FLAG ${CMAKE_CXX_FLAGS_RELEASE} )
endif()


# Send command from original project to OPENROX
if ( ${USE_OPENROX_SHARED} )
   set( OPENROX_BUILD_RELEASE ON )
   set( external_project_common_args
         -DCMAKE_INSTALL_PREFIX:PATH=${external_project_dir}
         -DBUILD_SHARED_LIBS:BOOL=ON
         -DROX_CREATE_SHARED_LIBRARIES:BOOL=ON
         -DCMAKE_C_FLAGS:STRING=${USEOPENROX_CONFIG_CMAKE_C_FLAG}
         -DCMAKE_CXX_FLAGS:STRING=${USEOPENROX_CONFIG_CMAKE_CXX_FLAG}
         -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
         -DROX_VERBOSE_DISPLAY:STRING=${USE_OPENROX_VERBOSE_DISPLAY}
         -DROX_VERBOSE_LOGFILE:STRING=${USE_OPENROX_VERBOSE_LOGFILE}
         #-DROX_HAS_VIEWER_SUPPORT:BOOL=TRUE
         -DROX_HAS_VIEWER_SUPPORT:BOOL=${USE_OPENROX_HAS_VIEWER_SUPPORT}
         -DROX_HAS_CAMERA_DISABLED:BOOL=${USE_OPENROX_HAS_CAMERA_DISABLED}
         #-DROX_HAS_CAMERA_SUPPORT:BOOL=${USE_OPENROX_HAS_CAMERA_SUPPORT}
         -DROX_CREATE_PLUGIN:BOOL=TRUE
         -DROX_LOGS:BOOL=TRUE
  )
else()
   set( OPENROX_BUILD_RELEASE OFF )
   set( external_project_common_args
         -DCMAKE_INSTALL_PREFIX:PATH=${external_project_dir}
         -DBUILD_SHARED_LIBS:BOOL=OFF
         -DROX_CREATE_SHARED_LIBRARIES:BOOL=OFF
         -DCMAKE_C_FLAGS:STRING=${USEOPENROX_CONFIG_CMAKE_C_FLAG}
         -DCMAKE_CXX_FLAGS:STRING=${USEOPENROX_CONFIG_CMAKE_CXX_FLAG}
         -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
         -DROX_VERBOSE_DISPLAY:STRING=${USE_OPENROX_VERBOSE_DISPLAY}
         -DROX_VERBOSE_LOGFILE:STRING=${USE_OPENROX_VERBOSE_LOGFILE}
         #-DROX_HAS_VIEWER_SUPPORT:BOOL=TRUE
         -DROX_HAS_VIEWER_SUPPORT:BOOL=${USE_OPENROX_HAS_VIEWER_SUPPORT}
         -DROX_HAS_CAMERA_DISABLED:BOOL=${USE_OPENROX_HAS_CAMERA_DISABLED}
         #-DROX_HAS_CAMERA_SUPPORT:BOOL=${USE_OPENROX_HAS_CAMERA_SUPPORT}
         -DROX_CREATE_PLUGIN:BOOL=TRUE
         -DROX_LOGS:BOOL=TRUE
endif()

# names that can be used by users
# the names are set by hand, if they change in openrox_lib.cmake they must be reset
set( OPENROX_TOPLEVEL_NAME      OPENROX )
set( OPENROX_LIBRARIES          openrox )
set( OPENROX_EXAMPLES_SOURCE_DIR $ENV{OPENROX_HOME}/examples )
message( "OPENROX_EXAMPLES_SOURCE_DIR : " ${OPENROX_EXAMPLES_SOURCE_DIR} )

set_property( DIRECTORY PROPERTY EP_BASE ${external_project_dir} )
ExternalProject_Add( ${OPENROX_TOPLEVEL_NAME}
                  SOURCE_DIR $ENV{OPENROX_HOME}
                  CMAKE_GENERATOR ${generator}
                  CMAKE_ARGS ${external_project_common_args}
                  )

ExternalProject_Add_Step( ${OPENROX_TOPLEVEL_NAME} forceconfigure
                COMMAND ${CMAKE_COMMAND} -E echo "Force configure of openrox"
                DEPENDEES update
                DEPENDERS configure
                ALWAYS 1
            )

#set BINARY_DIR in order to retrieve external binary dir value to set useful paths to local project
ExternalProject_Get_Property( ${OPENROX_TOPLEVEL_NAME} BINARY_DIR )
include_directories( ${PROJECT_BINARY_DIR}/external/inc )
include_directories( ${PROJECT_BINARY_DIR}/external/inc/generated )
include_directories( $ENV{OPENROX_HOME}/sources/ )
link_directories( ${BINARY_DIR}/ )

MESSAGE( "-----------------------------------------------------------------------------------------------------" )
