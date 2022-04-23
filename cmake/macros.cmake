MESSAGE(STATUS "\n_____ rox_open/cmake/macros.cmake ____________________________________________\n" )

list(APPEND CMAKE_MODULE_PATH "${OPENROX_SOURCE_DIR}/cmake/CMakeModules/CSharp")

macro(debug msg)
  if(OPENROX_DEBUG)
    MESSAGE(STATUS ${msg})
  endif()
endmacro()

macro(replace_platform_optimization liste)

  list(LENGTH ${liste} llist)

  foreach (iditem RANGE 1 ${llist})

    list(GET ${liste} 0 item)
    list(REMOVE_AT ${liste} 0)

    if (OPENROX_USE_MKL)
      string(REPLACE "?mkl?" "_mkl" item ${item})
    else()
      string(REPLACE "?mkl?" "" item ${item})
    endif()

    # Replace function names depending on the available optimization architecture
    if      (OPENROX_USE_AVX)
      string(REPLACE "?sse?"          "_sse"  item  ${item})
      string(REPLACE "?avx?"          "_avx"  item  ${item})
      string(REPLACE "?neon?"         ""      item  ${item})
      string(REPLACE "?sse,neon?"     "_sse"  item  ${item})
      string(REPLACE "?avx,neon?"     "_avx"  item  ${item})
      string(REPLACE "?sse,avx?"      "_avx"  item  ${item})
      string(REPLACE "?sse,avx,neon?" "_avx"  item  ${item})
    elseif  (OPENROX_USE_SSE)
      string(REPLACE "?sse?"          "_sse"  item  ${item})
      string(REPLACE "?avx?"          ""      item  ${item})
      string(REPLACE "?neon?"         ""      item  ${item})
      string(REPLACE "?sse,neon?"     "_sse"  item  ${item})
      string(REPLACE "?avx,neon?"     ""      item  ${item})
      string(REPLACE "?sse,avx?"      "_sse"  item  ${item})
      string(REPLACE "?sse,avx,neon?" "_sse"  item  ${item})
    elseif  (OPENROX_USE_NEON)
      string(REPLACE "?sse?"          ""      item  ${item})
      string(REPLACE "?avx?"          ""      item  ${item})
      string(REPLACE "?neon?"         "_neon" item  ${item})
      string(REPLACE "?sse,neon?"     "_neon" item  ${item})
      string(REPLACE "?avx,neon?"     "_neon" item  ${item})
      string(REPLACE "?sse,avx?"      ""      item  ${item})
      string(REPLACE "?sse,avx,neon?" "_neon" item  ${item})
    else ()
      string(REPLACE "?avx?"          ""      item  ${item})
      string(REPLACE "?sse?"          ""      item  ${item})
      string(REPLACE "?neon?"         ""      item  ${item})
      string(REPLACE "?sse,neon?"     ""      item  ${item})
      string(REPLACE "?avx,neon?"     ""      item  ${item})
      string(REPLACE "?sse,avx?"      ""      item  ${item})
      string(REPLACE "?sse,avx,neon?" ""      item  ${item})
    endif ()

    list(APPEND ${liste} ${item})

  endforeach ()

endmacro()

# Macro used to define C or C++ demo executable with specific options
# Required arguments are:
# name: Demo filename
# cxxType : Demo type (C or CPP)
#
# You need to define following variables:
# OPENROX_EXAMPLES_SOURCE_DIR : Demo source directory
# OPENROX_EXAMPLES_EXTERNAL_LIBS : Additional libraries used for this demo
# OPENROX_EXAMPLES_EXTERNAL_SOURCES : Additional sources used for this demo
#
macro(add_demo_macro name cxxType)
   if(${cxxType} MATCHES "CPP")
      set(sourcename ${OPENROX_EXAMPLES_SOURCE_DIR}/${name}.cpp)
   elseif(${cxxType} MATCHES "C")
      set(sourcename ${OPENROX_EXAMPLES_SOURCE_DIR}/${name}.c)
   else()
      message("unknown type :" ${cxxType})
   endif()

    unset(add_sources)

   #those previous dependencies needs pthread on linux platform
   if(OPENROX_IS_LINUX)
      #portable way to specify pthread dependency(see cmake 3.1 and above)
      set(THREADS_PREFER_PTHREAD_FLAG ON)
      find_package(Threads REQUIRED)
      list(APPEND OPENROX_EXAMPLES_EXTERNAL_LIBS "Threads::Threads")
   endif()

   include_directories(${OPENROX_EXAMPLES_SOURCE_DIR})

   # Create demo executable
   add_executable(${name} ${sourcename} ${OPENROX_EXAMPLES_EXTERNAL_SOURCES})
   set_target_properties (${name} PROPERTIES FOLDER examples)

   # Search for dependencies into external libs
   foreach(it ${OPENROX_EXAMPLES_EXTERNAL_LIBS})
      if(TARGET ${it})
        add_dependencies(${name} ${it})
      endif()
   endforeach()

   message("${name} ${OPENROX_EXAMPLES_EXTERNAL_LIBS}")

   target_link_libraries(${name} ${OPENROX_EXAMPLES_EXTERNAL_LIBS})
endmacro()

# Macro used to create unit test from a folder using a specific name
# You need to define following variables:
# TESTS_SOURCES_DIR : Tests source directory
# TESTS_EXTERNAL_LIBS : Additional libraries used for this test
# TESTS_EXTERNAL_SOURCES : Additional sources used for this test
macro( unit_test_macro folder name)
   if(OPENROX_TESTS_OK)
     
       message("Generating c++ test for: " OPENROX "//folder: " ${folder} "//name: " ${name}                      )
       include_directories  ( ${TESTS_SOURCES_DIR} "${OPENROX_SOURCE_DIR}/tests"                              )

       if(ANDROID OR OPENROX_USE_TESTS_RUNNER)
         #include_directories(${ANDROID_STL_INCLUDE_DIRS})
         add_library( ${name} SHARED ${TESTS_SOURCES_DIR}/${folder}/${name}.cpp ${TESTS_EXTERNAL_SOURCES} "${OPENROX_SOURCE_DIR}/tests/openrox_tests.cpp")
       else()
         add_executable( ${name} ${TESTS_SOURCES_DIR}/${folder}/${name}.cpp ${TESTS_EXTERNAL_SOURCES} "${OPENROX_SOURCE_DIR}/tests/openrox_tests.cpp")
       endif()
       target_link_libraries( ${name} ${TESTS_EXTERNAL_LIBS}                                                          )
       install_target(${name})
       add_test             ( ${name} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${name} )
       file                 ( GLOB REFERENCE_FILES ${TESTS_SOURCES_DIR}/${folder}/references/*                        )
       file                 ( COPY ${REFERENCE_FILES} DESTINATION ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/references  )
       file                 ( COPY ${REFERENCE_FILES} DESTINATION ${CMAKE_BINARY_DIR}/references                      )

      set_target_properties (${name} PROPERTIES FOLDER Tests)
   endif()
endmacro(unit_test_macro)

# Macro used to check if a file referenced by a variable exists or not
macro( set_and_check _var _file )
  set( ${_var} "${_file}" )
  if ( NOT EXISTS "${_file}" )
    message( FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !" )
  endif()
endmacro()

# Macro used to install library into the CMAKE_BINARY_DIR directory
# (required by shared library if compiled using add_subdirectory)
macro(install_target target)
  if(NOT ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
      set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release)
      set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/RelWithDebInfo)
      set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug)
      set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release)
      set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug)
      set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/RelWithDebInfo)
  endif()
endmacro()