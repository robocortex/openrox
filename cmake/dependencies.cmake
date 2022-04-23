MESSAGE(STATUS "\n_____ rox_open/cmake/dependencies.cmake ____________________________________________\n" )

MESSAGE(STATUS "--------------------- OPENROX dependencies: optimized linear algebra ------------------")

if (OPENROX_CREATE_PLATFORM_OPTIMIZED_LINEAR_ALGEBRA AND NOT OPENROX_BUILD_RELEASE)
   find_package(MKL QUIET)

   if(MKL_FOUND)
      include_directories(${MKL_INCLUDE_DIRS})
      set(OPENROX_LINEAR_ALGEBRA_LIBS ${MKL_MINIMAL_LIBRARY})
      set(OPENROX_USE_MKL true)
      MESSAGE("OPENROX_USE_MKL = " ${OPENROX_USE_MKL})
   endif()
endif()

MESSAGE(STATUS "--------------------- OPENROX dependencies: latex and doxygen ------------------")

if (OPENROX_CREATE_MANUAL_PROG)
   SET (OPENROX_NEEDS_DOXYGEN 1)
   SET (OPENROX_NEEDS_LATEX 1)
endif()

if (OPENROX_CREATE_MANUAL_USER)
   SET (OPENROX_NEEDS_LATEX 1)
endif()

if (OPENROX_NEEDS_DOXYGEN)
   find_package(Doxygen)
endif()

if (OPENROX_NEEDS_LATEX)
   find_package(LATEX)
endif()
