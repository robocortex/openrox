### Doxygen generation for library ####

#option(OPENROX_CREATE_MANUAL_PROG "Use Doxygen to create the HTML based API documentation" OFF)

if (OPENROX_CREATE_MANUAL_PROG)
   set(OLD_CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ${CMAKE_FIND_ROOT_PATH_MODE_PROGRAM})
   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
   

   #SET (OPENROX_NEEDS_DOXYGEN 1)
   #SET (OPENROX_NEEDS_LATEX 1)
   
   # find_package(Doxygen)
   # find_package(LATEX)

   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ${OLD_CMAKE_FIND_ROOT_PATH_MODE_PROGRAM})

   if (NOT DOXYGEN_FOUND)
      message(FATAL_ERROR "Doxygen is needed to build the documentation. Please install it correctly")
   endif()

   if (NOT PDFLATEX_COMPILER)
      message(FATAL_ERROR "PdfLatex is needed to build the documentation. Please install it correctly")
   endif()

   if (OPENROX_BUILD_RELEASE)

      message(STATUS "Build external release ${OPENROX_TARGET_RELEASE} prog manual in ${PROJECT_BINARY_DIR}")

      configure_file(${OPENROX_SOURCE_DIR}/manual/prog/Doxyfile_${OPENROX_TARGET_RELEASE}.in ${PROJECT_BINARY_DIR}/Doxyfile_${OPENROX_TARGET_RELEASE} @ONLY IMMEDIATE)

      add_custom_target (${OPENROX_TARGET_RELEASE}_prog_manual
         COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile_${OPENROX_TARGET_RELEASE}
         SOURCES ${PROJECT_BINARY_DIR}/Doxyfile_${OPENROX_TARGET_RELEASE}
      )

      add_custom_command(TARGET ${OPENROX_TARGET_RELEASE}_prog_manual
         COMMAND ${PDFLATEX_COMPILER} ${PROJECT_BINARY_DIR}/${OPENROX_TARGET_RELEASE}_prog_manual/latex/refman.tex
         WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${OPENROX_TARGET_RELEASE}_prog_manual/latex/
      )

      add_custom_command(TARGET ${OPENROX_TARGET_RELEASE}_prog_manual
         COMMAND ${PDFLATEX_COMPILER} ${PROJECT_BINARY_DIR}/${OPENROX_TARGET_RELEASE}_prog_manual/latex/refman.tex
         WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${OPENROX_TARGET_RELEASE}_prog_manual/latex/
      )

      add_custom_command (TARGET ${OPENROX_TARGET_RELEASE}_prog_manual
         COMMAND ${CMAKE_COMMAND} -E copy   ${PROJECT_BINARY_DIR}/${OPENROX_TARGET_RELEASE}_prog_manual/latex/refman.pdf  ${PROJECT_BINARY_DIR}/${OPENROX_TARGET_RELEASE}_prog_manual/${OPENROX_TARGET_RELEASE}_prog_manual.pdf
      )

   else ()

      message(STATUS "Build standard prog manual")

      configure_file(${OPENROX_SOURCE_DIR}/manual/prog/Doxyfile_openrox.in ${PROJECT_BINARY_DIR}/Doxyfile_openrox @ONLY IMMEDIATE)
      add_custom_target (openrox_prog_manual
         COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile_openrox
         SOURCES ${PROJECT_BINARY_DIR}/Doxyfile_openrox
      )
   endif()
endif()
