#INSTALL INFO FOR ALL IOS PLATFORM

include (InstallRequiredSystemLibraries)

install(TARGETS openrox
   LIBRARY DESTINATION bin
   ARCHIVE DESTINATION lib
)

if (OPENROX_BUILD_RELEASE)

   install(FILES ${OPENROX_BINARY_DIR}/generated/openrox.h DESTINATION inc)
   install(FILES ${OPENROX_RELEASE_DIR}/common/readme_license.txt DESTINATION lic)
   install(FILES ${OPENROX_RELEASE_DIR}/common/readme_res.txt DESTINATION res)
   install(FILES ${OPENROX_BINARY_DIR}/openrox_prog_manual/openrox_prog_manual.pdf DESTINATION doc)
   install(FILES ${OPENROX_BINARY_DIR}/openrox_user_manual/openrox_user_manual.pdf DESTINATION doc)
   install(DIRECTORY ${OPENROX_RELEASE_DIR}/common/minimal_seq/ DESTINATION seq FILES_MATCHING PATTERN "*.*")
   install(DIRECTORY ${OPENROX_RELEASE_DIR}/mobile/ios/openrox/sources/ DESTINATION src FILES_MATCHING PATTERN "*.*")

   set(CPACK_GENERATOR "ZIP")
   set(CPACK_PACKAGE_FILE_NAME "openrox.v${OPENROX_VERSION}.ios")

else()

   install(DIRECTORY ${OPENROX_SOURCE_DIR}/sources/
      DESTINATION include
      FILES_MATCHING PATTERN "*.h"
   )
   install(DIRECTORY ${OPENROX_BINARY_DIR}/generated/
      DESTINATION include/generated
      FILES_MATCHING PATTERN "*.h"
   )

   set(CPACK_GENERATOR "ZIP")
   set(CPACK_PACKAGE_FILE_NAME "openrox.v${OPENROX_VERSION}.ios")

endif()
