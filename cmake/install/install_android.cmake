#INSTALL INFO FOR ALL ANDROID PLATFORM

include(InstallRequiredSystemLibraries)

install(TARGETS openrox
   LIBRARY DESTINATION bin
   ARCHIVE DESTINATION lib
)

if (OPENROX_BUILD_RELEASE)

   install(FILES ${OPENROX_BINARY_DIR}/openrox_prog_manual/openrox_prog_manual.pdf DESTINATION doc)
   install(FILES ${OPENROX_BINARY_DIR}/openrox_user_manual/openrox_user_manual.pdf DESTINATION doc)

   install(FILES ${OPENROX_BINARY_DIR}/generated/openrox.h DESTINATION inc)
   install(FILES ${OPENROX_RELEASE_DIR}/common/readme_license.txt DESTINATION lic)
   install(FILES ${OPENROX_RELEASE_DIR}/common/readme_res.txt DESTINATION res)

   install(DIRECTORY ${OPENROX_RELEASE_DIR}/common/minimal_seq/ DESTINATION seq FILES_MATCHING PATTERN "*.*")
   install(DIRECTORY ${OPENROX_RELEASE_DIR}/mobile/android/openrox/ DESTINATION src FILES_MATCHING PATTERN "*.*")
   install(FILES ${OPENROX_BINARY_DIR}/generated/openrox.h DESTINATION src/tracking/jni)
   install(FILES ${OPENROX_BINARY_DIR}/generated/openrox.h DESTINATION src/model3d/jni)
   install(FILES ${OPENROX_BINARY_DIR}/generated/openrox.h DESTINATION src/database/jni)

   set(CPACK_GENERATOR "ZIP")
   set(CPACK_PACKAGE_FILE_NAME "openrox.v${OPENROX_VERSION}.and")

else()

   INSTALL(DIRECTORY ${OPENROX_SOURCE_DIR}/sources/
       DESTINATION include
       FILES_MATCHING PATTERN "*.h"
   )
   INSTALL(DIRECTORY ${OPENROX_BINARY_DIR}/generated/
       DESTINATION include/generated
       FILES_MATCHING PATTERN "*.h"
   )

   set(CPACK_GENERATOR "ZIP")
   set(CPACK_PACKAGE_FILE_NAME "openrox.v${OPENROX_VERSION}.and")

endif()
