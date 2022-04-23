MESSAGE(STATUS "\n_____ rox_open/cmake/install_desktop.cmake ____________________________________________\n" )

# INSTALL INFO FOR ALL DESKTOP PLATFORM

include (InstallRequiredSystemLibraries)

set(ROX_DESKTOP_EXAMPLES
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_single_plane.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_single_plane_filter_matse3.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_single_plane_database.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_database_features.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_database_compiled.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_multi_plane.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_camera_calibration.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_database_generation.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_identification_database_cloud.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_image_undistortion.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_inertial_observer.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_odometry_single_plane_photoframe.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_image_score.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_tracking.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_tracking_photoframe.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_motion_detection.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_rectangle_detection.c
   ${OPENROX_SOURCE_DIR}/examples/rox_example_plane_detection_from_rectangles.c
)

INSTALL(TARGETS openrox
   RUNTIME DESTINATION bin
   LIBRARY DESTINATION bin
   ARCHIVE DESTINATION lib
)

if(OPENROX_BUILD_RELEASE)

   install(DIRECTORY ${OPENROX_SOURCE_DIR}/sources/                                                   DESTINATION inc/           FILES_MATCHING PATTERN "*.h")
   install(DIRECTORY ${OPENROX_BINARY_DIR}/generated/                                                 DESTINATION inc/generated/ FILES_MATCHING PATTERN "*.h")

   install(FILES ${OPENROX_SOURCE_DIR}/LICENSE.COMMERCIAL                                             DESTINATION . )
   install(FILES ${OPENROX_SOURCE_DIR}/LICENSE.GPL                                                    DESTINATION . )
   install(FILES ${OPENROX_SOURCE_DIR}/LICENSE.LGPL                                                   DESTINATION . )

   # install(FILES ${OPENROX_BINARY_DIR}/generated/openrox.h                                          DESTINATION inc)
   install(FILES ${OPENROX_BINARY_DIR}/openrox_prog_manual/openrox_prog_manual.pdf                    DESTINATION doc)
   install(FILES ${OPENROX_BINARY_DIR}/openrox_user_manual/openrox_user_manual.pdf                    DESTINATION doc)
   install(FILES ${ROX_DESKTOP_EXAMPLES}                                                              DESTINATION src)
   install(FILES ${OPENROX_RELEASE_DIR}/desktop/openrox/cmake/CMakeLists.txt                          DESTINATION .)
   install(FILES ${OPENROX_RELEASE_DIR}/common/readme_res.txt                                         DESTINATION res)
   install(DIRECTORY ${OPENROX_RELEASE_DIR}/common/minimal_seq/                                       DESTINATION seq FILES_MATCHING PATTERN "*.*")

   SET (CPACK_GENERATOR "ZIP")
   SET (CPACK_PACKAGE_FILE_NAME "openrox.v${OPENROX_VERSION}.${CMAKE_SYSTEM_NAME}")

else()

   install(DIRECTORY ${OPENROX_SOURCE_DIR}/sources/                                                   DESTINATION inc            FILES_MATCHING PATTERN "*.h")
   install(DIRECTORY ${OPENROX_BINARY_DIR}/generated/                                                 DESTINATION inc/generated  FILES_MATCHING PATTERN "*.h")

   install(FILES ${OPENROX_SOURCE_DIR}/LICENSE.COMMERCIAL                                         DESTINATION . )
   install(FILES ${OPENROX_SOURCE_DIR}/LICENSE.GPL                                                DESTINATION . )
   install(FILES ${OPENROX_SOURCE_DIR}/LICENSE.LGPL                                               DESTINATION . )

   set(CPACK_GENERATOR "ZIP")
   set(CPACK_PACKAGE_FILE_NAME "openrox.v${OPENROX_VERSION}.${CMAKE_SYSTEM_NAME}")

endif()
