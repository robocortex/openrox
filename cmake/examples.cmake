set(OPENROX_EXAMPLES_EXTERNAL_LIBS openrox ${ROX_LICENCE_LIBS} ${OPENROX_PLUGIN_NAME})
set(OPENROX_EXAMPLES_SOURCE_DIR ${OPENROX_SOURCE_DIR}/examples)

MESSAGE(STATUS "\n_____ rox_open/cmake/examples.cmake ___________________________________________\n" )

if ( OPENROX_BUILD_EXAMPLES )

   # Define folder with standard data for examples
   # ADD_DEFINITIONS( -DROX_DATA_HOME="$ENV{OPENROX_DATA_HOME}" )

   set(OPENROX_EXAMPLES_SOURCE_DIR ${OPENROX_SOURCE_DIR}/examples/)

   MESSAGE(STATUS "\n_____ Build examples ___________________________________________\n" )

   # Release examples

   add_demo_macro(rox_example_camera_calibration                        C)
   add_demo_macro(rox_example_identification                            C)
   add_demo_macro(rox_example_identification_database_cloud             C)
   add_demo_macro(rox_example_identification_database_compiled          C)
   add_demo_macro(rox_example_image_score                               C)
   add_demo_macro(rox_example_image_undistortion                        C)
   add_demo_macro(rox_example_motion_detection                          C)
   add_demo_macro(rox_example_odometry_database_features                C)
   add_demo_macro(rox_example_odometry_inertial_observer                C)
   add_demo_macro(rox_example_odometry_multi_plane                      C)
   add_demo_macro(rox_example_odometry_single_plane                     C)
   add_demo_macro(rox_example_odometry_single_plane_filter_matse3       C)
   add_demo_macro(rox_example_odometry_single_plane_database            C)
   add_demo_macro(rox_example_odometry_single_plane_photoframe          C)
   add_demo_macro(rox_example_plane_detection_from_rectangles           C)
   add_demo_macro(rox_example_rectangle_detection                       C)
   add_demo_macro(rox_example_tracking                                  C)
   add_demo_macro(rox_example_tracking_photoframe                       C)
       
   # add_demo_macro(rox_example_database_generation                     C)
   # add_demo_macro(rox_example_odometry_database_compiled              C)

   # Simple examples

   add_demo_macro(rox_example_vvs_se3_so3z                              C)
   add_demo_macro(rox_example_vvs_se3_so3z_so3z                         C)
   add_demo_macro(rox_example_se3_so3z_so3z_init_and_vvs                C)
   add_demo_macro(rox_example_cam2proj                                  C)
   add_demo_macro(rox_example_projector_calibration                     C)
   add_demo_macro(rox_example_camera_projector_calibration              C)
   add_demo_macro(rox_example_database_generation                       C)
   add_demo_macro(rox_example_odometry_database_compiled                C)
   add_demo_macro(rox_example_detection_server                          C)
   add_demo_macro(rox_example_camera_projector_calibration_mocked       C)
   add_demo_macro(rox_example_prediction_matso3                         C)

   # Simple CPP examples

   add_demo_macro ( rox_example_create_virtual_sequence                 CPP)

   MESSAGE(STATUS "\n_____ Following examples need external format cao ___________________________________________\n" )

   # needs rox_caofile_edges
   add_demo_macro ( rox_example_mbo_odometry_nodependency               C)
   add_demo_macro ( rox_example_mbo_odometry_ellipses_nodependency      C)
   add_demo_macro ( rox_example_mbo_odometry_cylinders_nodependency     C)
   add_demo_macro ( rox_example_mbo_odometry_segments_nodependency      C) 

   unset ( OPENROX_EXAMPLE_EXTERNALS_SOURCES )

endif ()
