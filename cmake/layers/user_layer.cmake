#USER LAYER DEFINITION

SET (USER_LAYER_SOURCES_DIR ${OPENROX_CODE_DIR}/user)

SET (USER_LAYER_CALIBRATION_SOURCES
   ${USER_LAYER_SOURCES_DIR}/calibration/mono/camera_calibration.c
   ${USER_LAYER_SOURCES_DIR}/calibration/mono/camera_calibration_checkerboard.c
   ${USER_LAYER_SOURCES_DIR}/calibration/stereo/calibration_stereo_checkerboard.c
   ${USER_LAYER_SOURCES_DIR}/calibration/stereo/calibration_stereo_checkerboard_se3.c
   ${USER_LAYER_SOURCES_DIR}/calibration/projector/calibration_projector_checkerboard.c
   ${USER_LAYER_SOURCES_DIR}/calibration/camproj/calibration_camproj_checkerboard.c
)

SET (USER_LAYER_IDENTIFICATION_SOURCES
   ${USER_LAYER_SOURCES_DIR}/identification/texture/identification.c
   ${USER_LAYER_SOURCES_DIR}/identification/texture/ident_texture_sl3.c
   ${USER_LAYER_SOURCES_DIR}/identification/texture/ident_texture_se3.c
   ${USER_LAYER_SOURCES_DIR}/identification/texture/multiident_se3.c
   ${USER_LAYER_SOURCES_DIR}/identification/texture/multiident_sl3.c
   ${USER_LAYER_SOURCES_DIR}/identification/texture/ident_multiplane.c

   ${USER_LAYER_SOURCES_DIR}/identification/photoframe/ident_photoframe_sl3.c
   ${USER_LAYER_SOURCES_DIR}/identification/photoframe/ident_photoframe_se3.c
   ${USER_LAYER_SOURCES_DIR}/identification/photoframe/ident_multi_photoframe_se3.c

   ${USER_LAYER_SOURCES_DIR}/identification/database/database_item.c
   ${USER_LAYER_SOURCES_DIR}/identification/database/database.c
   ${USER_LAYER_SOURCES_DIR}/identification/database/database_features.c
   ${USER_LAYER_SOURCES_DIR}/identification/database/ident_database_se3.c
   ${USER_LAYER_SOURCES_DIR}/identification/database/ident_database_sl3.c
   ${USER_LAYER_SOURCES_DIR}/identification/database/image_score.c

   ${USER_LAYER_SOURCES_DIR}/identification/checkerboard/ident_checkerboard.c
   ${USER_LAYER_SOURCES_DIR}/identification/checkerboard/ident_checkerboard_projector.c
)

SET (USER_LAYER_INERTIAL_SOURCES
   ${USER_LAYER_SOURCES_DIR}/inertial/odometry_inertial_observer.c
)


SET (USER_LAYER_ODOMETRY_SOURCES
   ${USER_LAYER_SOURCES_DIR}/odometry/multiplane/odometry_multiplane_params.c
   ${USER_LAYER_SOURCES_DIR}/odometry/multiplane/odometry_multiplane.c

   ${USER_LAYER_SOURCES_DIR}/odometry/plane/odometry_singleplane_params.c
   ${USER_LAYER_SOURCES_DIR}/odometry/plane/odometry_singleplane.c

   ${USER_LAYER_SOURCES_DIR}/odometry/plane/odometry_singleplane_light_affine.c   

   ${USER_LAYER_SOURCES_DIR}/odometry/plane/odometry_singleplane_light_robust.c
   ${USER_LAYER_SOURCES_DIR}/odometry/plane/odometry_singleplane_sparse.c

   ${USER_LAYER_SOURCES_DIR}/odometry/edge/odometry_points.c
   ${USER_LAYER_SOURCES_DIR}/odometry/edge/odometry_segments.c
   ${USER_LAYER_SOURCES_DIR}/odometry/edge/odometry_ellipses.c
   ${USER_LAYER_SOURCES_DIR}/odometry/edge/odometry_cylinders.c
   ${USER_LAYER_SOURCES_DIR}/odometry/edge/odometry_cadmodel.c
)

SET (USER_LAYER_SENSOR_SOURCES
   ${USER_LAYER_SOURCES_DIR}/sensor/camera/camera.c
   ${USER_LAYER_SOURCES_DIR}/sensor/inertial/inertial.c
)

SET (USER_LAYER_TRACKING_SOURCES
   ${USER_LAYER_SOURCES_DIR}/tracking/tracking_params.c
   ${USER_LAYER_SOURCES_DIR}/tracking/tracking.c
   ${USER_LAYER_SOURCES_DIR}/tracking/tracking_sl3.c
   ${USER_LAYER_SOURCES_DIR}/tracking/tracking_tu_tv_s_r.c
   ${USER_LAYER_SOURCES_DIR}/tracking/tracking_tu_tv_su_sv.c
)

SET (USER_LAYER_MOTION_DETECTION_SOURCES
   ${USER_LAYER_SOURCES_DIR}/detection/motion/motion_detection.c
   ${USER_LAYER_SOURCES_DIR}/detection/motion/cluster.c
   ${USER_LAYER_SOURCES_DIR}/detection/rectangle/rectangle_detection.c
   ${USER_LAYER_SOURCES_DIR}/detection/plane/plane_detection.c
   ${USER_LAYER_SOURCES_DIR}/detection/blob/blob_detection.c
)

#Add sources
SET (USER_LAYER_SOURCES
   ${USER_LAYER_CALIBRATION_SOURCES}
   ${USER_LAYER_IDENTIFICATION_SOURCES}
   ${USER_LAYER_INERTIAL_SOURCES}
   ${USER_LAYER_MODEL_SOURCES}
   ${USER_LAYER_ODOMETRY_SOURCES}
   ${USER_LAYER_SENSOR_SOURCES}
   ${USER_LAYER_TRACKING_SOURCES}
   ${USER_LAYER_MOTION_DETECTION_SOURCES}
)

#define hierarchical organization for ide projects
source_group("user\\identification"    FILES    ${USER_LAYER_IDENTIFICATION_SOURCES})
source_group("user\\inertial"          FILES    ${USER_LAYER_INERTIAL_SOURCES})
source_group("user\\odometry"          FILES    ${USER_LAYER_ODOMETRY_SOURCES})
source_group("user\\calibration"       FILES    ${USER_LAYER_CALIBRATION_SOURCES})
source_group("user\\sensor"            FILES    ${USER_LAYER_SENSOR_SOURCES})
source_group("user\\tracking"          FILES    ${USER_LAYER_TRACKING_SOURCES})
source_group("user\\model"             FILES    ${USER_LAYER_MODEL_SOURCES})
source_group("user\\motion"            FILES    ${USER_LAYER_MOTION_DETECTION_SOURCES})
