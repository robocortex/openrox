#CORE LAYER DEFINITION

SET (CORE_LAYER_SOURCES_DIR ${OPENROX_CODE_DIR}/core)

SET (CORE_CALIBRATION_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/calibration/mono/calibration_perspective.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/mono/calibration_generalized.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/stereo/stereo_rectifier.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/stereo/stereo_calibration.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/stereo/stereo_calibration_se3.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/inertial/calibration_vision_inertial.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/projector/calibration_perspective.c
   ${CORE_LAYER_SOURCES_DIR}/calibration/camproj/calibration_camproj.c
)
SET (CORE_DETECTION_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/detection/detection_checkerboard.c
   ${OPENROX_BINARY_DIR}/generated/objset_detection_checkerboard.c
   ${CORE_LAYER_SOURCES_DIR}/detection/detection_sl3.c
   ${OPENROX_BINARY_DIR}/generated/objset_detection_sl3.c
)

SET (CORE_BUNDLE_SOURCES
   ${OPENROX_BINARY_DIR}/generated/objset_bundle_camera.c
   ${OPENROX_BINARY_DIR}/generated/objset_bundle_frame.c
   ${OPENROX_BINARY_DIR}/generated/objset_bundle_point.c
   ${OPENROX_BINARY_DIR}/generated/objset_bundle_measure.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_bundle_measure.c
   ${CORE_LAYER_SOURCES_DIR}/bundle/bundle_camera.c
   ${CORE_LAYER_SOURCES_DIR}/bundle/bundle_point.c
   ${CORE_LAYER_SOURCES_DIR}/bundle/bundle_frame.c
   ${CORE_LAYER_SOURCES_DIR}/bundle/bundle_measure.c
   ${CORE_LAYER_SOURCES_DIR}/bundle/bundle.c
)

SET (CORE_FEATURES_DESCRIPTORS_SOURCES
   ${OPENROX_BINARY_DIR}/generated/dynvec_sraiddesc.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_sraid_feature.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_brief_point.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_ehid_point.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_ehid_match.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_ehid_dbindex.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_ehid_dbnode.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_tlid_segment.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_fpsm_feature.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_fpsm_template.c
   ${OPENROX_BINARY_DIR}/generated/objset_ehid_window.c
   ${OPENROX_BINARY_DIR}/generated/objset_ehid_target.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_fpsm_template.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/brief/brief.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/sraid.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/sraiddesc.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/sraid_match?sse?.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/sraid_matchset.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/sraid_matchresultset.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/heap_branch.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/kdtree_sraid.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/sraid/sraiddesc_kmean.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_computedescriptor.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/dynvec_ehid_dbindex_tools.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_window.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_viewpointbin.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_target.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_matcher.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_database.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_searchtree.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/ehid/ehid_compiler.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/tlid/tlid.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/tlid/tlid_matcher.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/fpsm/fpsm.c
   ${CORE_LAYER_SOURCES_DIR}/features/descriptors/fpsm/fpsm_index.c
)

SET (CORE_FEATURES_DETECTORS_SOURCES
   ${OPENROX_BINARY_DIR}/generated/dynvec_dog.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_segment_point.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_edgeturn.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_edgel.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_segment_part.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_segment_point.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_edgel.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_checkercorner.c
   ${OPENROX_BINARY_DIR}/generated/objset_checkerboard.c
   ${OPENROX_BINARY_DIR}/generated/objset_sdwm_object.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_quad.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_quad_segment2d.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_orientedimagepoint.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_orientedimagepoint.c

   ${CORE_LAYER_SOURCES_DIR}/features/detectors/dog/dog.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/segment/fastst?sse,neon?.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/segment/fastst_score?sse,neon?.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/segment/segmentpoint_tools.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/corners/shicorner9x9?sse?.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/corners/shicorner.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/corners/harris.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/corners/gftt.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/orientation/orimoments.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/checkerboard/checkercorner_detect.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/checkerboard/checkerboard_detect.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/checkerboard/checkerboard.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/canny.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/edgedraw.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/edgepreproc_gray.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/edgepreproc_rgb.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/edgepostproc_ac.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/edgepostproc_segment.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/edges/edgepostproc_normal.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/shape/sdwm.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/shape/sdwm_object.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/quad/quad_detection.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/quad/quad_gradientclusterer.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/quad/quad_segment2d.c
   ${CORE_LAYER_SOURCES_DIR}/features/detectors/quad/quad_gradientclusterer_funcs?neon?.c
)

SET (CORE_MATCHING_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/matching/matching_3d2d.c
)

SET (CORE_MODEL_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/model/model_single_plane.c
   ${CORE_LAYER_SOURCES_DIR}/model/model_multi_plane.c
   ${CORE_LAYER_SOURCES_DIR}/model/model_checkerboard.c
   ${CORE_LAYER_SOURCES_DIR}/model/model_projector_checkerboard.c
)

SET (CORE_ODOMETRY_SOURCES
   ${OPENROX_BINARY_DIR}/generated/objset_model_single_plane.c
   ${OPENROX_BINARY_DIR}/generated/objset_plane_search.c
   ${OPENROX_BINARY_DIR}/generated/objset_plane_search_uchar.c
   ${OPENROX_BINARY_DIR}/generated/objset_patchplane_pyramid.c

   ${CORE_LAYER_SOURCES_DIR}/odometry/plane/odometry_plane_robustlight.c

   ${CORE_LAYER_SOURCES_DIR}/odometry/plane/odometry_plane.c

   ${CORE_LAYER_SOURCES_DIR}/odometry/multiplane/odometry_planes.c


   ${CORE_LAYER_SOURCES_DIR}/odometry/essential/odometry_essential.c
   ${CORE_LAYER_SOURCES_DIR}/odometry/affine/odometry_se2.c
   ${CORE_LAYER_SOURCES_DIR}/odometry/rotation/odometry_so3.c
   ${CORE_LAYER_SOURCES_DIR}/odometry/depth/odometry_dense_depthmap.c

   # Edges

   ${CORE_LAYER_SOURCES_DIR}/odometry/edge/objset_edge_point_tools.c
   ${CORE_LAYER_SOURCES_DIR}/odometry/edge/objset_edge_segment_tools.c
   ${CORE_LAYER_SOURCES_DIR}/odometry/edge/objset_edge_ellipse_tools.c
   ${CORE_LAYER_SOURCES_DIR}/odometry/edge/objset_edge_cylinder_tools.c

)

SET (CORE_OCCUPANCY_SOURCES
   ${OPENROX_BINARY_DIR}/generated/dllist_quadtree_item.c
   ${CORE_LAYER_SOURCES_DIR}/occupancy/quadtree_ref.c
   ${CORE_LAYER_SOURCES_DIR}/occupancy/quadtree_static.c
)

SET (CORE_PATCH_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/patch/patchplane.c
   ${CORE_LAYER_SOURCES_DIR}/patch/patchplane_pyramid.c
   ${CORE_LAYER_SOURCES_DIR}/patch/patchplane_robustlight.c
   ${CORE_LAYER_SOURCES_DIR}/patch/patchplane_robustlight_pyramid.c
)

SET (CORE_IDENTIFICATION_SOURCES
   ${OPENROX_BINARY_DIR}/generated/objset_photoframe.c
   ${OPENROX_BINARY_DIR}/generated/objset_template_ident.c
   ${CORE_LAYER_SOURCES_DIR}/identification/templateident.c
   ${CORE_LAYER_SOURCES_DIR}/identification/templateident_sl3.c
   ${CORE_LAYER_SOURCES_DIR}/identification/templateident_se3.c
   ${CORE_LAYER_SOURCES_DIR}/identification/multiident.c
   ${CORE_LAYER_SOURCES_DIR}/identification/photoframe.c
   ${CORE_LAYER_SOURCES_DIR}/identification/codedframe.c
   ${CORE_LAYER_SOURCES_DIR}/identification/dbident_se3.c
   ${CORE_LAYER_SOURCES_DIR}/identification/dbident_sl3.c
)

SET (CORE_INDIRECT_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/indirect/homography/ransacsl3.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/homography/vvspointssl3.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/ransacse3.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/vvspointsse3.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/vvs_se3_so3z.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/vvs_se3_so3z_so3z.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/vvs_points_se3_so3z_r2.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/vvs_tools.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/vvspointsse3_stereo.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/p3points.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/p5points.c
   # ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/p16lines.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/triangulate.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/pointcloud_similarity.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/matso3_from_vectors.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/ransac_se3_vvs_se3.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/euclidean/covariance_backpropagation_se3_points.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/essential/ransacessentialcommon.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/essential/ransacessentialpose.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/essential/e5points_nister.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/essential/essentialerror.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/essential/essentialposes.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/essential/essentialminimize.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/cd4_1.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/cd4_2.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/cd4_3.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/cd4_4.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/cdet4.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/makeeqs.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/makeMt.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/p7p.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/ransac_nonoverlap.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/nonoverlapminimize.c
   ${CORE_LAYER_SOURCES_DIR}/indirect/multinonoverlap/nonoverlaperror.c
)

SET (CORE_INERTIAL_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/inertial/frame/frame.c
   ${CORE_LAYER_SOURCES_DIR}/inertial/measure/inertial_measure.c
   ${CORE_LAYER_SOURCES_DIR}/inertial/measure/inertial_measure_buffer.c
   ${CORE_LAYER_SOURCES_DIR}/inertial/sensor/imu.c
   ${CORE_LAYER_SOURCES_DIR}/inertial/observer/inertial_observer.c
   ${CORE_LAYER_SOURCES_DIR}/inertial/observer/complementary_filter.c
)

SET (CORE_PREDICT_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/predict/motion/kalman_ctscav.c
   ${CORE_LAYER_SOURCES_DIR}/predict/plane_search.c
   ${CORE_LAYER_SOURCES_DIR}/predict/plane_search_uchar.c

   ${CORE_LAYER_SOURCES_DIR}/score/image_window_difference_score.c
)

SET (CORE_TEMPLATESEARCH_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/templatesearch/region_zebc_search_mask_template_mask.c
   ${CORE_LAYER_SOURCES_DIR}/templatesearch/ansi_region_zncc_search_mask_template_mask.c
   ${CORE_LAYER_SOURCES_DIR}/templatesearch/region_zncc_search_mask_template_mask?sse,neon?.c
   ${CORE_LAYER_SOURCES_DIR}/templatesearch/ocm.c
)

SET (CORE_TRACKING_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/tracking/patch/tracking_patch_sl3.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/patch/tracking_patch_tu_tv_s_r.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/patch/tracking_patch_tu_tv_su_sv.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/point/tracking_point.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/point/tracking_point_9x9.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/point/tracking_point_11x11.c

   # Edge

   ${OPENROX_BINARY_DIR}/generated/dynvec_edge_point_site.c
   ${OPENROX_BINARY_DIR}/generated/objset_edge_point.c

   ${OPENROX_BINARY_DIR}/generated/dynvec_edge_segment_site.c
   ${OPENROX_BINARY_DIR}/generated/objset_edge_segment.c

   ${OPENROX_BINARY_DIR}/generated/dynvec_edge_ellipse_site.c
   ${OPENROX_BINARY_DIR}/generated/objset_edge_ellipse.c

   ${OPENROX_BINARY_DIR}/generated/dynvec_edge_cylinder_site.c
   ${OPENROX_BINARY_DIR}/generated/objset_edge_cylinder.c

   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/moving_edge_params.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/moving_edge.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/search_edge.c

   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/edge_point.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/edge_segment.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/edge_ellipse.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/edge_cylinder.c

   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/tracking_epoint.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/tracking_segment.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/tracking_ellipse.c
   ${CORE_LAYER_SOURCES_DIR}/tracking/edge/tracking_cylinder.c
)

SET (CORE_VIRTUALVIEW_SOURCES
   ${CORE_LAYER_SOURCES_DIR}/virtualview/planar_view_generator.c
   ${CORE_LAYER_SOURCES_DIR}/virtualview/unwrap_model.c
)

replace_platform_optimization(CORE_CALIBRATION_SOURCES)
replace_platform_optimization(CORE_BUNDLE_SOURCES)
replace_platform_optimization(CORE_FEATURES_DESCRIPTORS_SOURCES)
replace_platform_optimization(CORE_FEATURES_DETECTORS_SOURCES)
replace_platform_optimization(CORE_IDENTIFICATION_SOURCES)
replace_platform_optimization(CORE_INDIRECT_SOURCES)
replace_platform_optimization(CORE_INERTIAL_SOURCES)
replace_platform_optimization(CORE_MATCHING_SOURCES)
replace_platform_optimization(CORE_OCCUPANCY_SOURCES)
replace_platform_optimization(CORE_ODOMETRY_SOURCES)
replace_platform_optimization(CORE_PATCH_SOURCES)
replace_platform_optimization(CORE_PREDICT_SOURCES)
replace_platform_optimization(CORE_TEMPLATESEARCH_SOURCES)
replace_platform_optimization(CORE_TRACKING_SOURCES)
replace_platform_optimization(CORE_VIRTUALVIEW_SOURCES)

#Add sources
SET (CORE_LAYER_SOURCES
   ${CORE_CALIBRATION_SOURCES}
   ${CORE_DETECTION_SOURCES}
   ${CORE_BUNDLE_SOURCES}
   ${CORE_FEATURES_DESCRIPTORS_SOURCES}
   ${CORE_FEATURES_DETECTORS_SOURCES}
   ${CORE_IDENTIFICATION_SOURCES}
   ${CORE_INDIRECT_SOURCES}
   ${CORE_INERTIAL_SOURCES}
   ${CORE_MATCHING_SOURCES}
   ${CORE_MODEL_SOURCES}
   ${CORE_LAYER_MODEL_SOURCES}
   ${CORE_OCCUPANCY_SOURCES}
   ${CORE_ODOMETRY_SOURCES}
   ${CORE_PATCH_SOURCES}
   ${CORE_PREDICT_SOURCES}
   ${CORE_TEMPLATESEARCH_SOURCES}
   ${CORE_TRACKING_SOURCES}
   ${CORE_VIRTUALVIEW_SOURCES}
)

#define hierarchical organization for ide projects
source_group("core\\calibration"             FILES    ${CORE_CALIBRATION_SOURCES})
source_group("core\\detection"               FILES    ${CORE_DETECTION_SOURCES})
source_group("core\\bundle"                  FILES    ${CORE_BUNDLE_SOURCES})
source_group("core\\features\\detectors"     FILES    ${CORE_FEATURES_DETECTORS_SOURCES})
source_group("core\\features\\descriptors"   FILES    ${CORE_FEATURES_DESCRIPTORS_SOURCES})
source_group("core\\identification"          FILES    ${CORE_IDENTIFICATION_SOURCES})
source_group("core\\indirect"                FILES    ${CORE_INDIRECT_SOURCES})
source_group("core\\inertial"                FILES    ${CORE_INERTIAL_SOURCES})
source_group("core\\odometry"                FILES    ${CORE_ODOMETRY_SOURCES})
source_group("core\\matching"                FILES    ${CORE_MATCHING_SOURCES})
source_group("core\\model"                   FILES    ${CORE_MODEL_SOURCES})
source_group("core\\occupancy"               FILES    ${CORE_OCCUPANCY_SOURCES})
source_group("core\\odometry"                FILES    ${CORE_ODOMETRY_SOURCES})
source_group("core\\patch"                   FILES    ${CORE_PATCH_SOURCES})
source_group("core\\predict"                 FILES    ${CORE_PREDICT_SOURCES})
source_group("core\\templatesearch"          FILES    ${CORE_TEMPLATESEARCH_SOURCES})
source_group("core\\tracking"                FILES    ${CORE_TRACKING_SOURCES})
source_group("core\\pyramid"                 FILES    ${CORE_PYRAMID_SOURCES})
source_group("core\\vview"                   FILES    ${CORE_VIRTUALVIEW_SOURCES})
