#BASEPROC LAYER DEFINITION

SET (BASEPROC_LAYER_SOURCES_DIR ${OPENROX_CODE_DIR}/baseproc)

SET (BASEPROC_LAYER_ARRAY_SOURCES
   ${BASEPROC_LAYER_SOURCES_DIR}/array/add/add.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/band/band.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/bnot/bnot.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/count/count_nonzero.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/copy/copy.c

   ${BASEPROC_LAYER_SOURCES_DIR}/array/conversion/array2d_double_from_uchar.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/conversion/array2d_double_from_float.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/conversion/array2d_float_from_uchar.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/conversion/array2d_uint_from_uchar.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/conversion/array2d_uchar_from_float.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/conversion/array2d_uchar_from_uint.c

   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/array2d_uchar_zncc?neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/array2d_uchar_zncc_nomask?neon?.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/array2d_float_zncc.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/array2d_float_zncc_nomask.c

   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/zncrosscor.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/zncrosscor9x9.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/crosscor/zncrosscor11x11.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/crossprod/crossprod.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/qr.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/svd?mkl?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/svd_jacobi.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/svdsort.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/housebidiag.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/cholesky?mkl?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/gausspivoting.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/array/determinant/detgl3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/determinant/detgl2.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/array/eigenv/real_eigenvalues_eigenvectors?mkl?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/eigenv/real_eigenvalues_from_tridiagonal_matrix.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/eigenv/real_eigenvectors_from_eigenvalues.c

   ${BASEPROC_LAYER_SOURCES_DIR}/array/error/ssd.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/error/l2_error.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/error/centered_error.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/exponent/expmat.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/logarithm/logmat.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/fill/fillval.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/fill/fillunit.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/fill/fillzero.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/flip/fliplr.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/flip/flipud.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/integral/integralsum.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/integral/integralaccess.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/mat3x3inv.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/mat2x2inv.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/pseudoinverse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/svdinverse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/lotinverse?mkl?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/inverse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/inverse_lu?mkl?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/median/median.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/mad/mad.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/mean/mean.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/meanvar/meanvar.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/minmax/minmax.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/maxima/maxima.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/morphological/dilate_grayone.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/multiply/mulmatmat.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/multiply/mulmatmattrans.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/multiply/mulmattransmat.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/nonmax/nonmax.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/rotate/rotate90.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/norm/norm2sq.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/normalize/normalize.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/transpose/transpose.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/symmetrise/symmetrise.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/robust/huber.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/robust/tukey.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/scale/scale.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/scale/scaleshift.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/solve/symm3x3solve.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/solve/svd_solve.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/solve/linsys_solve_cholesky.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/substract/substract.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/tridiagonal/tridiagonal.c

   # ANSI functions
   ${BASEPROC_LAYER_SOURCES_DIR}/array/copy/ansi_array_float_copy.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/scale/ansi_scale.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/meanvar/ansi_meanvar.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/multiply/ansi_mulmatmat.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/multiply/ansi_mulmattransmat.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/decomposition/ansi_cholesky.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/inverse/ansi_lotinverse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/error/ansi_ssd.c
   ${BASEPROC_LAYER_SOURCES_DIR}/array/solve/ansi_linsys_solve_cholesky.c
)


# Vectorisation

  if (OPENROX_USE_AVX)
    message("USE AVX")
    SET ( BASEPROC_LAYER_AVX_CALCULUS_SOURCES
        ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/avx_interaction_row_texture_matse3_model3d_zi.c
    )
  endif ()

  if (OPENROX_USE_SSE OR OPENROX_USE_AVX)
    message("USE SSE")
    SET ( BASEPROC_LAYER_SSE_CALCULUS_SOURCES
        ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/sse_interaction_row_texture_matse3_model3d_zi.c
    )
  endif ()

  if (OPENROX_USE_NEON)
    message("USE NEON")
    SET ( BASEPROC_LAYER_NEON_CALCULUS_SOURCES
      ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/neon_interaction_row_texture_matse3_model3d_zi.c
    )
  endif ()

SET (BASEPROC_LAYER_CALCULUS_SOURCES

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/interaction_point2d_pix_matut3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/interaction_point2d_pix_matse3.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/interaction_point2d_nor_matse3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/interaction_point2d_nor_matse3_matso3z_r2.c


   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/interaction_row_point_to_line_matse3.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/ansi_interaction_row_texture_matse3_model3d_zi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/interaction_row_texture_matse3_model3d_zi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/jacobian_row_bundle_matse3_point3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/jacobians/jacobian_perspective_stereo_calibration.c

   # Wrappers for ansi, sse, avx or neon optimisation
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/ansi_linsys_texture_matse3_light_affine_model3d_zi?sse,avx,neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matse3_light_affine_model3d_zi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_weighted_texture_matse3_light_affine_model3d_zi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/ansi_linsys_se3_z1_light_affine_premul?sse,neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_se3_z1_light_affine_premul.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_point_to_line_matse3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matse3_model3d_zi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/ansi_linsys_texture_matsl3_light_affine?neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matsl3_light_affine.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_weighted_texture_matse3_light_affine_model3d.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_stereo_point2d_pix_matse3_weighted.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_essential_geometric_weighted_premul.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_generalized_geometric_weighted_premul.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/ansi_linsys_se3_light_affine_premul?sse?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_se3_light_affine_premul.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_se3_light_affine_premul_left.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_se3_z1_light_affine_premul_left.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_se3_light_affine_change_update.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_point2d_nor_matse3_matso3z.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_point2d_nor_matse3_matso3z_matso3z.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_point2d_pix_matsl3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_point2d_pix_matse3_weighted.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_point2d_nor_matse3_weighted.c

   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matse3_light_affine_model2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matse3_light_affine_depth_model3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matse2_light_affine_model2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_tutvsr_light_affine_model2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_tutvsusv_light_affine_model2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_matso3_light_affine.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_rx_light_affine.c
   ${BASEPROC_LAYER_SOURCES_DIR}/calculus/linsys/linsys_texture_rxry_light_affine.c
)

SET (BASEPROC_LAYER_GEOMETRY_SOURCES

   ${OPENROX_BINARY_DIR}/generated/array2d_point2d_float.c
   ${OPENROX_BINARY_DIR}/generated/array2d_point2d_double.c
   ${OPENROX_BINARY_DIR}/generated/array2d_point3d_float.c
   ${OPENROX_BINARY_DIR}/generated/array2d_point3d_double.c
   ${OPENROX_BINARY_DIR}/generated/array2d_point2d_sshort.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point_double.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point2d_float.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point2d_double.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point2d_sint.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point2d_uint.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point2d_sshort.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point3d_float.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point3d_double.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_point3d_sint.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_segment2d.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_triangle_double.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_triangle_index.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_rect_sint.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_point2d_double.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_point2d_sint.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_point2d_sshort.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_point_double.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_rect_sint.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_point3d_float.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_point3d_double.c
   ${OPENROX_BINARY_DIR}/generated/objset_array2d_point3d_float.c
   ${OPENROX_BINARY_DIR}/generated/objset_ellipse3d.c
   ${OPENROX_BINARY_DIR}/generated/objset_cylinder3d.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/cadmodel/cadmodel.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/calibration/optimalcalib.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/connectivity/connectivity.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/disparity/depthfromdisparity.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/dynvec_point2d_tools.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/dynvec_point2d_projection_from_dynvec_point3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/dynvec_point3d_tools.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/dynvec_point3d_matse3_transform.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/objset_dynvec_point3d_matse3_transform.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point2d_tools.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point2d_matsl3_transform.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point2d_projection_from_point3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point2d_projection_from_point3d_transform.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point3d_sphere.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point3d_from_template.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point3d_inverse_projection_from_point2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point3d_matse3_transform.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/point/point3d_tools.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/segment/segment2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/segment/segment3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/segment/segment_project.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/segment/segment_transform.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line_clip.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line_from_planes.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line_from_points.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line_project.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line_transform.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/line/line_closestpoint.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/rectangle/rectangle.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/ellipse/ellipse2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/ellipse/ellipse3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/ellipse/ellipse_from_points.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/ellipse/ellipse_project.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/ellipse/ellipse_transform.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/cylinder/cylinder2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/cylinder/cylinder3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/cylinder/cylinder_project.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/cylinder/cylinder_transform.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/plane/plane_3points.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/plane/plane_transform.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/meshgrid2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/warp_grid_distortion.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/warp_grid_distortion_matsl3.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/ansi_warp_grid_matsl3?sse,neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/warp_grid_matsl3.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/ansi_warp_grid_matsl3_fixed12_4?neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/warp_grid_matsl3_fixed12_4.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/ansi_warp_grid_matse3_z?sse?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/warp_grid_matse3_z.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/ansi_warp_grid_matse3_zi?sse?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/pixelgrid/warp_grid_matse3_zi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/measures/distance_point_to_point.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/measures/distance_point_to_line.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/measures/distance_point_to_segment.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/measures/distance_point_to_ellipse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/measures/intersection_line_ellipse.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/transform_tools.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/distortion/point2d_undistort.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matsl3/sl3virtualview.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matsl3/sl3from4points.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matsl3/sl3fromNpoints.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matsl3/sl3normalize.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matsl3/sl3interfrom3dpoints.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matse3/matse3_from_n_points3d_to_points2d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matse3/matse3_from_n_points3d_to_planes3d.c
   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/transforms/matse3/matse3_from_points3d_sets.c

   ${BASEPROC_LAYER_SOURCES_DIR}/geometry/specifics/init_vvs_se3_so3z_so3z.c
)

SET (BASEPROC_LAYER_IMAGE_SOURCES

   ${BASEPROC_LAYER_SOURCES_DIR}/image/image.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/image_rgba.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/image_rgba_draw.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/image_rgba_draw_projection_model_single_plane.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/image_rgba_draw_projection_model_multi_plane.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/color.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_line.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_circle.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_ellipse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_cylinder.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_points.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_polygon.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_polyline.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_rectangle.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_warp_polygon.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_warp_rectangle.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/draw/draw_checkercorner_detector_corners.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/integral/integral.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/imask/imask.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/imask/apply/mask_rgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/imask/fill/set_border.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/imask/fill/set_data.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/imask/fill/set_ellipse.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/imask/fill/set_polygon.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/image/roi/image_roi.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/warp/image_warp_matsl3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/warp/image_rgba_warp_matsl3.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/convolve/basic_convolve.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convolve/sparse_convolve.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/convolve/ansi_array2d_float_symmetric_separable_convolve.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convolve/array2d_float_symmetric_separable_convolve?sse?.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/convolve/array2d_uchar_symmetric_separable_convolve.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/gradient/gradientsobel.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/gradient/gradient_tap_5x5.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/gradient/gradient_anglenorm.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/gradient/ansi_basegradient?sse,neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/gradient/basegradient.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/pyramid/pyramid_tools.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/pyramid/pyramid_float.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/pyramid/pyramid_uint.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/pyramid/pyramid_uchar.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/pyramid/pyramid_npot_uchar.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_nn_halved/remap_nn_halved.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_box_halved/ansi_remap_box_halved?sse,avx?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_box_halved/remap_box_halved.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_box_mask_halved/remap_box_mask_halved.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_float_to_float/ansi_remap_bilinear_float_to_float?sse?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_float_to_float/remap_bilinear_float_to_float.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_uchar_to_float/ansi_remap_bilinear_uchar_to_float?sse?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_uchar_to_float/remap_bilinear_uchar_to_float.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_omo_float_to_float/ansi_remap_bilinear_omo_float_to_float?sse,neon?.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_uchar_to_uchar/remap_bilinear_uchar_to_uchar.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_onepixel/remap_bilinear_onepixel.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_trans/remap_bilinear_trans.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float_doubled.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar?neon?.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_bilinear_nomask_uint_to_uint/remap_bilinear_nomask_uint_to_uint?sse?.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/remap/remap_ewa_omo/remap_ewa_omo.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/image/noise/gaussian_noise.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/transform/distance.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/filter/median/medianfilter.c

   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/argb_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/argb_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/bgra_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/bgra_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/bgr_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/bgr_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/gray_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/gray_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/rgba_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/rgba_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/rgb_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/rgb_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxgray_to_gray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxgray_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxrgba_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxrgba_to_roxlab.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxrgba_to_rgb.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxrgba_to_rgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/yuv422_to_roxgray.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/yuv422_to_roxrgba.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/roxrgba_split.c
   ${BASEPROC_LAYER_SOURCES_DIR}/image/convert/alpha8_to_roxgray.c
)

SET (BASEPROC_LAYER_MATHS_SOURCES
   ${OPENROX_BINARY_DIR}/generated/objset_combination.c
   ${OPENROX_BINARY_DIR}/generated/objset_matse3

   ${BASEPROC_LAYER_SOURCES_DIR}/maths/base/basemaths.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/filter/filter_matse3.c

   ${BASEPROC_LAYER_SOURCES_DIR}/maths/kernels/gaussian2d.c
   
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/nonlin/polynomials.c

   ${BASEPROC_LAYER_SOURCES_DIR}/maths/random/combination.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/random/random.c

   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algsl3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algse3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algse2.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algso3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/alglt3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algut3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algtutvsr.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/algtutvsusv.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/matrix.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/matse3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/matso3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/matsl3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/matut3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/matlt3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/quaternion/quaternion.c

   # ANSI functions 
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/generators/ansi_algse3.c
   ${BASEPROC_LAYER_SOURCES_DIR}/maths/linalg/ansi_matse3.c
)

SET (BASEPROC_LAYER_TOOLS_SOURCES
   ${BASEPROC_LAYER_SOURCES_DIR}/tools/encoder/bch.c
   ${BASEPROC_LAYER_SOURCES_DIR}/tools/string/filepath.c
)

replace_platform_optimization(BASEPROC_LAYER_ARRAY_SOURCES)
replace_platform_optimization(BASEPROC_LAYER_CALCULUS_SOURCES)
replace_platform_optimization(BASEPROC_LAYER_GEOMETRY_SOURCES)
replace_platform_optimization(BASEPROC_LAYER_IMAGE_SOURCES)
replace_platform_optimization(BASEPROC_LAYER_MATHS_SOURCES)
replace_platform_optimization(BASEPROC_LAYER_TOOLS_SOURCES)

#Add sources
SET (BASEPROC_LAYER_SOURCES
   ${BASEPROC_LAYER_ARRAY_SOURCES}
   ${BASEPROC_LAYER_CALCULUS_SOURCES}
   ${BASEPROC_LAYER_SSE_CALCULUS_SOURCES}
   ${BASEPROC_LAYER_AVX_CALCULUS_SOURCES}
   ${BASEPROC_LAYER_NEON_CALCULUS_SOURCES}
   ${BASEPROC_LAYER_GEOMETRY_SOURCES}
   ${BASEPROC_LAYER_IMAGE_SOURCES}
   ${BASEPROC_LAYER_MATHS_SOURCES}
   ${BASEPROC_LAYER_TOOLS_SOURCES}
)

#define hierarchical organization for ide projects
source_group("baseproc\\array"      FILES    ${BASEPROC_LAYER_ARRAY_SOURCES})
source_group("baseproc\\calculus"   FILES    ${BASEPROC_LAYER_CALCULUS_SOURCES})
source_group("baseproc\\geometry"   FILES    ${BASEPROC_LAYER_GEOMETRY_SOURCES})
source_group("baseproc\\image"      FILES    ${BASEPROC_LAYER_IMAGE_SOURCES})
source_group("baseproc\\maths"      FILES    ${BASEPROC_LAYER_MATHS_SOURCES})
source_group("baseproc\\tools"      FILES    ${BASEPROC_LAYER_TOOLS_SOURCES})
