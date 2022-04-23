# Object sets of complex structures (i.e. structures that contains dynamically allocated data)

# Object sets definitions
macro(objset_generator includename structname typename_lowercase typename deleter)
   SET (OBJSETTYPE_INCLUDE          ${includename})
   SET (OBJSETTYPE_INCLUDE_STRUCT   ${structname})
   SET (LOBJSETTYPE                 ${typename_lowercase})
   SET (OBJSETTYPE                  ${typename})
   SET (OBJSETDELETER               ${deleter})
   configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h    ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
   configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h           ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h        @ONLY)
   configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c           ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c        @ONLY)
endmacro()

objset_generator("<generated/array2d_double.h>" "<generated/array2d_double.h>"  "array2d_double"     "Array2D_Double" "rox_array2d_double_del")
objset_generator("<generated/array2d_uchar.h>"  "<generated/array2d_uchar.h>"   "array2d_uchar"      "Array2D_Uchar"  "rox_array2d_uchar_del")
objset_generator("<generated/array2d_uint.h>"   "<generated/array2d_uint.h>"    "array2d_uint"       "Array2D_Uint"   "rox_array2d_uint_del")
objset_generator("<generated/array2d_float.h>"  "<generated/array2d_float.h>"   "array2d_float"      "Array2D_Float"  "rox_array2d_float_del")
objset_generator("<generated/array2d_sshort.h>" "<generated/array2d_sshort.h>"  "array2d_sshort"     "Array2D_Sshort" "rox_array2d_sshort_del")

# ObjSet array point3d_float
SET (OBJSETTYPE_INCLUDE             "<generated/array2d_point3d_float.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/array2d_point3d_float.h>")
SET (LOBJSETTYPE                    "array2d_point3d_float")
SET (OBJSETTYPE                     "Array2D_Point3D_Float")
SET (OBJSETDELETER                  "rox_array2d_point3d_float_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h       ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h        @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c        @ONLY)

# ObjSet of dynamic vector of sraid
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_sraiddesc.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_sraiddesc_struct.h>")
SET (LOBJSETTYPE                    "dynvec_sraid_feature")
SET (OBJSETTYPE                     "DynVec_SRAID_Feature")
SET (OBJSETDELETER                  "rox_dynvec_sraiddesc_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h       ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h        @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c        @ONLY)

# ObjSet of dynamic vector of segment point
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_segment_point.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_segment_point_struct.h>")
SET (LOBJSETTYPE                    "dynvec_segment_point")
SET (OBJSETTYPE                     "DynVec_Segment_Point")
SET (OBJSETDELETER                  "rox_dynvec_segment_point_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h       ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h        @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c        @ONLY)

# ObjSet of dynamic vector of edgels
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_edgel.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_edgel.h>")
SET (LOBJSETTYPE                    "dynvec_edgel")
SET (OBJSETTYPE                     "DynVec_Edgel")
SET (OBJSETDELETER                  "rox_dynvec_edgel_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h       ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h        @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c        @ONLY)

# ObjSet of dynamic vector of fpsm templates
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_fpsm_template.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_fpsm_template.h>")
SET (LOBJSETTYPE                    "dynvec_fpsm_template")
SET (OBJSETTYPE                     "DynVec_Fpsm_Template")
SET (OBJSETDELETER                  "rox_dynvec_fpsm_template_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h       ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c              ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of sparsevalue
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_sparse_value.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_sparse_value.h>")
SET (LOBJSETTYPE                    "dynvec_sparse_value")
SET (OBJSETTYPE                     "DynVec_Sparse_Value")
SET (OBJSETDELETER                  "rox_dynvec_sparse_value_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of rect_sint_struct
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_rect_sint.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_rect_sint_struct.h>")
SET (LOBJSETTYPE                    "dynvec_rect_sint")
SET (OBJSETTYPE                     "DynVec_Rect_Sint")
SET (OBJSETDELETER                  "rox_dynvec_rect_sint_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of dynamic vector of points 2d double
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point2d_double.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_point2d_double_struct.h>")
SET (LOBJSETTYPE                    "dynvec_point2d_double")
SET (OBJSETTYPE                     "DynVec_Point2D_Double")
SET (OBJSETDELETER                  "rox_dynvec_point2d_double_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of dynamic vector of points 2d float
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point2d_float.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_point2d_float_struct.h>")
SET (LOBJSETTYPE                    "dynvec_point2d_float")
SET (OBJSETTYPE                     "DynVec_Point2D_Float")
SET (OBJSETDELETER                  "rox_dynvec_point2d_float_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h    ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of points 2d sint
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point2d_sint.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_point2d_sint_struct.h>")
SET (LOBJSETTYPE                    "dynvec_point2d_sint")
SET (OBJSETTYPE                     "DynVec_Point2D_Sint")
SET (OBJSETDELETER                  "rox_dynvec_point2d_sint_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of points 2d sshort
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point2d_sshort.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_point2d_sshort_struct.h>")
SET (LOBJSETTYPE                    "dynvec_point2d_sshort")
SET (OBJSETTYPE                     "DynVec_Point2D_Sshort")
SET (OBJSETDELETER                  "rox_dynvec_point2d_sshort_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of points 3d in double precision
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point3d_double.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_point3d_double_struct.h>")
SET (LOBJSETTYPE                    "dynvec_point3d_double")
SET (OBJSETTYPE                     "DynVec_Point3D_Double")
SET (OBJSETDELETER                  "rox_dynvec_point3d_double_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of points 3d in float precision
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point3d_float.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_point3d_float_struct.h>")
SET (LOBJSETTYPE                    "dynvec_point3d_float")
SET (OBJSETTYPE                     "DynVec_Point3D_Float")
SET (OBJSETDELETER                  "rox_dynvec_point3d_float_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of points
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_point_double.h>")
SET (LOBJSETTYPE                    "dynvec_point_double")
SET (OBJSETTYPE                     "DynVec_Point_Double")
SET (OBJSETDELETER                  "rox_dynvec_point_double_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of sint
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_sint.h>")
SET (LOBJSETTYPE                    "dynvec_sint")
SET (OBJSETTYPE                     "DynVec_Sint")
SET (OBJSETDELETER                  "rox_dynvec_sint_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h           ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c           ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of dynamic vector of orientedimagepoint
SET (OBJSETTYPE_INCLUDE             "<generated/dynvec_orientedimagepoint.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<generated/dynvec_orientedimagepoint_struct.h>")
SET (LOBJSETTYPE                    "dynvec_orientedimagepoint")
SET (OBJSETTYPE                     "DynVec_OrientedImagePoint")
SET (OBJSETDELETER                  "rox_dynvec_orientedimagepoint_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h    ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h           ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c           ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of combination
SET (OBJSETTYPE_INCLUDE             "<baseproc/maths/random/combination.h>")
SET (LOBJSETTYPE                    "combination")
SET (OBJSETTYPE                     "Combination")
SET (OBJSETDELETER                  "rox_combination_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of photoframes
SET (OBJSETTYPE_INCLUDE             "<core/identification/photoframe.h>")
SET (LOBJSETTYPE                    "photoframe")
SET (OBJSETTYPE                     "PhotoFrame")
SET (OBJSETDELETER                  "rox_photoframe_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of ehid windows
SET (OBJSETTYPE_INCLUDE             "<core/features/descriptors/ehid/ehid_window.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<core/features/descriptors/ehid/ehid_window_struct.h>")
SET (LOBJSETTYPE                    "ehid_window")
SET (OBJSETTYPE                     "Ehid_Window")
SET (OBJSETDELETER                  "rox_ehid_window_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of checkerboard
SET (OBJSETTYPE_INCLUDE             "<core/features/detectors/checkerboard/checkerboard.h>")
SET (LOBJSETTYPE                    "checkerboard")
SET (OBJSETTYPE                     "CheckerBoard")
SET (OBJSETDELETER                  "rox_checkerboard_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of checkerboard
SET (OBJSETTYPE_INCLUDE             "<core/features/detectors/shape/sdwm_object.h>")
SET (LOBJSETTYPE                    "sdwm_object")
SET (OBJSETTYPE                     "Sdwm_Object")
SET (OBJSETDELETER                  "rox_sdwm_object_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of ehid targets
SET (OBJSETTYPE_INCLUDE             "<core/features/descriptors/ehid/ehid_target.h>")
SET (OBJSETTYPE_INCLUDE_STRUCT      "<core/features/descriptors/ehid/ehid_target_struct.h>")
SET (LOBJSETTYPE                    "ehid_target")
SET (OBJSETTYPE                     "Ehid_Target")
SET (OBJSETDELETER                  "rox_ehid_target_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of model_single_plane
SET (OBJSETTYPE_INCLUDE             "<core/model/model_single_plane.h>")
SET (LOBJSETTYPE                    "model_single_plane")
SET (OBJSETTYPE                     "Model_Single_Plane")
SET (OBJSETDELETER                  "rox_model_single_plane_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of plane search
SET (OBJSETTYPE_INCLUDE             "<core/predict/plane_search.h>")
SET (LOBJSETTYPE                    "plane_search")
SET (OBJSETTYPE                     "Plane_Search")
SET (OBJSETDELETER                  "rox_plane_search_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of plane search uchar
SET (OBJSETTYPE_INCLUDE             "<core/predict/plane_search_uchar.h>")
SET (LOBJSETTYPE                    "plane_search_uchar")
SET (OBJSETTYPE                     "Plane_Search_Uchar")
SET (OBJSETDELETER                  "rox_plane_search_uchar_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h    ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of patch pyramid
SET (OBJSETTYPE_INCLUDE             "<core/patch/patchplane_pyramid.h>")
SET (LOBJSETTYPE                    "patchplane_pyramid")
SET (OBJSETTYPE                     "PatchPlane_Pyramid")
SET (OBJSETDELETER                  "rox_patchplane_pyramid_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of templateident
SET (OBJSETTYPE_INCLUDE             "<core/identification/templateident.h>")
SET (LOBJSETTYPE                    "template_ident")
SET (OBJSETTYPE                     "Template_Ident")
SET (OBJSETDELETER                  "rox_template_ident_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of bundle camera
SET (OBJSETTYPE_INCLUDE             "<core/bundle/bundle_camera.h>")
SET (LOBJSETTYPE                    "bundle_camera")
SET (OBJSETTYPE                     "Bundle_Camera")
SET (OBJSETDELETER                  "rox_bundle_camera_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# ObjSet of bundle frame
SET (OBJSETTYPE_INCLUDE             "<core/bundle/bundle_frame.h>")
SET (LOBJSETTYPE                    "bundle_frame")
SET (OBJSETTYPE                     "Bundle_Frame")
SET (OBJSETDELETER                  "rox_bundle_frame_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h 		${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of bundle point
SET (OBJSETTYPE_INCLUDE             "<core/bundle/bundle_point.h>")
SET (LOBJSETTYPE                    "bundle_point")
SET (OBJSETTYPE                     "Bundle_Point")
SET (OBJSETDELETER                  "rox_bundle_point_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of bundle measure
SET (OBJSETTYPE_INCLUDE             "<core/bundle/bundle_measure.h>")
SET (LOBJSETTYPE                    "bundle_measure")
SET (OBJSETTYPE                     "Bundle_Measure")
SET (OBJSETDELETER                  "rox_bundle_measure_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

SET (OBJSETTYPE_INCLUDE             "<core/detection/detection_checkerboard.h>")
SET (LOBJSETTYPE                    "detection_checkerboard")
SET (OBJSETTYPE                     "Detection_Checkerboard")
SET (OBJSETDELETER                  "rox_detection_checkerboard_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

SET (OBJSETTYPE_INCLUDE             "<core/detection/detection_sl3.h>")
SET (LOBJSETTYPE                    "detection_sl3")
SET (OBJSETTYPE                     "Detection_Sl3")
SET (OBJSETDELETER                  "rox_detection_sl3_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

objset_generator ("<core/tracking/edge/edge_point.h>"    "<core/tracking/edge/edge_point.h>"    "edge_point"    "Edge_Point"    "rox_edge_point_del")
objset_generator ("<core/tracking/edge/edge_segment.h>"  "<core/tracking/edge/edge_segment.h>"  "edge_segment"  "Edge_Segment"  "rox_edge_segment_del")
objset_generator ("<core/tracking/edge/edge_ellipse.h>"  "<core/tracking/edge/edge_ellipse.h>"  "edge_ellipse"  "Edge_Ellipse"  "rox_edge_ellipse_del")
objset_generator ("<core/tracking/edge/edge_cylinder.h>" "<core/tracking/edge/edge_cylinder.h>" "edge_cylinder" "Edge_Cylinder" "rox_edge_cylinder_del")

# Objset segment tracker
SET (OBJSETTYPE_INCLUDE "<core/tracking/edge/tracking_segment.h>")
SET (LOBJSETTYPE                    "tracking_segment")
SET (OBJSETTYPE                     "Tracking_Segment")
SET (OBJSETDELETER                  "rox_tracking_segment_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of ellipse3d 
SET (OBJSETTYPE_INCLUDE             "<baseproc/geometry/ellipse/ellipse3d.h>")
SET (LOBJSETTYPE                    "ellipse3d")
SET (OBJSETTYPE                     "Ellipse3D")
SET (OBJSETDELETER                  "rox_ellipse3d_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h  	${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c  ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of cylinder3d 
SET (OBJSETTYPE_INCLUDE             "<baseproc/geometry/cylinder/cylinder3d.h>")
SET (LOBJSETTYPE                    "cylinder3d")
SET (OBJSETTYPE                     "Cylinder3D")
SET (OBJSETDELETER                  "rox_cylinder3d_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h        ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c        ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of matse3 
SET (OBJSETTYPE_INCLUDE             "<baseproc/maths/linalg/matse3.h>")
SET (LOBJSETTYPE                    "matse3")
SET (OBJSETTYPE                     "MatSE3")
SET (OBJSETDELETER                  "rox_matse3_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h        ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c        ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)

# Objset of imask 
SET (OBJSETTYPE_INCLUDE             "<baseproc/image/imask/imask.h>")
SET (LOBJSETTYPE                    "imask")
SET (OBJSETTYPE                     "Imask")
SET (OBJSETDELETER                  "rox_imask_del")
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template_struct.h ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}_struct.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.h        ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.h @ONLY)
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/templates/objset_template.c        ${OPENROX_BINARY_DIR}/generated/objset_${LOBJSETTYPE}.c @ONLY)
