//==============================================================================
//
//    OPENROX   : File openrox.h
//
//    Contents  : API of OPENROX
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <system/memory/datatypes.h>
#include <system/version/version.h>
#include <system/time/timer.h>

#include <generated/objset_matse3.h>
#include <generated/objset_matse3_struct.h>
#include <generated/objset_dynvec_point3d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/filter/filter_matse3.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <baseproc/geometry/point/objset_dynvec_point3d_matse3_transform.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/image/warp/image_rgba_warp_matsl3.h>
#include <baseproc/tools/string/filepath.h>
#include <baseproc/image/image.h>
#include <baseproc/image/image_rgba.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>
#include <baseproc/image/draw/image_rgba_draw_projection_model_single_plane.h>
#include <baseproc/image/draw/image_rgba_draw_projection_model_multi_plane.h>
#include <baseproc/image/draw/color.h>
#include <baseproc/image/draw/draw_points.h>
#include <baseproc/image/draw/draw_warp_polygon.h>
#include <baseproc/image/draw/draw_rectangle.h>
#include <baseproc/image/warp/image_rgba_warp_matsl3.h>

#include <core/features/descriptors/sraid/sraid.h>
#include <core/model/model_multi_plane.h>
#include <core/model/model_projector_checkerboard.h>
#include <core/indirect/euclidean/vvs_points_se3_so3z_r2.h>

#include <user/calibration/mono/camera_calibration.h>
#include <user/calibration/projector/calibration_projector_checkerboard.h>
#include <user/detection/motion/motion_detection.h>
#include <user/detection/plane/plane_detection.h>
#include <user/detection/rectangle/rectangle_detection.h>
#include <user/identification/texture/identification.h>
#include <user/identification/texture/ident_texture_sl3.h>
#include <user/identification/texture/ident_texture_se3.h>
#include <user/identification/texture/ident_multiplane.h>
#include <user/identification/database/ident_database_sl3.h>
#include <user/identification/database/ident_database_se3.h>
#include <user/identification/database/database_features.h>
#include <user/identification/database/image_score.h>
#include <user/identification/photoframe/ident_photoframe_se3.h>
#include <user/identification/photoframe/ident_photoframe_sl3.h>
#include <user/tracking/tracking.h>
#include <user/tracking/tracking_params.h>
#include <user/odometry/plane/odometry_singleplane.h>
#include <user/odometry/plane/odometry_singleplane_params.h>
#include <user/odometry/multiplane/odometry_multiplane.h>
#include <user/inertial/odometry_inertial_observer.h>
#include <user/sensor/camera/camera.h>
#include <user/sensor/inertial/inertial.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>
#include <inout/geometry/point/point3d_save.h>
#include <inout/geometry/point/point2d_print.h>
#include <inout/geometry/point/point3d_print.h>
#include <inout/numeric/array_save.h>
#include <inout/numeric/array2d_save.h>
#include <inout/numeric/array2d_print.h>
#include <inout/numeric/dynvec_serialize.h>
#include <inout/geometry/point/objset_dynvec_point2d_print.h>