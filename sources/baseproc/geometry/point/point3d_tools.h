//==============================================================================
//
//    OPENROX   : File points3d_tools.h
//
//    Contents  : API of points3d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS3D_TOOLS__
#define __OPENROX_POINTS3D_TOOLS__

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

#include <system/errors/errors.h>

#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_float.h>

//! \addtogroup Point3D
//! @{

//! Set data
//! \param  [out]  output 
//! \param  [in ]  X
//! \param  [in ]  Y
//! \param  [in ]  Z
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point3d_double_set_data ( 
   Rox_Point3D_Double output, 
   const Rox_Double X, 
   const Rox_Double Y, 
   const Rox_Double Z
);

//! Copy
//! \param  [out]  output 
//! \param  [in ]  input  
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point3d_double_copy ( 
   Rox_Point3D_Double output, 
   const Rox_Point3D_Double input 
);



//! Compute the distance between two 3D points
//! \param  [out]  distance    The distance = || point3d_1 - point3d_2 ||
//! \param  [in ]  point3d_1
//! \param  [in ]  point3d_2 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point3d_double_distance ( 
   Rox_Double * distance, 
   const Rox_Point3D_Double point3d_1, 
   const Rox_Point3D_Double point3d_2 
);

//! Convert point3d float to double
//! \param  [out]  output    The distance = || point3d_1 - point3d_2 ||
//! \param  [in ]  input
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point3d_double_convert_from_float ( 
   Rox_Point3D_Double output, 
   const Rox_Point3D_Float input 
);

//! Compute the distance between two 3D points
//! \param  [out]  bounding_box   The 8 corners of the bounding box
//! \param  [in ]  points_list    The list of 3D points [X1, Y1, Z1, X2, Y2, Z2, ..., Xn, Yn, Zn ]  
//! \param  [in ]  nb_points      The number of points n in point_list 
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_double_compute_bounding_box ( 
   Rox_Point3D_Double bounding_box, 
   const Rox_Double * points_list,
   const Rox_Sint nb_points
);


ROX_API Rox_ErrorCode rox_point3d_float_copy (
   Rox_Point3D_Float output, 
   const Rox_Point3D_Float input 
);


ROX_API Rox_ErrorCode rox_vector_point3d_float_copy ( 
   Rox_Point3D_Float output, 
   const Rox_Point3D_Float input,
   const Rox_Sint nbp
);


// TODO : should these functions be moved in cadmodel module ???

//! Compute the distance between two 3D points
//! \param  [out]  bounding_box   The 8 corners of the bounding box
//! \param  [in ]  points_list    The list of 3D points [X1, Y1, Z1, X2, Y2, Z2, ..., Xn, Yn, Zn ]  
//! \param  [in ]  nb_points      The number of points n in point_list 
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_float_compute_bounding_box ( 
   Rox_Point3D_Float bounding_box, 
   const Rox_Float * points_list,
   const Rox_Sint nb_points
);

//! Compute the distance between two 3D points
//! \param  [out]  vertices       The list of 3D vertices [X1, Y1, Z1, X2, Y2, Z2, ..., Xn, Yn, Zn ]  
//! \param  [in ]  n_vertices     The number of points n in vertices
//! \param  [in ]  shift          The shift
//! \param  [in ]  scale          The scale
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_float_shift_scale_double ( 
   float * vertices, 
   const int n_vertices, 
   const double center[3], 
   const double scale 
);

//! Compute the distance between two 3D points
//! \param  [out]  vertices       The list of 3D vertices [X1, Y1, Z1, X2, Y2, Z2, ..., Xn, Yn, Zn ]  
//! \param  [in ]  n_vertices     The number of points n in vertices
//! \param  [in ]  shift          The shift
//! \param  [in ]  scale          The scale
//! \return An error code

ROX_API Rox_ErrorCode rox_point3d_float_shift_scale_float(
   float * vertices, 
   const int n_vertices, 
   const float shift[3], 
   const float scale 
);

ROX_API Rox_ErrorCode rox_point3d_float_compute_center ( 
   Rox_Point3D_Float center,
   Rox_Point3D_Float points_list, 
   const Rox_Sint nb_points
);

ROX_API Rox_ErrorCode rox_vector_point3d_double_mean (
   Rox_Point3D_Double pt_mean,
   const Rox_Point3D_Double pts_vec,
   const Rox_Sint nbp
);

ROX_API Rox_ErrorCode rox_vector_point3d_double_shift (
   Rox_Point3D_Double pts_vec,
   Rox_Sint nbp,
   Rox_Point3D_Double pt_move
);

ROX_API Rox_ErrorCode rox_vector_point3d_double_normalize_unit (
   Rox_Point3D_Double point3D_vector,
   const Rox_Sint nbp
);


ROX_API Rox_ErrorCode rox_vector_point3d_double_center_normalize (
   Rox_Point3D_Double pts_dst,
   const Rox_Point3D_Double pts_src,
   const Rox_Sint nbp
);

ROX_API Rox_ErrorCode rox_vector_point3d_double_set_data ( 
   Rox_Point3D_Double points3D, 
   const Rox_Double * data_points3D, 
   const Rox_Sint numb_points3D
);

ROX_API Rox_ErrorCode rox_vector_point3d_double_copy ( 
   Rox_Point3D_Double output, 
   const Rox_Point3D_Double input,
   const Rox_Sint nbp
);

//! Read from file
//! \param  [out]   output   :
//! \param  [in]    filename :
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_vector_point3d_double_read_txt ( 
   Rox_Point3D_Double output, 
   const Rox_Char * filename 
);


//! Check if all points repprojects in teh image from a given point of view
//! \param  [out]  visibility     Flag [0 the points are not visible, 1 the points are visible]
//! \param  [in ]  image_rows     Number of rows of the image
//! \param  [in ]  image_cols     Number of cols of the image
//! \param  [in ]  Kc             The camera intronsic parameters
//! \param  [in ]  cTo            The pose of the camera frame Fc relative to the object frame Fo 
//! \param  [in ]  mo             The 3D points
//! \param  [in ]  nbp            The number of points
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_vector_point3d_double_check_visibility (
   Rox_Sint * visibility, 
   const Rox_Sint image_rows,
   const Rox_Sint image_cols,
   const Rox_MatUT3 Kc, 
   const Rox_MatSE3 cTo, 
   const Rox_Point3D_Double mo,
   const Rox_Sint nbp
);

//! @}

#endif
