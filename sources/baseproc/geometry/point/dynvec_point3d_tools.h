//==============================================================================
//
//    OPENROX   : File dynvec_points3d_tools.h
//
//    Contents  : API of dynvec points3d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_POINTS3D_TOOLS__
#define __OPENROX_DYNVEC_POINTS3D_TOOLS__

#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_float.h>

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/maths/linalg/matrix.h>

//! \addtogroup Point3D
//! @{

//! Set data
//! \param  [out]  points3D       The dynvec of points3D to be set
//! \param  [in ]  data_points3D  The pointer to a buffer of points3D structures
//! \param  [in ]  numb_points3D  The number of points3D structures in the buffer
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_set_data ( 
   Rox_DynVec_Point3D_Double points3D,
   const Rox_Double * data_points3D,
   const Rox_Sint     numb_points3D 
);


//! Set the data of 3D points data = [ X1, Y1, Z1, X2, Y2, Z2, ..., Xn, Yn, Zn ]
//! \param  [out]  points3D
//! \param  [in ]  data_points3D
//! \param  [in ]  numb_points3D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point3d_float_set_data (
   Rox_DynVec_Point3D_Float points3D, 
   Rox_Float * data_points3D, 
   Rox_Sint numb_points3D
);

ROX_API Rox_ErrorCode rox_dynvec_point3d_double_set_data_vector (
   Rox_DynVec_Point3D_Double dynvec_points3D,
   const Rox_Point3D_Double  vector_points3D,
   const Rox_Sint numb_points3D
);

//! Append a vector of points 3D to a dynvec of points 3D
//! \param  [out]  points3D
//! \param  [in ]  points3D_struct
//! \param  [in ]  numb_points3D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_append_vector_point3d_double (
   Rox_DynVec_Point3D_Double dynvec_points3D, 
   Rox_Point3D_Double points3D, 
   Rox_Sint numb_points3D
);

//!
//! \param  [out]  points3D
//! \param  [in ]  points3D_struct_double
//! \param  [in ]  numb_points3D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point3d_float_append_vector_point3d_double (
   Rox_DynVec_Point3D_Float points3D, 
   Rox_Point3D_Double_Struct * points3D_struct_double, 
   Rox_Sint numb_points3D
);


//! Get a point 3d from a dynamic vector of points 3d
//! \param [out]  points3D_vector : The dynvec of points3D to be set
//! \param [in]   numb_points3D   : The number of points3D structures in the buffer
//! \param [in]   points3D_dynvec :
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_get_point3d ( 
   Rox_Point3D_Double points3D_vector,
   Rox_Sint                  * numb_points3D,
   Rox_DynVec_Point3D_Double   points3D_dynvec 
);


//! Append a point 3d to a dynamic vector of points 3d
//! \param  [out]  model          The dynvec of points3D to be set
//! \param  [in ]  model_size_x   The pointer to a buffer of points3D structures
//! \param  [in ]  model_size_y   The number of points3D structures in the buffer
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_float_append_rectangle ( 
   Rox_DynVec_Point3D_Float model, 
   Rox_Float model_size_x, 
   Rox_Float model_size_y 
);

ROX_API Rox_ErrorCode rox_dynvec_point3d_double_append_rectangle_newframe ( 
   Rox_DynVec_Point3D_Double model, 
   const Rox_Double model_size_x, 
   const Rox_Double model_size_y 
);

//! Append
//! \param  [out]  model          The dynvec of points3D to be set
//! \param  [in ]  model_size_x   The pointer to a buffer of points3D structures
//! \param  [in ]  model_size_y   The number of points3D structures in the buffer
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_append_rectangle ( 
   Rox_DynVec_Point3D_Double model, 
   Rox_Double model_size_x, 
   Rox_Double model_size_y 
);

//! Create a new rectagle model on the x-y playe (z=0)
//! \param  [out]  model          The newly created planar model
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_new_rectangle ( 
   Rox_DynVec_Point3D_Double * model, 
   const Rox_Double size_x, 
   const Rox_Double size_y 
);


//! Normlize points in the dynvec
//! \param  [out]  point3d_dynvec The dynvec of 3d points to be normlized
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_normalize_unit ( Rox_DynVec_Point3D_Double point3d_dynvec );


//! Compute the mean of a set of Point3D
//! \param  [out]  pt_mean        The mean point
//! \param  [in ]  pts_vec        The dynvec of 3d points to considerf
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_mean ( 
   Rox_Point3D_Double pt_mean, 
   Rox_DynVec_Point3D_Double pts_vec 
);


//! Translate a set of Point3D
//! \param  [out]  pts_vec        The dynvec of 3d points to translate
//! \param  [in ]  pt_move        The translation to apply
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_shift ( Rox_DynVec_Point3D_Double pts_vec, Rox_Point3D_Double pt_move );


//! Center and normalize a set of Point3D around its barycenter
//! \param  [out]  pts_dst        The dynvec of 3d points centered and normalized
//! \param  [in ]  pts_src        The dynvec of 3d points to center and normalize
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_center_normalize ( Rox_DynVec_Point3D_Double pts_dst, Rox_DynVec_Point3D_Double pts_src );


//! Fill a dynvec of point3d double from a matrix size 3xN ( you can make a subarray if your matrix is 4xN )
//! \param  [out]  pts_dst        The dynvec of 3d points
//! \param  [in ]  pts_src        The matrix of points coordinates
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_from_matrix( Rox_DynVec_Point3D_Double pts_dst, Rox_Matrix pts_src );


//! Fill a matrix size 3xN from a dynvec of point3d double
//! \param  [out]  pts_dst        The matrix of points coordinates
//! \param  [in ]  pts_src        The dynvec of 3d points
//! \return An error code
ROX_API Rox_ErrorCode rox_matrix_from_dynvec_point3d_double( Rox_Matrix pts_dst , Rox_DynVec_Point3D_Double pts_src );

//! @}

#endif
