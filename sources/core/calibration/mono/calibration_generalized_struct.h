//==============================================================================
//
//    OPENROX   : File calibration_generalized_struct.h
//
//    Contents  : API of calibration_generalized module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_GENERALIZED_STRUCT__
#define __OPENROX_CALIBRATION_GENERALIZED_STRUCT__

#include <generated/array2d_point2d_float.h>

#include <generated/objset_dynvec_point_double_struct.h>
#include <generated/objset_array2d_double_struct.h>
#include <generated/dynvec_double_struct.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Camera_Calibration
//! \defgroup Calibration_Mono_Generalized Generalized Monocular Calibration

//! \addtogroup Calibration_Mono_Generalized
//! @{

//! The Rox_Calibration_Mono_Generalized_Struct object 
struct Rox_Calibration_Mono_Generalized_Struct
{
   //! Polynom order 
   Rox_Uint order;

   //! Image width 
   Rox_Sint width;

   //! Image height 
   Rox_Sint height;

   //! Estimation mean square error 
   Rox_Double mse;

   //! list of (list of points) per images 
   Rox_ObjSet_DynVec_Point_Double points;

   //! list of (list of points) per images, undistorted
   Rox_ObjSet_DynVec_Point_Double points_undistorted;

   //! Image center 
   Rox_Point2D_Double_Struct image_center;

   //! Per image camera pose 
   Rox_ObjSet_Array2D_Double viewposes;

   //! Polynomial coefficients 
   Rox_Array2D_Double coefficients;

   //! Polynomial coefficients intermediate buffer
   Rox_Array2D_Double coefficients_buf;

   //! Polynomial inverse coefficients 
   Rox_DynVec_Double coefficients_inv;

   //! Polynomial roots
   Rox_Array2D_Double roots;

   //! Coefficient a 
   Rox_Double coef_a;

   //! Coefficient b 
   Rox_Double coef_b;

   //! Coefficient c 
   Rox_Double coef_c;

   //! Coefficient d 
   Rox_Double coef_d;

   //! Coefficient e 
   Rox_Double coef_e;

   //! LUT 
   Rox_Array2D_Point2D_Float map;

   //! LUT mask 
   Rox_Imask mapmask;
};

//! @} 

#endif // __OPENROX_CALIBRATION_GENERALIZED_STRUCT__
