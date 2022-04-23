//==============================================================================
//
//    OPENROX   : File bundle_point_struct.h
//
//    Contents  : API of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_POINT_STRUCT__
#define __OPENROX_BUNDLE_POINT_STRUCT__

#include <baseproc/geometry/point/points_struct.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_bundle_measure_struct.h>

//!   \ingroup Vision
//!   \addtogroup Bundle
//!   @{

//! Bundle structure
struct Rox_Bundle_Point_Struct
{
   //! Is this point invalid ? 
   Rox_Uint is_invalid;

   //! Column associated
   Rox_Uint pos_jacobian;

   //! Measured width this point
   Rox_DynVec_Bundle_Measure measures;

   //! hessian
   Rox_Array2D_Double hessian;

   //! hessian * error
   Rox_Array2D_Double projected_error;

   //! Intermediate Bundle buffer
   Rox_Array2D_Double tp;

   //! coordinates relative to global reference frame
   Rox_Point3D_Double_Struct coords;

   //! coordinates backup relative to global reference frame
   Rox_Point3D_Double_Struct coords_previous;

   //! Update storage
   Rox_Double update[3];
};

//! @}

#endif
