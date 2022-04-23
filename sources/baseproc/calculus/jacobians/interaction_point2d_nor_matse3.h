//==============================================================================
//
//    OPENROX   : File interaction_matse3_point2d_nor.h
//
//    Contents  : API of interaction_matse3_point2d_nor module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERACTION_MATSE3_POINT2D_NOR__
#define __OPENROX_INTERACTION_MATSE3_POINT2D_NOR__

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matrix.h>

//! \addtogroup Calibration_Jacobians
//! @{

//! Jacobian for 2d point in meters
//! d p / dt = L * v 
//! \remark It is an Interation matrix (J = L) !!!
//! \param  [out]	 L
//! \param  [in ]	 q              Coordinat
//! \param  [in ]	 z
//! \param  [in ]	 count
//! \return An error code
//! \todo   To be tested           
//! \todo   The function should be named "rox_interaction_matse3_point2d_nor"
ROX_API Rox_ErrorCode rox_interaction_matse3_point2d_nor (
   Rox_Matrix L, 
   const Rox_Point2D_Double q, 
   const Rox_Double * z, 
   const Rox_Sint count
);

//! @}

#endif // __OPENROX_INTERACTION_MATSE3_POINT2D_NOR__
