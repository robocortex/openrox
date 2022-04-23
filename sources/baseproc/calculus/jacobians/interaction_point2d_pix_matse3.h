//==============================================================================
//
//    OPENROX   : File interaction_matse3_point2d_pix.h
//
//    Contents  : API of interaction_matse3_point2d_pix module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERACTION_MATSE3_POINT2D_PIX__
#define __OPENROX_INTERACTION_MATSE3_POINT2D_PIX__

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>

//! \addtogroup Calibration_Jacobians
//! @{

//! Imteraction matrix L for 2d point in pixels
//! d p / dt = L * v
//! p = [u; v] are the coordinates od a 2D point in pixels 
//! v = [vt ; vr] is the velocity
//! \param  [out]  L              The interaction matrix
//! \param  [in ]	 pts            The 2d points
//! \param  [in ]	 z              The depth of the point in the current frame
//! \param  [in ]	 K              The camera intrinsic parameters
//! \param  [in ]	 count
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_interaction_matse3_point2d_pix (
   Rox_Matrix L, 
   const Rox_Point2D_Double pts, 
   const Rox_Double * z, 
   const Rox_MatUT3 K, 
   const Rox_Sint count
);

//! @}

#endif // __OPENROX_INTERACTION_MATSE3_POINT2D_PIX__
