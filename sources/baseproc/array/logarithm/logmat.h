//==============================================================================
//
//    OPENROX   : File logmat.h
//
//    Contents  : API of logmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LOGMAT__
#define __OPENROX_LOGMAT__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup matrixlog
//! @{

//! Given a pose (the top left 3*3 subarray), compute a rotation axis/angle
//! \param [out] axis_x the x component of the axis
//! \param [out] axis_y the y component of the axis
//! \param [out] axis_z the z component of the axis
//! \param [out] angle the angle value
//! \param [in] rotation matrix
//! \return An error code
//! \todo To be tested, redundant relative to the function in matso3.h, should be pose 4x4 instead, should be renamed since specific 
ROX_API Rox_ErrorCode rox_array2d_double_logmat(Rox_Double *axis_x, Rox_Double *axis_y, Rox_Double *axis_z, Rox_Double *angle, Rox_Array2D_Double rotation);

//! @} 

#endif // __OPENROX_LOGMAT__
