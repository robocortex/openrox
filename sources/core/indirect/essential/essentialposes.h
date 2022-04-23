//==============================================================================
//
//    OPENROX   : File essentialposes.h
//
//    Contents  : API of essentialposes module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ESSENTIAL_POSES__
#define __OPENROX_ESSENTIAL_POSES__

#include <generated/array2d_double.h>

//! \ingroup Geometry
//! \addtogroup Essential
//! @{

//! Given an essential matrix, extract the four possible poses
//! \param  []  poses            an array of 4 poses which contains the possible poses
//! \param  []  essential        an essential (3x3) matrix
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_essential_possible_poses(Rox_Array2D_Double * poses, Rox_Array2D_Double essential);

//! @}

#endif
