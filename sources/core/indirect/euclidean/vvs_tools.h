//==============================================================================
//
//    OPENROX   : File vvs_tools.h
//
//    Contents  : API of vvs_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VVS_TOOLS__
#define __OPENROX_VVS_TOOLS__

#include <baseproc/maths/linalg/matut3.h>

#include <system/errors/errors.h>

//! Convert a distance expressed in pixel units to a distance expressed in normalized units
//! given an intrinsic parameters matrix
//! \param  [out]  dist_norm  resulting normalized units distance
//! \param  [in ]  dist_pix   pixel units distance to convert
//! \param  [in ]  K          intrinsic parameters matrix
//! \todo   To be tested
ROX_API Rox_ErrorCode conv_dist_pixel_to_normalized
( 
   Rox_Double * dist_norm, 
   Rox_Double  dist_pix, 
   Rox_MatUT3 K
);

#endif
