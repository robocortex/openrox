//==============================================================================
//
//    OPENROX   : File plane3d_print.h
//
//    Contents  : API of plane3d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLANE3D_PRINT__
#define __OPENROX_PLANE3D_PRINT__

#include <baseproc/geometry/plane/plane_struct.h>

//! \addtogroup Line3D
//! @{

//! Print a 3D plane double on stdout
//! \param  [in ]  plane  The plane to print
//! \return An error code
ROX_API Rox_ErrorCode rox_plane3d_print ( const Rox_Plane3D_Double plane );

//! @}

#endif
