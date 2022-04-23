//==============================================================================
//
//    OPENROX   : File line3d_print.h
//
//    Contents  : API of line3d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE3D_PRINT__
#define __OPENROX_LINE3D_PRINT__

#include <baseproc/geometry/line/line3d.h>

//! \addtogroup Line3D
//! @{

//! Print a 3D line planes double on stdout
//! \param  [in ]  line3d_planes  The line to print
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_print ( const Rox_Line3D_Planes line3d_planes );

//! @}

#endif
