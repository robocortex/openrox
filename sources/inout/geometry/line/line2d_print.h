//==============================================================================
//
//    OPENROX   : File line2d_print.h
//
//    Contents  : API of line2d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE2D_PRINT__
#define __OPENROX_LINE2D_PRINT__

#include <baseproc/geometry/line/line2d.h>

//! \addtogroup Point2D
//! @{

//! Display a 2D line double on stdout
//! \param  [in ] 	line2d_normal 		 The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_normal_print ( const Rox_Line2D_Normal line2d_normal);

//! Display a vector of 2D line double on stdout
//! \param  [in ] 	lines2d_normal 		 The array to print
//! \param  [in ] 	nblines
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_normal_vector_print ( Rox_Line2D_Normal lines2d_normal, const Rox_Sint nblines);

//! @}

#endif
