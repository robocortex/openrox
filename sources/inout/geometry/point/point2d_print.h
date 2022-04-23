//==============================================================================
//
//    OPENROX   : File point2d_print.h
//
//    Contents  : API of point2d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT2D_PRINT__
#define __OPENROX_POINT2D_PRINT__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point2d_struct.h>

//! \addtogroup Point2D
//! @{

//! Display a 2D point float on stdout
//! \param  [in ] 	point2D 		The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_float_print ( const Rox_Point2D_Float point2D);

//! Display a 2D point double on stdout
//! \param  [in ] 	point2D 		The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_double_print ( const Rox_Point2D_Double point2D );

//! Display a 2D point uint on stdout
//! \param  [in ]  point2D     The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_uint_print ( const Rox_Point2D_Uint point2D );

//! Display a 2D point uint on stdout
//! \param  [in ]   point2D     The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_sint_print ( const Rox_Point2D_Sint point2D );

//! Display a vector of 2D point double on stdout
//! \param  [in ]  points2D       The array to print
//! \param  [in ]  nbpoints
//! \return An error code
ROX_API Rox_ErrorCode rox_vector_point2d_double_print ( const Rox_Point2D_Double points2D, const Rox_Sint nbpoints);

//! @}

#endif
