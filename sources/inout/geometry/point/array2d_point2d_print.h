//==============================================================================
//
//    OPENROX   : File array2d_point2d_print.h
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

#ifndef __OPENROX_ARRAY2D_POINT2D_PRINT__
#define __OPENROX_ARRAY2D_POINT2D_PRINT__

#include <generated/array2d_point2d_double.h>
#include <generated/array2d_point2d_float.h>
#include <generated/array2d_point2d_sshort.h>

//! \addtogroup Point2D
//! @{

//! Display a 2D array on stdout
//! \param  [in ]  input          The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_point2d_double_print ( Rox_Array2D_Point2D_Double input );

//! Display a 2D array on stdout
//! \param  [in ]  input          The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_point2d_float_print ( Rox_Array2D_Point2D_Float input );

ROX_API Rox_ErrorCode rox_array2d_point2d_sshort_save ( const Rox_Char * filename, const Rox_Array2D_Point2D_Sshort input );

//! @}

#endif
