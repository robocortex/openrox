//==============================================================================
//
//    OPENROX   : File objset_dynvec_point2d_print.h
//
//    Contents  : API of point2d_print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OBJSET_dynvec_point2d_print__
#define __OPENROX_OBJSET_dynvec_point2d_print__

#include <generated/objset_dynvec_point2d_float.h>
#include <generated/objset_dynvec_point2d_double.h>
// #include <objset_dynvec_point2d_uint.h>

//! \addtogroup Point2D
//! @{

//! Display a 2D points dynamic vector of float on stdout
//! \param  [in ]  dynvec_point2D 		The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_dynvec_point2d_float_print ( Rox_ObjSet_DynVec_Point2D_Float dynvec_point2D);

//! Display a 2D points dynamic vector of double on stdout
//! \param  [in ]  dynvec_point2D 		The array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_dynvec_point2d_double_print ( Rox_ObjSet_DynVec_Point2D_Double dynvec_point2D);

// //! Display a 2D points dynamic vector of unit on stdout
// //! \param  [in]  dynvec_point2D     The array to print
// //! \return An error code
// ROX_API Rox_ErrorCode rox_objset_dynvec_point2d_uint_print ( Rox_ObjSet_DynVec_Point2D_Uint dynvec_point2D);

//! @} 

#endif
