//==============================================================================
//
//    OPENROX   : File pyramid_uchar.h
//
//    Contents  : Structure of pyramid_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PYRAMID_UCHAR_STRUCT__
#define __OPENROX_PYRAMID_UCHAR_STRUCT__

#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Pyramid
//!	@{

//! Pyramid structure 
struct Rox_Pyramid_Uchar_Struct
{
   //! number of levels 
   Rox_Uint nb_levels;

   //! width of the first level 
   Rox_Uint base_width;

   //! height of the first level 
   Rox_Uint base_height;

   //! List of levels 
   Rox_Image * levels;

   //! Fast access to row pointers 
   Rox_Uchar *** fast_access;
};

//! @} 

#endif // __OPENROX_PYRAMID_UCHAR_STRUCT__
