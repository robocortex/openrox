//==============================================================================
//
//    OPENROX   : File pyramid_uint.h
//
//    Contents  : API of pyramid_uint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PYRAMID_UINT__
#define __OPENROX_PYRAMID_UINT__

#include <generated/array2d_uint.h>

//! \ingroup Vision
//! \addtogroup Pyramid
//!	@{

//! Pyramid structure 
struct Rox_Pyramid_Uint_Struct
{
   //! number of levels 
   Rox_Uint nb_levels;

   //! width of the first level 
   Rox_Uint base_width;

   //! height of the first level 
   Rox_Uint base_height;

   //! List of levels 
   Rox_Array2D_Uint * levels;

   //! Fast access to row pointers 
   Rox_Uint *** fast_access;
};

//! Pyramid structure 
typedef struct Rox_Pyramid_Uint_Struct * Rox_Pyramid_Uint;

//! Create a new pyramid object
//! \param  [out] obj               The pointer to the newly created object
//! \param  [in]  width             The width of the base level
//! \param  [in]  height            The height of the base level
//! \param  [in]  max_levels        Maximum number of levels (-1 = automatic)
//! \param  [in]  min_size          Minimal dimension of a level (no level with less than this size will be created)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uint_new(Rox_Pyramid_Uint * obj, Rox_Sint width, Rox_Sint height, Rox_Uint max_levels, Rox_Uint min_size);

//! Delete a pyramid object
//! \param  [in]  obj               The pointer to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uint_del(Rox_Pyramid_Uint * obj);

//! Apply pyramid on an image using nearest neighboor subsampling
//! \param  [in]  obj               The pyramid object
//! \param  [in]  source            The image to subsample
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uint_assign_nofiltering(Rox_Pyramid_Uint obj, Rox_Array2D_Uint source);

//! @} 

#endif // __OPENROX_PYRAMID_UINT_OBJECT__
