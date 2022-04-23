//==============================================================================
//
//    OPENROX   : File pyramid_float.h
//
//    Contents  : API of pyramid_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PYRAMID_FLOAT__
#define __OPENROX_PYRAMID_FLOAT__

#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Pyramid
//! @{

//! Pyramid structure 
struct Rox_Pyramid_Float_Struct
{
   //! number of levels 
   Rox_Uint nb_levels;

   //! width of the first level 
   Rox_Uint base_width;

   //! height of the first level 
   Rox_Uint base_height;

   //! List of levels 
   Rox_Image_Float * levels;

   //! Fast access to row pointers 
   Rox_Float *** fast_access;
};

//! Pyramid structure 
typedef struct Rox_Pyramid_Float_Struct * Rox_Pyramid_Float;

//! Create a new pyramid object
//! \param  [out] pyramid           The pointer to the newly created object
//! \param  [in ]  width             The width of the base level
//! \param  [in ]  height            The height of the base level
//! \param  [in ]  max_levels        Maximum number of levels (-1 = automatic)
//! \param  [in ]  min_size          Minimal dimension of a level (no level with less than this size will be created)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_new(Rox_Pyramid_Float * pyramid, Rox_Sint width, Rox_Sint height, const Rox_Sint max_levels, const Rox_Sint min_size);

//! Delete a pyramid object
//! \param  [out] pyramid            The pointer to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_del(Rox_Pyramid_Float * pyramid);

//! Apply pyramid on an image using nearest neighboor subsampling
//! \param  [in] pyramid            The pyramid object
//! \param  [in] source             The image to subsample
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_assign_nofiltering(Rox_Pyramid_Float pyramid, const Rox_Image_Float source);

//! Apply pyramid on an image using box filter subsampling
//! \param  [in] pyramid            The pyramid object
//! \param  [in] source             The image to subsample
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_assign(Rox_Pyramid_Float pyramid, const Rox_Image_Float source);

//! Apply pyramid on an image using gaussian subsampling
//! \param  [in] pyramid            The pyramid object
//! \param  [in] source             The image to subsample
//! \param  [in] sigma              The gaussian sigma
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_assign_gaussian(Rox_Pyramid_Float pyramid, const Rox_Image_Float source, const Rox_Float sigma);

//! Get the pointer to the image corresponding to the desired level of the pyramid
//! \param  [out]  image          The pointer to the image float object at level "level"
//! \param  [in ]   pyramid       The pyramid object
//! \param  [in ]   level         The desired level of the pyramid
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_get_image ( Rox_Image_Float * image, const Rox_Pyramid_Float pyramid, const Rox_Sint level);

//! Get the number of levels of the pyramid
//! \param  [out]  nb_levels        The number of levels
//! \param  [in]   pyramid          The pyramid object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_float_get_nb_levels(Rox_Sint * nb_levels, const Rox_Pyramid_Float pyramid);

//! @} 

#endif // __OPENROX_PYRAMID_FLOAT__
