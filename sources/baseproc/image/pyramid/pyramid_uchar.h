//==============================================================================
//
//    OPENROX   : File pyramid_uchar.h
//
//    Contents  : API of pyramid_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PYRAMID_UCHAR__
#define __OPENROX_PYRAMID_UCHAR__

#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Pyramid
//! @{

//! Pyramid structure 
typedef struct Rox_Pyramid_Uchar_Struct * Rox_Pyramid_Uchar;

//! Create a new pyramid object
//! \param  [out]  pyramid           the pointer to the newly created object
//! \param  [in ]  width             the width of the base level
//! \param  [in ]  height            the height of the base level
//! \param  [in ]  max_levels        maximum number of levels (-1 = automatic)
//! \param  [in ]  min_size          minimal dimension of a level (no level with less than this size will be created)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uchar_new(Rox_Pyramid_Uchar * pyramid, Rox_Sint width, Rox_Sint height, Rox_Uint max_levels, Rox_Uint min_size);

//! Delete a pyramid object
//! \param  [out]  pyramid           the pointer to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uchar_del(Rox_Pyramid_Uchar * pyramid);

//! Apply pyramid on an image using box filter subsampling
//! \param  [in ]  pyramid            the pyramid object
//! \param  [in ]  source             the image to subsample
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uchar_assign ( Rox_Pyramid_Uchar pyramid, const Rox_Image source);

//! Apply pyramid on an image using gaussian subsampling
//! \param  [in ]  pyramid            the pyramid object
//! \param  [in ]  source             the image to subsample
//! \param  [in ]  sigma              the gaussian sigma
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_pyramid_uchar_assign_gaussian ( Rox_Pyramid_Uchar pyramid, const Rox_Image source, const Rox_Float sigma);

//! Get the image corresponding to the desired level of the pyramid
//! \param  [out]  image            The image at level "level"
//! \param  [in ]  pyramid          The pyramid object
//! \param  [in ]  level            The desired level of the pyramid
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uchar_get_image(Rox_Image * image, const Rox_Pyramid_Uchar pyramid, const Rox_Sint level);

//! Get the number of levels of the pyramid
//! \param  [out]  nb_levels        The number of levels
//! \param  [in ]  pyramid          The pyramid object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_uchar_get_nb_levels(Rox_Sint * nb_levels, const Rox_Pyramid_Uchar pyramid);

//! @} 

#endif // __OPENROX_PYRAMID_UCHAR__
