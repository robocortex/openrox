//==============================================================================
//
//    OPENROX   : File ident_texture_sl3.h
//
//    Contents  : API of ident_texture_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_TEXTURE_SL3__
#define __OPENROX_IDENT_TEXTURE_SL3__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>

#include <core/identification/templateident_sl3.h>

//! \ingroup Ident_Texture
//! \addtogroup Ident_Texture_SL3
//! @{

//! SL3 identifier structure 
struct Rox_Ident_Texture_SL3_Struct
{
   //! The SL3 internal identifier 
   Rox_Template_Ident_SL3 identifier;

   //! Flag to double or not the image size 
   Rox_Uint dbl_size;
};

//! SL3 identifier structure pointer
typedef struct Rox_Ident_Texture_SL3_Struct * Rox_Ident_Texture_SL3;

//! Create a new object for identification of a plane on SL(3)
//! \param [out]  ident_texture_sl3    The newly created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_sl3_new(Rox_Ident_Texture_SL3 * obj);

//! Delete an identification object
//! \param [in] ident_texture_sl3         The created identification object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_sl3_del(Rox_Ident_Texture_SL3 * ident_texture_sl3);

//! Set the current model to detect
//! \param [out] ident_texture_sl3        The identification object
//! \param [in] model_image               The template to detect
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_sl3_set_model(Rox_Ident_Texture_SL3 ident_texture_sl3, const Rox_Image model_image);

//! Perform identification on an image
//! \param [out] is_identified            A flag 
//! \param [out] homography               The identified homography
//! \param [out] ident_texture_sl3     The identification object
//! \param [in] current_image             The image to detect template into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_sl3_make(Rox_Sint * is_identified, Rox_MatSL3 homography, const Rox_Ident_Texture_SL3 ident_texture_sl3, const Rox_Image current_image);

//! Set to 1 the dbl_size flag
//! \param [out] ident_texture_sl3     The object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_sl3_enable_double_size(Rox_Ident_Texture_SL3 ident_texture_sl3);

//! @} 

#endif
