//===============================================================================
//
//    OPENROX   : File identification.h
//
//    Contents  : API of identification module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENTIFICATION__
#define __OPENROX_IDENTIFICATION__

#include <user/tracking/tracking.h>
#include <user/identification/texture/ident_texture_sl3.h>

//! \addtogroup Identification
//!   @{
//!   \todo Replace Rox_Identification by ?? (robust, accurate...)

//! The Rox_Identification_Struct object 
struct Rox_Identification_Struct
{
   //! The SL3 identifier 
   Rox_Ident_Texture_SL3 identifier;

   //! The tracker to refine the estimated homography from the detection step 
   Rox_Tracking  tracker;
};

//! Define the pointer of the Rox_Identification_Struct 
typedef struct Rox_Identification_Struct * Rox_Identification;

//! Constructor of Rox_Identification
//! \param [in] ident The identification object
//! \param [in] model The model image
//! \return An error code
//! \todo to be tested

ROX_API Rox_ErrorCode rox_identification_new(Rox_Identification* ident, Rox_Image model);

//! Destructor of Rox_Identification
//! \param [in] ident  Rox_Identification object to be deleted
//! \return An error code
//! \todo to be tested

ROX_API Rox_ErrorCode rox_identification_del(Rox_Identification* ident);

//! Make the template detection
//! \param [in] ident  Rox_Identification object
//! \param [in] image  the image to detect template into
//! \return An error code
//! \todo to be tested

ROX_API Rox_ErrorCode rox_identification_make(Rox_Identification ident, Rox_Image image);

//! Get the identification result
//! \param [out] H The homography matrix
//! \param [in] ident  Rox_Identification object
//! \return An error code
//! \todo to be tested

ROX_API Rox_ErrorCode rox_identification_get_homography(Rox_MatSL3 H, Rox_Identification ident);

//! @} 

#endif // __OPENROX_IDENTIFICATION__ 
