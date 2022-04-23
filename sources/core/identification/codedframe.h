//==============================================================================
//
//    OPENROX   : File codedframe.h
//
//    Contents  : API of codedframe module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CODEDFRAME__
#define __OPENROX_CODEDFRAME__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>

//! \ingroup Identification_Photoframe
//! \addtogroup Coded_Frame
//! @{

//! The codedframe stucture 
struct Rox_CodedFrame_Struct
{
   //! Decoded value 
   Rox_Uint value;

   //! Homography shift 
   Rox_MatSL3 G;

   //! Code Homography 
   Rox_MatSL3 cH;
};

//! Pointer to a codedframe structure 
typedef struct Rox_CodedFrame_Struct * Rox_CodedFrame;

//! Create a new object for coded frame identification
//! \param [out] codedframe	The codedframe object to be created
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_codedframe_new(Rox_CodedFrame * codedframe);

//! Delete a codedframe object
//! \param [in] codedframe	The codedframe object to be deleted
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_codedframe_del(Rox_CodedFrame * codedframe);

//! Extract code from frame coded on 64 bits
//! \param [in]   codedframe  The created codedframe object
//! \param [in]   image       The image to extract the template from
//! \param [in]   homography  The homography localizing the template (inside the code frame).
//!                           H is originally meant to warp the interior of the template into a 128x128 image.
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_codedframe_make64(Rox_CodedFrame codedframe, Rox_Image image, Rox_MatSL3 homography);

//! Extract code from frame coded on 16 bits
//! \param [in]   codedframe  The created codedframe object
//! \param [in]   image       The image to extract the template from
//! \param [in]   homography  The homography localizing the template (inside the code frame).
//!                           H is originally meant to warp the interior of the template into a 128x128 image.
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_codedframe_make16(Rox_CodedFrame codedframe, Rox_Image image, Rox_MatSL3 homography);

//! Get the code from codedframe
//! \param [out]  value       The image to extract the template from
//! \param [in]   codedframe 	The created codedframe object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_codedframe_get_value(Rox_Sint *value, Rox_CodedFrame codedframe);

//! @} 

#endif
