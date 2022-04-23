//==============================================================================
//
//    OPENROX   : File bundle_frame.h
//
//  	Contents  : API of bundle_frame module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_FRAME__
#define __OPENROX_BUNDLE_FRAME__

#include <system/memory/datatypes.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! Bundle object
typedef struct Rox_Bundle_Frame_Struct * Rox_Bundle_Frame;

//! Create a container object for a bundle frame
//! \param  []  bundle_frame the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_frame_new ( Rox_Bundle_Frame * bundle_frame );

//! Delete a container object for a bundle frame
//! \param  []  bundle_frame the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_frame_del ( Rox_Bundle_Frame * bundle_frame );

//! Compute hessian of a bundle frame
//! \param  []  bundle_frame the container pointer to use
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_frame_compute_hessian (Rox_Bundle_Frame bundle_frame );

//! @} 

#endif // __OPENROX_BUNDLE_FRAME__
