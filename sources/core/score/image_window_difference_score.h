//==============================================================================
//
//    OPENROX   : File image_window_difference_score.h
//
//    Contents  : API of image window difference score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_WINDOW_DIFFERENCE_SCORE__
#define __OPENROX_IMAGE_WINDOW_DIFFERENCE_SCORE__

#include <generated/dynvec_double.h>
#include <generated/dynvec_rect_sint.h>
#include <generated/objset_imask.h>

#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/image/image.h>

#include <inout/system/errors_print.h>

//! Constructor for camera structure
//! \param  [out]  score          The output score 
//! \param  [in ]  window         The window in the reference frame in which compute the score
//! \param  [in ]  image_ref      The reference image
//! \param  [in ]  image_cur      The current image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_window_nomask_difference_get_score ( 
   Rox_Double * score, 
   const Rox_Rect_Sint window, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
);


//! \param  [out]  score_list     The output scores 
//! \param  [in ]  window_list    The windows in the reference frame in which compute the scores
//! \param  [in ]  image_ref      The reference image
//! \param  [in ]  image_cur      The current image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_windows_nomask_difference_get_scores ( 
   Rox_DynVec_Double score_list, 
   const Rox_DynVec_Rect_Sint window_list, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
);


ROX_API Rox_ErrorCode rox_image_window_difference_get_score ( 
   Rox_Double * score, 
   const Rox_Rect_Sint window, 
   const Rox_Imask imask, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
);


ROX_API Rox_ErrorCode rox_image_windows_difference_get_scores ( 
   Rox_DynVec_Double score_list, 
   const Rox_DynVec_Rect_Sint window_list, 
   const Rox_ObjSet_Imask imask_list, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
);


#endif // __OPENROX_IMAGE_WINDOW_DIFFERENCE_SCORE__ 
