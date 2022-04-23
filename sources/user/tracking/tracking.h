//==============================================================================
//
//    OPENROX   : File tracking.h
//
//    Contents  : API of tracking module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING__
#define __OPENROX_TRACKING__

#include "tracking_params.h"

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

#include <generated/array2d_float.h>

//! \addtogroup Tracking Tracking
//! \brief Structure and functions of the tracking
//! @{

//! The Rox_Tracking object is a pointer to the opaque structure Rox_Tracking_Struct
typedef struct Rox_Tracking_Struct * Rox_Tracking;

//! Define the _Rox_Tracking object. Useless, to be deleted, use directly Rox_Tracking_Struct instead
typedef struct Rox_Tracking_Struct   Rox_Tracking_Struct;

//! The Rox_Tracking_Struct is opaque, should be placed in a different tracking_struct.h file
struct Rox_Tracking_Struct
{
   //! The intermediate homography to move into the template pyramid
   Rox_MatSL3 zoom_homography;

   //! Homography matrix
   Rox_MatSL3 homography;

   //! The normalized template
   Rox_Array2D_Float normalized_ref;

   //! The normalized image
   Rox_Array2D_Float normalized_cur;

   //! The zncc score between the reference and the current warped template
   Rox_Double score;

   //! The threshold to decide if the template is tracked
   Rox_Double min_score;

   //! The prediction radius
   Rox_Sint prediction_radius;

   //! Maximal iterations of the ESM algorithm
   Rox_Sint miter;

   //! The higher pyramid level
   Rox_Sint init_pyr;

   //! The lower pyramid level
   Rox_Sint stop_pyr;

   //! Function pointer of the specific make function (depends on the defined usecase)
   Rox_ErrorCode (*_fptr_make)(Rox_Tracking, Rox_Image);

   //! Function pointer of the specific delete function (depends on the defined usecase)
   Rox_ErrorCode (*_fptr_del)(Rox_Tracking *);

   //! Function pointer of the specific set_mask function (depends on the defined usecase)
   Rox_ErrorCode (*_fptr_set_mask) (Rox_Tracking, Rox_Imask);
};

//! Create the tracking object
//! \param  [out]    tracking    The newly created object
//! \param  [in]     params      The tracking parameters
//! \param  [in]     model       The 2d model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_new (
   Rox_Tracking * tracking, 
   const Rox_Tracking_Params params, 
   const Rox_Image model
);

//! Delete the tracking object
//! \param  [in]     tracking tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_del (
   Rox_Tracking * tracking
);

//! Allocate the internal data of the structure tracking
//! \param  [out]    tracking    The newly created object
//! \param  [in]     params      The tracking parameters
//! \param  [in]     model       The 2d model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_alloc (
   Rox_Tracking tracking, 
   const Rox_Tracking_Params params, 
   const Rox_Image model
);

//! Release the allocated memory
//! \param  [in ] tracking The structure to be freed
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_free ( 
   Rox_Tracking tracking 
);

//! Make the tracking
//! \param  [in ]  tracking tracking object
//! \param  [in ]  image   the current image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_make ( 
   Rox_Tracking tracking, 
   const Rox_Image image 
);

//! Get the tracking score
//! \param  [out]  score   the tracking score
//! \param  [in ]  tracking the tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_get_score (
   Rox_Double * score, 
   const Rox_Tracking tracking
);

//! Set the prediction homography
//! \param  [in] tracking the tracking object
//! \param  [in] homography the homography
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_set_homography ( 
   Rox_Tracking tracking, 
   const Rox_MatSL3 homography
);

//! Set the threshold to decide if the tracking is lost
//! \param [in] tracking the tracking object
//! \param [in] score_thresh the decision threshold
//! \remarks score_thresh must be in the range [0,1]
//! \remarks default value is score_thresh = 0.89
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_tracking_set_score_thresh (
   Rox_Tracking tracking, 
   const Rox_Double score_thresh
);

//! Set the maximum number of iterations
//! \param  [in ]  tracking       The tracking object
//! \param  [in ]  miter          The maximum number of iterations
//! \return An error code
//! \remarks default value is miter = 10
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_set_miter (
   Rox_Tracking tracking, 
   const Rox_Sint miter
);

//! Set the template mask
//! \param  [in ]  tracking       The tracking object
//! \param  [in ]  mask           The template mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_set_mask(
   Rox_Tracking tracking, 
   const Rox_Imask mask
);

//! Get the homography
//! \param  [out]  homography     Homography matrix in SL3
//! \param  [in ]  tracking       The tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_get_homography ( 
   Rox_MatSL3 homography, 
   const Rox_Tracking tracking
);

//! @}

#endif // __OPENROX_TRACKING__
