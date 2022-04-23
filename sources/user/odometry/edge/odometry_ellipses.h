//==============================================================================
//
//    OPENROX   : File odometry_ellipses.h
//
//    Contents  : API of odometry_ellipses module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_ELLIPSES__
#define __OPENROX_ODOMETRY_ELLIPSES__

#include <generated/objset_edge_ellipse.h>
#include <generated/objset_edge_ellipse_struct.h>

#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/image.h>

#include <core/tracking/edge/tracking_ellipse.h>
#include <core/tracking/edge/edge_ellipse.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Ellipses
//! @{

//! 3D Segments based odometry structure
struct Rox_Odometry_Ellipses_Struct
{
   //! odometry estimated pose 
   Rox_MatSE3 pose;

   //! odometry virtual camera calibration 
   Rox_MatUT3 calibration;

   //! Set of ellipses 
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse;

   //! Edge tracker 
   Rox_Tracking_Ellipse tracker;
};

//! CAD Model based odometry structure
typedef struct Rox_Odometry_Ellipses_Struct * Rox_Odometry_Ellipses;

//! Create a new odometry with 3D ellipses model constraint object
//! \param  [out]  odometry_ellipses      The pointer to the newly created object
//! \param  [in ]  search_range           The distance in number of pixels to search from each site
//! \param  [in ]  contrast_threshold     The contrast threshold to validate a convolution result
//! \param  [in ]  sampling_step          The step when sampling ellipses
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_ellipses_new (
   Rox_Odometry_Ellipses * odometry_ellipses,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete an odometry object with 3D ellipses model constraint
//! \param  [out]  odometry_ellipses    The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_ellipses_del (
   Rox_Odometry_Ellipses * odometry_ellipses
);

//! Append a 3D ellipse to the odometry object
//! \param  [out]  odometry_ellipses       The odometry object
//! \param  [in ]  ellipse                 The 3D ellipse to add in world coordinates
//! \param  [in ]  sampling_step           The sampling step for an ellipse
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_ellipses_add_ellipse (
   Rox_Odometry_Ellipses odometry_ellipses, 
   const Rox_Ellipse3D ellipse3d, 
   const Rox_Double sampling_step
);

//! Compute the pose to match the ellipses in the image
//! \param  [out]  odometry_ellipses       The odometry object
//! \param  [in ]  image                   The image to track into
//! \param  [in ]  max_iters               The number of iteration to make vvs estimation
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_ellipses_make ( 
   Rox_Odometry_Ellipses odometry_ellipses, 
   const Rox_Image image, 
   const Rox_Sint max_iters
);

//! Compute the score (mean distance in pixels: 0 is the best score)
//! \param  [out]  score                   The score
//! \param  [in ]  odometry_ellipses       The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_ellipses_get_score (
   Rox_Double * score, 
   Rox_Odometry_Ellipses odometry_ellipses 
);

ROX_API Rox_ErrorCode rox_odometry_ellipses_get_valid_sites (
   Rox_DynVec_Point2D_Double dynvec_point2d, 
   Rox_Odometry_Ellipses odometry_ellipses
);

//! @} 

#endif
