//==============================================================================
//
//    OPENROX   : File ocm.h
//
//    Contents  : API of ocm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OCM__
#define __OPENROX_OCM__

#include <generated/array2d_sshort.h>
#include <generated/dynvec_point2d_sshort.h>

//! \ingroup Vision
//! \addtogroup CHAMFER
//! @{

//! Compute the chamfer matching score using the equation of "Fast Detection of 3-D Multiple Objects Textureless" by Matas
//! \param  [out]  score               the result chamfer matching score
//! \param  [in ]  x                   top left coordinates of the interest window in the current image (Size is the template size)
//! \param  [in ]  y                   top left coordinates of the interest window in the current image (Size is the template size)
//! \param  [in ]  edges               template list of edges
//! \param  [in ]  anglemap_template   template angles map of edges
//! \param  [in ]  anglemap_image      image angles map of edges
//! \param  [in ]  distancemap         distance map of the current image
//! \param  [in ]  lambda              how to weight small objects
//! \param  [in ]  mean_edges          the mean number of edges over the training images
//! \param  [in ]  max_dist
//! \param  [in ]  max_angle
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_ocm_cardinal_process(
   Rox_Double * score, 
   Rox_Sshort x, 
   Rox_Sshort y, 
   Rox_DynVec_Point2D_Sshort edges, 
   Rox_Array2D_Sshort anglemap_template, 
   Rox_Array2D_Sshort anglemap_image, 
   Rox_Array2D_Sshort distancemap, 
   Rox_Double lambda, 
   Rox_Double mean_edges, 
   Rox_Sint max_dist, 
   Rox_Double max_angle
);

//! @} 

#endif
