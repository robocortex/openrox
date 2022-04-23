//==============================================================================
//
//    OPENROX   : File bundle_point.h
//
//    Contents  : API of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_POINT__
#define __OPENROX_BUNDLE_POINT__

#include <generated/dynvec_bundle_measure.h>

#include <baseproc/geometry/point/points_struct.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! checkerboard object
typedef struct Rox_Bundle_Point_Struct * Rox_Bundle_Point;

//! Create a container object for a bundle point
//! \param  [out]  bundle_point the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_point_new(Rox_Bundle_Point * bundle_point);

//! Delete a container object for a bundle point
//! \param  [out]  bundle_point the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_point_del(Rox_Bundle_Point * bundle_point);

//! Compute hessian of a bundle frame
//! \param  [out]  bundle_point the container pointer to use
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_point_compute_hessian(Rox_Bundle_Point bundle_point);

//! @}

#endif
