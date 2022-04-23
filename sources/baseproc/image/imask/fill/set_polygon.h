//==============================================================================
//
//    OPENROX   : File set_polygon.h
//
//    Contents  : API of set_polygon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MASK_SET_POLYGON__
#define __OPENROX_MASK_SET_POLYGON__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup  Mask
//! \addtogroup MaskSetEllipse
//! @{

//!  Set the mask with a polygon shape defined by 2D points
//! \param  [out]  mask  	       The mask to set
//! \param  [in ]  pts  		    The array containing all 2D points coordinates
//! \param  [in ]  nbpts  	       The number of points contained in "pts" array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_set_polygon ( Rox_Imask mask , Rox_Point2D_Double  pts , Rox_Sint nbpts);

ROX_API Rox_ErrorCode rox_array2d_uint_new_polygon ( Rox_Imask * mask, const Rox_Point2D_Double points_list, Rox_Sint const nb_points );

ROX_API Rox_ErrorCode rox_bounding_box_get_size ( Rox_Sint * u, Rox_Sint * v, Rox_Sint * rows, Rox_Sint * cols, Rox_Point2D_Double bounding_box );

//! @} 

#endif // __OPENROX_MASK_SET_ELLIPSE__
