//==============================================================================
//
//    OPENROX   : File segment_transform.h
//
//    Contents  : API of segment_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENT_TRANSFORM__
#define __OPENROX_SEGMENT_TRANSFORM__

#include <generated/array2d_double.h>

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/geometry/segment/segment3d.h>
#include <baseproc/geometry/segment/segment2d.h>

//! \addtogroup segment
//! @{

//! Transform a 3D segment into a 3D segment in a different euclidean frame (using a MatSE3 pose matrix)
//! \param  [out]  segment3d_out  The output 3d segment
//! \param  [in ]  out_T_inp      The pose
//! \param  [in ]  segment3d_inp  The input  3d segment
//! \return An error code
ROX_API Rox_ErrorCode rox_segment3d_transform ( Rox_Segment3D segment3d_out, Rox_MatSE3 out_T_inp, Rox_Segment3D segment3d_inp);

//! Transform a 2D segment into a 2D segment in a different projective frame (using a MatSL3 homography matrix)
//! \param  [out]  segment2d_out  The output 2d segment
//! \param  [in ]  out_H_inp      The homography
//! \param  [in ]  segment2d_inp  The input  2d segment
//! \return An error code
ROX_API Rox_ErrorCode rox_segment2d_transform ( Rox_Segment2D segment2d_out, Rox_MatSL3 out_H_inp, Rox_Segment2D segment2d_inp);

//! @} 

#endif
