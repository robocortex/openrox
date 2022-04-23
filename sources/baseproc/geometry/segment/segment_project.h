//==============================================================================
//
//    OPENROX   : File segment_project.h
//
//    Contents  : API of segment_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENT_PROJECT__
#define __OPENROX_SEGMENT_PROJECT__

#include <generated/array2d_double.h>

#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/segment/segment3d.h>

//! \addtogroup segment
//! @{

//! Project a 3D segment into a 2D segment on the image plane
//! \param [out]  segment2d      The output 2d segment
//! \param [in]   segment3d      The input  3d segment
//! \param [in]   matct2
//! \return An error code
ROX_API Rox_ErrorCode rox_segment2d_project_segment3d(Rox_Segment2D segment2d, Rox_Array2D_Double matct2, Rox_Segment3D segment3d);

//! Project a 3D segment into a 2D segment on the image plane
//! \param [out]  segment2d      The output 2d segment
//! \param [in]   segment3d      The input  3d segment
//! \param [in]   matct2
//! \param [in]   matse3
//! \return An error code
ROX_API Rox_ErrorCode rox_segment2d_transform_project_segment3d(Rox_Segment2D segment2d, Rox_Array2D_Double matct2, Rox_Array2D_Double matse3, Rox_Segment3D segment3d);

//! @} 

#endif
