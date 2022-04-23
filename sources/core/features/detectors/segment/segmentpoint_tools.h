//==============================================================================
//
//    OPENROX   : File segmentpoint_tools.h
//
//    Contents  : API of segmentpoint_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENTPOINT_TOOLS__
#define __OPENROX_SEGMENTPOINT_TOOLS__

#include <generated/dynvec_sint.h>
#include <generated/dynvec_segment_point.h>
#include <baseproc/image/image.h>

//! \addtogroup Segment_Point
//! @{

//! Given a list of point, create a row look up table. This look up table have the size of the number of rows in the source image.
//! It either contains the index of the first feature in this row or -1 if no feature found in this row
//! \param  [out]  lut the result look up table
//! \param  [in ]  input the list of points to use
//! \param  [in ]  image_count_rows the image height. If a points has a coordinates row superior than image_count_rows, an error will be thrown.
//! \return An error code.
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment_points_make_lut(Rox_DynVec_Sint lut, Rox_DynVec_Segment_Point input, Rox_Uint image_count_rows);

//! Given a list of points, compute for each of them their shi-tomasi corner responsa and store it as its score.
//! If the points is too close to the image border such that some pixel in the window are outside, this point's score will be set to 0.
//! \param  [in ]  points the list of points
//! \param  [in ]  image the image to compute response on.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment_points_compute_shi_score(Rox_DynVec_Segment_Point points, Rox_Image image);

//! Given a list of points, compute for each of them their shi-tomasi corner response and store it as its score.
//! If the points is too close to the image border such that some pixel in the window are outside, this point's score will be set to 0.
//! \param  [in ]  points the list of points
//! \param  [in ]  image the image to compute response on.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment_points_compute_shi_score9x9(Rox_DynVec_Segment_Point points, Rox_Image image);

//! Given a list of points, sort it by decreasing response.
//! \param  [in ]  points the list of points.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment_points_sort_response ( Rox_DynVec_Segment_Point points );

ROX_API Rox_ErrorCode rox_segment_points_print ( Rox_DynVec_Segment_Point points );

//! @} 

#endif
