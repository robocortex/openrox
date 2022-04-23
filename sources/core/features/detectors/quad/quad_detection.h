//==============================================================================
//
//    OPENROX   : File quad_detection.h
//
//    Contents  : API of quad_detection module
//                Detection of quadrilaterals in an image
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUAD_DETECTION__
#define __OPENROX_QUAD_DETECTION__

#include <generated/dynvec_quad.h>
#include <generated/dynvec_quad_segment2d.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/maths/linalg/matsl3.h>

//! \ingroup Vision
//! \addtogroup Quad
//! @{

//! To be commented
typedef struct Rox_QuadDetector_Struct * Rox_QuadDetector;

//! Create a new quad detector object
//! \param  [out]  detector       The quad detector
//! \param  [in ]  imagewidth     The image width
//! \param  [in ]  imageheight    The image height
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_new ( 
   Rox_QuadDetector * detector, const Rox_Uint imagewidth, const Rox_Uint imageheight);

//! Delete the quad detector object
//! \param  [out]  detector       The quad detector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_del ( Rox_QuadDetector *detector);

//! Process image to detect quads with the standard method
//! \param  [out]  detector       The quad detector
//! \param  [in ]  image          The image
//! \param  [in ]  mask           The image mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_process_image ( 
   Rox_QuadDetector detector, Rox_Image image, Rox_Imask mask);

//! Process image to detect quads with the _ac method
//! \param  [out]  detector       The quad detector
//! \param  [in ]  image          The image
//! \param  [in ]  mask           The image mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_process_image_ac ( 
   Rox_QuadDetector detector, Rox_Image image, Rox_Imask mask);

//! Set the type of quad starting from quad center: white to black or black to white (default)
//! \param  [out]  detector          The quad Detector
//! \param  [in ]  black_to_white    Set to 0 to detect white to black quads, set to 1 to detect black to white quads, 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_set_quad_color ( 
   Rox_QuadDetector detector, 
   const Rox_Uint black_to_white);

//! Set the type of quad starting from quad center: white to black or black to white (default)
//! \param  [out]  detector               The quad Detector
//! \param  [in ]  segment_length_min     The minimum legnth of a segment 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_set_segment_length_min (
   Rox_QuadDetector detector, 
   const Rox_Double segment_length_min
);

//! Set the bounds (min and max) of the size of a side of the quad
//! \param  [out]  detector       The quad Detector
//! \param  [in ]  side_min       The minimum size of a quad size in pixels
//! \param  [in ]  side_max       The maximum size of a quad size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_set_side_bounds ( 
   Rox_QuadDetector detector, Rox_Double side_min, Rox_Double side_max);

//! Set the bounds (min and max) of the size of a side of the quad
//! \param  [out]  detector       The quad Detector
//! \param  [in ]  area_min       The minimum area of a quad size in pixels
//! \param  [in ]  area_max       The maximum area of a quad size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_set_area_bounds ( 
   Rox_QuadDetector detector, Rox_Double area_min, Rox_Double area_max );

//! Get the number of detected quads
//! \param  [out]  detected_quads    Number of detected quads
//! \param  [in ]  detector          The quad detector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_get_quad_count (
   Rox_Sint * detected_quads, Rox_QuadDetector detector);

//! Get a homography matrix between a 3D square of size 1 m and the detected quad in the image
//! \param  [out]  H              The homography
//! \param  [in ]  detector       The quad detector
//! \param  [in ]  id             The quad ID
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_get_quad_SL3 ( 
   Rox_MatSL3 H, 
   Rox_QuadDetector detector, 
   Rox_Uint id
);

//! Get the coordinates of the 4 corners of the quad in the image
//! \param  [out]  pts            The points
//! \param  [in ]  detector       The quad detector
//! \param  [in ]  id             The quad ID
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quaddetector_get_points(
   Rox_Point2D_Double pts, 
   Rox_QuadDetector detector, 
   Rox_Uint id);

ROX_API Rox_ErrorCode rox_dynvec_quad_computequads ( 
   Rox_DynVec_Quad quadlist, 
   Rox_DynVec_Quad_Segment2D seglist, 
   Rox_Double side_min, 
   Rox_Double side_max, 
   Rox_Double area_min, 
   Rox_Double area_max);


ROX_API Rox_ErrorCode rox_quaddetector_save_segment2d (
   const Rox_Char * filename, 
   const Rox_QuadDetector quaddetector
);

//! @} 

#endif
