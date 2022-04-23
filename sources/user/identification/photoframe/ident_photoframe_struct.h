//==============================================================================
//
//    OPENROX   : File ident_photoframe_struct.h
//
//    Contents  : Structure of ident_photoframe module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_PHOTOFRAME_STRUCT__
#define __OPENROX_IDENT_PHOTOFRAME_STRUCT__

#include <generated/objset_photoframe.h>

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/imask/imask.h>

#include <core/features/detectors/quad/quad_detection.h>

//! \addtogroup Identification_Photoframe
//! @{

//! Photoframe identifier structure
struct Rox_Ident_PhotoFrame_Struct
{
   //! Set of photoframes to be identified
   Rox_ObjSet_PhotoFrame photoframes;

   //! Quad detector object
   Rox_QuadDetector detector;

   //! The image mask
   Rox_Imask mask;
   
   //! Homography transforming the a normalize 1 x 1 square to the current image
   Rox_MatSL3 qhom;

   //! Score threshold
   Rox_Double score_threshold;
   
   //! Do we use simple orientation detection or compute one zncc per orientation ?
   Rox_Uint orientation_method;
   
   //! Detection metod for the quad
   Rox_Uint detection_method;
   
   //! Color of the quad
   Rox_Uint quad_color;

   //! Minimum length of a segment extracted in an image for quad detection
   Rox_Double quad_segment_length_min;
};

//! @}

#endif
