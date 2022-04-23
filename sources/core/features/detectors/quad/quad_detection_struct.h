//==============================================================================
//
//    OPENROX   : File quad_detection_struct.h
//
//    Contents  : Structure of quad_detection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUAD_DETECTION_STRUCT__
#define __OPENROX_QUAD_DETECTION_STRUCT__

#include <system/memory/datatypes.h>
#include <generated/dynvec_quad_segment2d_struct.h>
#include <generated/dynvec_quad_struct.h>
#include "quad_gradientclusterer.h"
#include "quad_segment2d_struct.h"

//! \ingroup Vision
//! \addtogroup Quad
//! @{

//! To be commented
struct Rox_QuadDetector_Struct
{
   //! To be commented
   Rox_DynVec_Quad_Segment2D segments;
   
   //! To be commented
   Rox_DynVec_Quad quads;
   
   //! To be commented
   Rox_GradientClusterer clusterer;
   
   //! Flag to set the color of the quad
   Rox_Uint black_to_white;

   //! Length min of a quad segment 2D (in pixels)
   Rox_Double segment_length_min;

   //! Size min of a side of the quad (in pixels)
   Rox_Double side_min;

   //! Size max of a side of the quad (in pixels)
   Rox_Double side_max;

   //! Size min area of the quad (in pixels²)
   Rox_Double area_min;

   //! Size max area of the quad (in pixels²)
   Rox_Double area_max;
};

//! @}

#endif
