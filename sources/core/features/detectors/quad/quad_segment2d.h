//==============================================================================
//
//    OPENROX   : File quad_segment2d.h
//
//    Contents  : API of quad_segment2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUAD_SEGMENT2D__
#define __OPENROX_QUAD_SEGMENT2D__

#include <generated/dynvec_quad_segment2d.h>
#include <generated/dynvec_orientedimagepoint.h>

#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/image/image_rgba.h>

typedef struct Rox_Quad_Segment2D_Struct * Rox_Quad_Segment2D;

//! To be commented
//! \param  [out]  ptr pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_quad_segment2d_computechildren (
   Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d
);

//! To be commented
//! \param  [out]  coordx    x coordinates of the intersection point
//! \param  [out]  coordy    y coordinates of the intersection point
//! \param  [in ]  segment1  The segment 1
//! \param  [in ]  segment2  The segment 2
//! \return True if two segments intersect each other
//! \todo   To be tested
ROX_API Rox_Bool rox_dynvec_quad_segment2d_intersect (
   Rox_Double * coordx, Rox_Double * coordy, Rox_Quad_Segment2D segment1, Rox_Quad_Segment2D segment2);

//! Find a quad
//! \param  [out]  dynvec_quad_segment2d  The dynvec of quad segements
//! \param  [in ]  groups                 Pointer to the groups of pixels 
//! \param  [in ]  nbgroups               Number of groups
//! \param  [in ]  black_to_white         Set to 0 to detect white to black quads, set to 1 to detect black to white quads
//! \param  [in ]  segment_length_min     The minimum lenght of a segment (segments with length less than segment_length_min will be ignored)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_quad_segment2d_make (
   Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d, 
   Rox_DynVec_OrientedImagePoint * groups, 
   const Rox_Uint nbgroups, 
   const Rox_Uint black_to_white,
   const Rox_Double segment_length_min 
); //, Rox_Double segment_length_max );

ROX_API Rox_ErrorCode rox_dynvec_quad_segment2d_draw ( 
   Rox_Image_RGBA image_rgba, 
   const Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d, 
   const Rox_Uint color
);

ROX_API Rox_ErrorCode rox_dynvec_quad_segment2d_draw_childrens ( 
   Rox_Image_RGBA image_rgba, 
   const Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d, 
   const Rox_Uint color, 
   const Rox_Uint id
);

ROX_API Rox_ErrorCode rox_dynvec_quad_segment2d_save (
   const Rox_Char * filename, 
   const Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d
);

ROX_API Rox_ErrorCode rox_dynvec_quad_segment2d_increase (
   Rox_Quad_Segment2D created, 
   const Rox_Quad_Segment2D one, 
   const Rox_Quad_Segment2D two
);

#endif
