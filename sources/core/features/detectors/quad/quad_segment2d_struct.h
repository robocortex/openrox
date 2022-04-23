//==============================================================================
//
//    OPENROX   : File quad_segment2d_struct.h
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

#ifndef __OPENROX_QUAD_SEGMENT2D_STRUCT__
#define __OPENROX_QUAD_SEGMENT2D_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Vision
//! \addtogroup Quad
//! @{

//! To be commented
struct Rox_Quad_Segment2D_Struct
{
   //! TODO: Should be better to reuse segment2D double structure ?

   //! Coordinates of the starting point
   Rox_Double start[2];
   
   //! Coordinates of the ending point
   Rox_Double end[2];

   //! Orientation of the segment
   Rox_Double theta;

   //! Length of the segment
   Rox_Double length;

   //! TODO: Should be better to reuse a dynvec_quad_segment2d ? 
   
   //! Number of children
   Rox_Uint countchildren;

   //! Childrens
   struct Rox_Quad_Segment2D_Struct * child[100];
};

typedef struct Rox_Quad_Segment2D_Struct Rox_Quad_Segment2D_Struct;

//! @}

#endif
