//==============================================================================
//
//    OPENROX   : File segment2d.h
//
//    Contents  : Structure of segment2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENT2D__
#define __OPENROX_SEGMENT2D__

#include <system/memory/datatypes.h>
#include <generated/dynvec_point2d_double.h>
#include <baseproc/geometry/line/line2d.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Segment Segment

//! \addtogroup Segment
//! @{

//! Segment 2D double pointer to structure
typedef struct Rox_Segment2D_Struct * Rox_Segment2D;

//! Segment 2D signed int pointer to structure
typedef struct Rox_Segment2D_Sint_Struct * Rox_Segment2D_Sint;

//! To be commented
//! \param  [out]  segment2d 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment2d_new ( 
   Rox_Segment2D * segment2d
);

//! To be commented
//! \param  [out]  segment2d 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment2d_del ( 
   Rox_Segment2D * segment2d
);

//! To be commented
//! \param  [out]  segment2d 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment2d_set ( 
   Rox_Segment2D segment2d, 
   const Rox_Double u1, 
   const Rox_Double v1, 
   const Rox_Double u2, 
   const Rox_Double v2
);

//! To be commented
//! \param  [in ]  segment2d 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment2d_print (
   const Rox_Segment2D segment2d
);

//! To be commented
//! \param  [out]  dynvec_point2d 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment2d_sample (
   Rox_DynVec_Point2D_Double dynvec_point2d, 
   const Rox_Segment2D segment2d, 
   const Rox_Double sampling_step
);

//! To be commented
//! \param  [out]  line2d 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_segment2d_get_line2d_normal (
   Rox_Line2D_Normal line2d, 
   const Rox_Segment2D segment2d
);

//! @}

#endif
