//==============================================================================
//
//    OPENROX   : File edgepostproc_segment.h
//
//    Contents  : API of edgepostproc_segment module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGEPOSTPROC_SEGMENT__
#define __OPENROX_EDGEPOSTPROC_SEGMENT__

#include <system/memory/datatypes.h>

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <generated/array2d_point2d_sshort.h>
#include <generated/dynvec_segment2d.h>

#include <core/features/detectors/edges/edgedraw.h>

//! \ingroup Edges
//! \addtogroup Edges_Postproc
//! @{

//! The Rox_EdgePostproc_Segment_Struct object 
struct Rox_EdgePostproc_Segment_Struct
{
   //! The source width 
   Rox_Sint width;

   //! The source height 
   Rox_Sint height;

   //! The minimal length for a line
   Rox_Uint minimal_length;

   //! Result set of segments
   Rox_DynVec_Segment2D resultsegments;
};

//! Define the pointer of the Rox_EdgePostproc_Segment_Struct 
typedef struct Rox_EdgePostproc_Segment_Struct * Rox_EdgePostproc_Segment;

//! Create edge preprocessor object
//! \param [out] obj the pointer to the object
//! \param [in] width the width
//! \param [in] height the height
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_edgepostproc_segment_new(Rox_EdgePostproc_Segment * obj, Rox_Sint width, Rox_Sint height);

//! Delete edge preprocessor  object
//! \param [in] obj the pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_edgepostproc_segment_del(Rox_EdgePostproc_Segment * obj);

//! Process edge preprocessir on image
//! \param [in] obj the pointer to the object
//! \param [in] segments the drawer to postprocess
//! \return An error code
//! \todo to be tested 
ROX_API Rox_ErrorCode rox_edgepostproc_segment_process(Rox_EdgePostproc_Segment obj, Rox_ObjSet_DynVec_Edgel segments);

//! Process edge preprocessir on image
//! \param [in] obj the pointer to the object
//! \param [in] segments the drawer to postprocess
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_edgepostproc_segment_process_robust(Rox_EdgePostproc_Segment obj, Rox_ObjSet_DynVec_Edgel segments);

//! @} 

#endif // __OPENROX_EDGEPOSTPROC_SEGMENT__


