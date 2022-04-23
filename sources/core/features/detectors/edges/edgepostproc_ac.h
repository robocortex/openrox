//==============================================================================
//
//    OPENROX   : File edgepostproc_ac.h
//
//    Contents  : API of edgepostproc_ac module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGEPOSTPROC_AC__
#define __OPENROX_EDGEPOSTPROC_AC__

#include <system/memory/datatypes.h>

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <generated/array2d_point2d_sshort.h>
#include <generated/dynvec_segment_part.h>
#include <generated/dynvec_segment_part_struct.h>

#include <core/features/detectors/edges/edgedraw.h>

#define MAX_GRADIENT 1500

//! \addtogroup Edges
//! \addtogroup Edges_Postproc
//! @{

//! The Rox_EdgePostproc_Ac_Struct object 
struct Rox_EdgePostproc_Ac_Struct
{
   //! The source width 
   Rox_Sint width;

   //! The source height 
   Rox_Sint height;

   //! Validation stack 
   Rox_DynVec_Segment_Part validation_stack;

   //! Result set of segments
   Rox_ObjSet_DynVec_Edgel resultsegments;
};

//! Define the pointer of the Rox_EdgePostproc_Ac_Struct 
typedef struct Rox_EdgePostproc_Ac_Struct * Rox_EdgePostproc_Ac;

//! Create edge preprocessor object
//! \param [out] obj the pointer to the object
//! \param [in] width the width
//! \param [in] height the height
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_edgepostproc_ac_new(Rox_EdgePostproc_Ac * obj, Rox_Sint width, Rox_Sint height);

//! Delete edge preprocessor  object
//! \param [in] obj the pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_edgepostproc_ac_del(Rox_EdgePostproc_Ac * obj);

//! Process edge preprocessor on image
//! \param [in] obj the pointer to the object
//! \param [in] segments the drawer to postprocess
//! \param [in] min_NFA NFA minimum value to add edge	
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_edgepostproc_ac_process(Rox_EdgePostproc_Ac obj, Rox_ObjSet_DynVec_Edgel segments, Rox_Double min_NFA);

//! @} 

#endif

