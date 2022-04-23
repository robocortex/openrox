//==============================================================================
//
//    OPENROX   : File edgedraw.h
//
//    Contents  : API of edgedraw module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGEDRAW__
#define __OPENROX_EDGEDRAW__

#include <system/memory/datatypes.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_sshort.h>
#include <generated/array2d_float.h>
#include <generated/array2d_point2d_sshort.h>
#include <generated/objset_dynvec_edgel.h>
#include <generated/dynvec_edgeturn.h>

//#define MAX_GRADIENT 1500

//! \ingroup Image_Display
//! \addtogroup EdgeDraw
//! @{

//! The Rox_EdgeDraw_Struct object
struct Rox_EdgeDraw_Struct
{
   //! The source width 
   Rox_Sint width;

   //! The source height 
   Rox_Sint height;

   //! Threshold
   Rox_Sshort anchorthresh;

   //! Minimal gradient
   Rox_Sshort min_gradient;

   //! Gradient norm
   Rox_Array2D_Sshort gnorm;

   //! Gradient direction (1 is vertical, 0 horizontal)
   Rox_Array2D_Uchar gori;

   //! List of anchor points
   Rox_DynVec_Edgel anchors;

   //! List of anchor points
   Rox_DynVec_EdgeTurn stack;

   //! Result mask
   Rox_Array2D_Uchar resultmask;

   //! Result set of segments
   Rox_ObjSet_DynVec_Edgel resultsegments;
};

//! Define the pointer of the Rox_EdgeDraw_Struct 
typedef struct Rox_EdgeDraw_Struct * Rox_EdgeDraw;

//! Create edge draw object
//! \param[out]  obj             The pointer to the object
//! \param[in]   width           The width
//! \param[in]   height          The height
//! \param[in]   min_gradient    The minimum gradient
//! \param[in]   anchorthresh    The threshold for a pixel to be considered an anchor
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_edgedraw_new(Rox_EdgeDraw * obj, Rox_Sint width, Rox_Sint height, Rox_Uint min_gradient, Rox_Uint anchorthresh);

//! Delete edge draw  object
//! \param[in]   obj            The pointer to the object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_edgedraw_del(Rox_EdgeDraw * obj);

//! Process edge drawing on image
//! \param[in]   obj                  The pointer to the object
//! \param[in]   gradient             The image gradients to process (with dimensions equals to those set in object constructor)
//! \param[in]   min_segment_size     The minimum size of a segment to take in account
//! \param[in]   straight_edge_only   Only take in account straight edge, no turns allowed
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_edgedraw_process(Rox_EdgeDraw obj, Rox_Array2D_Point2D_Sshort gradient, Rox_Uint min_segment_size, Rox_Bool straight_edge_only);

//! @} 

#endif

