//==============================================================================
//
//    OPENROX   : File moving_edge_params.h
//
//    Contents  : API of moving_edge_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MOVING_EDGE_PARAMS__
#define __OPENROX_MOVING_EDGE_PARAMS__

#include <generated/array2d_double.h>

//! \ingroup Tracking
//! \addtogroup Moving_Edge
//! @{

//! Moving edge param structure 
struct Rox_Moving_Edge_Params_Struct
{
   //! Count of masks
   Rox_Uint _count_masks;

   //! Mask size
   Rox_Uint _mask_size;

   //! Search range for edge
   Rox_Sint _search_range;

   //! Contrast threshold 
   Rox_Double _contrast_threshold;

   //! Contrast minimal value
   Rox_Double _contrast_min;

   //! Contrast maximal value
   Rox_Double _contrast_max;
   
   //! Prebuild masks 
   Rox_Array2D_Double_Collection _masks;
};

//! Rox_Moving_Edge_Params object 
typedef struct Rox_Moving_Edge_Params_Struct * Rox_Moving_Edge_Params;

//! Create Moving Edges parameters object
//! \param obj the pointer to the moving edge params object
//! \param search_range distance in number of pixels to search from each site
//! \param contrast_threshold contrast threshold to validate a convolution result
//! \return an error code
ROX_API Rox_ErrorCode rox_moving_edge_params_new(Rox_Moving_Edge_Params * obj, const Rox_Sint search_range, const Rox_Double contrast_threshold);

//! Delete Moving Edges parameters object
//! \param obj the pointer to the moving edge params object
//! \return an error code
ROX_API Rox_ErrorCode rox_moving_edge_params_del(Rox_Moving_Edge_Params * obj);

//! @} 

#endif // __OPENROX_TRACKING_moving_edge_PARAMS__
