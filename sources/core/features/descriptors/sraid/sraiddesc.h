//==============================================================================
//
//    OPENROX   : File sraiddesc.h
//
//    Contents  : API of sraiddesc module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID_DESC__
#define __OPENROX_SRAID_DESC__

#include <core/features/detectors/dog/dog.h>
#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_sraiddesc_struct.h>

//! \addtogroup SRAID
//!  @{

//! Given a list of detected features, compute associated descriptors, there may be more than one
//! \param  [out]  sraidfeatures     descriptor list
//! \param  [in ]  dogfeatures       vector of input features
//! \param  [in ]  countdogfeatures  count input features
//! \param  [in ]  scalespace        the scale space collection to use
//! \param  [in ]  object_id         a convenience value to store an associated id.
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraiddescriptor_process (
   Rox_DynVec_SRAID_Feature sraidfeatures, 
   Rox_Dog_Feature dogfeatures, 
   Rox_Uint countdogfeatures, 
   Rox_Array2D_Float_Collection scalespace, 
   Rox_Uint object_id
);

//! Given a list of detected features, compute their associated descriptor
//! \param  [out]  sraidfeatures     descriptor list
//! \param  [in ]  dogfeatures       vector of input features
//! \param  [in ]  countdogfeatures  count input features
//! \param  [in ]  scalespace        the scale space collection to use
//! \param  [in ]  object_id         a convenience value to store an associated id.
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraiddescriptor_process_fixed ( 
   Rox_DynVec_SRAID_Feature sraidfeatures, 
   Rox_Dog_Feature dogfeatures, 
   Rox_Uint countdogfeatures, 
   Rox_Array2D_Float_Collection scalespace, 
   Rox_Uint object_id
);

//! @}

#endif
