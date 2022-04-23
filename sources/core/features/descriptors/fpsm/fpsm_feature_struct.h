//==============================================================================
//
//    OPENROX   : File fpsm_feature_struct.h
//
//    Contents  : API of fpsm_objects module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FPSM_FEATURE_STRUCT__
#define __OPENROX_FPSM_FEATURE_STRUCT__

#include <system/memory/datatypes.h>

//! \addtogroup FPSM
//! @{

//! Structure
struct Rox_Fpsm_Feature_Struct
{
   //! To be commented
   Rox_Sint top;
   //! To be commented
   Rox_Sint left;
   //! To be commented
   Rox_Double distances[128];
   //! To be commented
   Rox_Double angles[128];
};

//! FPSM point structure
typedef struct Rox_Fpsm_Feature_Struct Rox_Fpsm_Feature_Struct;

//! define
#define ROX_TYPE_FPSM_FEATURE (sizeof(struct Rox_Fpsm_Feature_Struct) << 2)

//! @}

#endif