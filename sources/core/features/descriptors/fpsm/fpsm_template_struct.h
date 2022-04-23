//==============================================================================
//
//    OPENROX   : File fpsm_template_struct.h
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

#ifndef __OPENROX_FPSM_TEMPLATE_STRUCT__
#define __OPENROX_FPSM_TEMPLATE_STRUCT__

#include <system/memory/datatypes.h>

//! \addtogroup FPSM
//! @{

//! Struct
struct Rox_Fpsm_Template_Struct
{
   //! To be commented
   Rox_Sint object_id;
   //! To be commented
   Rox_Sint view_id;
   //! To be commented
   Rox_Double angle;
   //! To be commented
   Rox_Double dist;
};

//! FPSM template structure
typedef struct Rox_Fpsm_Template_Struct Rox_Fpsm_Template_Struct;

//! define
#define ROX_TYPE_FPSM_TEMPLATE (sizeof(struct Rox_Fpsm_Template_Struct) << 2)

//! @}

#endif // __OPENROX_FPSM_TEMPLATE_STRUCT__