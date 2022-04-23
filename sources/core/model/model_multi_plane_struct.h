//==============================================================================
//
//    OPENROX   : File model_multi_plane_struct.h
//
//    Contents  : API of model_multi_plane_struct module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MODEL_MULTI_PLANE_STRUCT__
#define __OPENROX_MODEL_MULTI_PLANE_STRUCT__

#include <generated/objset_model_single_plane.h>
#include <generated/objset_matse3.h>

//! \addtogroup Model_Multi_Plane
//! @{

//! Structure of model multi plane
struct Rox_Model_Multi_Plane_Struct
{
   //! Planes
   Rox_ObjSet_Model_Single_Plane planes;

   Rox_ObjSet_MatSE3 pTo;
};

//! @}

#endif // __OPENROX_MODEL_MULTI_PLANE_STRUCT__
