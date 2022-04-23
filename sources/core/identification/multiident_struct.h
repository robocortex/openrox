//==============================================================================
//
//    OPENROX   : File multiident_struct.h
//
//    Contents  : Structure of multiident module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MULTI_IDENT_STRUCT__
#define __OPENROX_MULTI_IDENT_STRUCT__

#include "templateident_struct.h"

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/objset_template_ident.h>
#include <generated/objset_template_ident_struct.h>
#include <generated/dynvec_sraiddesc_struct.h>
#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_uint.h>

//! \addtogroup Identification
//! @{

//! The Rox_Multi_Ident_Struct object
struct Rox_Multi_Ident_Struct
{
   //! List of templates
   Rox_ObjSet_Template_Ident idents;

   //! Detected features
   Rox_DynVec_SRAID_Feature current_features;

   //! A buffer for matches ids
   Rox_DynVec_Sint matcheslist;
};

//! @}

#endif // __OPENROX_MULTI_IDENT_STRUCT__
