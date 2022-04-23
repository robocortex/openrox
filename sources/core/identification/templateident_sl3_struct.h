//==============================================================================
//
//    OPENROX   : File templateident_sl3_struct.h
//
//    Contents  : Structure of templateident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TEMPLATE_IDENT_SL3_STRUCT__
#define __OPENROX_TEMPLATE_IDENT_SL3_STRUCT__

#include <core/identification/multiident.h>
#include "multiident_struct.h"
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/objset_dynvec_sraid_feature.h>

//! \addtogroup Identification
//! @{

//! SL3 identifier structure
struct Rox_Template_Ident_SL3_Struct
{
   //! SRAID matcher
   Rox_Multi_Ident matcher;

   //! Ransac filtered matches
   Rox_DynVec_Point2D_Float current_points_ransac;

   //! Ransac filtered matches
   Rox_DynVec_Point2D_Float reference_points_ransac;
};

//! @}

#endif
