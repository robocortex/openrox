//==============================================================================
//
//    OPENROX   : File ident_texture_se3_stuct.h
//
//    Contents  : API of ident_texture_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_SE3_STRUCT__
#define __OPENROX_IDENT_SE3_STRUCT__

#include <core/identification/templateident_se3.h>

//! \addtogroup Ident_Texture_SE3
//! @{

//! SE3 identifier structure
struct Rox_Ident_Texture_SE3_Struct
{
   //! The SE3 internal identifier
   Rox_Template_Ident_SE3 template_ident_se3;
};

//! @}

#endif
