//==============================================================================
//
//    OPENROX   : File quad_struct.h
//
//    Contents  : Structure of quad module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUAD_STRUCT__
#define __OPENROX_QUAD_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Vision
//! \addtogroup Quad
//! @{

//! The structure of a quad
struct Rox_Quad_Struct
{
   //! Coordiantes u of the 4 points of the quad
   Rox_Double u[4];

   //! Coordiantes v of the 4 points of the quad
   Rox_Double v[4];
};

//! To be commented
typedef struct Rox_Quad_Struct Rox_Quad_Struct;

//! To be commented
typedef struct Rox_Quad_Struct * Rox_Quad;

//! @}

#endif
