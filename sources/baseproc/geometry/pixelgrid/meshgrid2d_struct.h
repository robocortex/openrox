//==============================================================================
//
//    OPENROX   : File meshgrid2d_struct.h
//
//    Contents  : Stucture of meshgrid2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MESHGRID2D_STRUCT__
#define __OPENROX_MESHGRID2D_STRUCT__

#include <generated/array2d_float.h>
#include <generated/array2d_sshort.h>

//! \addtogroup MeshGrid2D
//! @{

//! The Rox_MeshGrid2D_Float_Struct object
struct Rox_MeshGrid2D_Float_Struct
{
   //! The u coordinates
   Rox_Array2D_Float  u;

   //! The v coordinates
   Rox_Array2D_Float  v;
};

//! The Rox_MeshGrid2D_Float_Struct object
struct Rox_MeshGrid2D_Sshort_Struct
{
   //! The u coordinates
   Rox_Array2D_Sshort  u;

   //! The v coordinates
   Rox_Array2D_Sshort  v;
};
//! @}

#endif // __OPENROX_MESHGRID2D_STRUCT__
