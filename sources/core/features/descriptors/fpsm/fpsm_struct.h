//==============================================================================
//
//    OPENROX   : File fpsm_struct.h
//
//    Contents  : Structure of fpsm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FPSM_STRUCT__
#define __OPENROX_FPSM_STRUCT__

#include <system/memory/datatypes.h>

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_sint.h>
#include <generated/array2d_point2d_sshort.h>

#include <core/features/detectors/edges/edgepreproc_gray.h>
#include <core/features/detectors/edges/edgedraw.h>
#include <core/features/detectors/edges/edgepostproc_ac.h>

//! \ingroup Descriptors
//! \defgroup FPSM FPSM
//! \brief FPSM edges descriptor.

//! \addtogroup FPSM
//! @{

//! The Rox_Fpsm_Struct object 
struct Rox_Fpsm_Struct
{
   //! To be commented  
   Rox_Sint width;
   //! To be commented  
   Rox_Sint height;
   //! To be commented  
   Rox_Uint nbr_ref_points;
   //! To be commented  
   Rox_Uint count_edgels;

   //! To be commented  
   Rox_EdgePreproc_Gray preproc;
   //! To be commented  
   Rox_EdgeDraw drawing;
   //! To be commented  
   Rox_EdgePostproc_Ac postproc;

   //! To be commented  
   Rox_Array2D_Uchar edges;
   //! To be commented  
   Rox_Array2D_Sshort angles;
   //! To be commented  
   Rox_Array2D_Sshort anglemap;
   //! To be commented  
   Rox_Array2D_Sshort distancemap;
   //! To be commented  
   Rox_Array2D_Point2D_Sshort distancemap_points;
};

//! @} 

#endif // __OPENROX_FPSM__ 

