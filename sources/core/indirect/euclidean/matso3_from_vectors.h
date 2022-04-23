//==============================================================================
//
//    OPENROX   : File matso3_from_vectors.h
//
//    Contents  : API of matso3_from_vectors module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATSO3_FROM_VECTORS__
#define __OPENROX_MATSO3_FROM_VECTORS__

#include <generated/dynvec_point3d_double.h>
#include <baseproc/maths/linalg/matso3.h>

//! \ingroup Geometry
//! \addtogroup MatSO3
//! @{

//! Find the rotation matrix R in MatSO3 which minimize the distance ||vcur - R*vref|| in the least square sense.
//! \param  [out]  R              The estimated rotation matrix
//! \param  [in ]  vref           The reference set of unit vectors (|| vref || = 1) 
//! \param  [in ]  vcur           The current set of unit vectors   (|| vcur || = 1) 
//! \return An error code
ROX_API Rox_ErrorCode rox_matso3_from_vectors ( Rox_MatSO3 R, const Rox_DynVec_Point3D_Double vref, const Rox_DynVec_Point3D_Double vcur);

//! @}

#endif
