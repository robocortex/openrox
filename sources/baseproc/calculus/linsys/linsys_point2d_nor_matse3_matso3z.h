//==============================================================================
//
//    OPENROX   : File linsys_point2d_nor_matse3_matso3z.h
//
//    Contents  : API of se3_so3z_intmat
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SE3_SO3Z_INTMAT__
#define __OPENROX_SE3_SO3Z_INTMAT__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/geometry/point/point3d.h>

//! \ingroup Jacobians
//! \defgroup se3jacobians se3jacobians
//! \brief Jacobians relative to the SE3 group.
   
//! \addtogroup se3jacobians
//! @{

//! Compute the interaction matrix for two sets of points
//! related by a transformation with an unknown in XY-plane rotation
//! prepare its pseudo-inversion
//! \param  [out]  LtL            The result L^t*L Matrix
//! \param  [out]  Lte            The result projected vector (L^t*diff)
//! \param  [in ]  diffr          The error vector for the right points
//! \param  [in ]  weightr        The weight vector for the right points (for robust estimation)
//! \param  [in ]  mrc            The right points in meters which were used to compute the error
//! \param  [in ]  diffl          The error vector for the left points
//! \param  [in ]  weightl        The weight vector for the left points (for robust estimation)
//! \param  [in ]  mlc            The left points in meters which were used to compute the error, !the partial transformation must have been applied to these!
//! \param  [in ]  mltmp          The artefact vector needed to estimate the XY-place rotation
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_intmat_se3_so3z_weighted_premul_float (
	Rox_Matrix         LtL,
	Rox_Matrix         Lte,
	const Rox_Array2D_Double         diffr,
	const Rox_Array2D_Double         weightr,
	const Rox_DynVec_Point3D_Float   mrc,
	const Rox_Array2D_Double         diffl,
	const Rox_Array2D_Double         weightl,
	const Rox_DynVec_Point3D_Float   mlc,
	const Rox_DynVec_Point3D_Float   mltmp 
);

//! @} */

#endif
