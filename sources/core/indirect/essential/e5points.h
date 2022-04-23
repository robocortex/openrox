//==============================================================================
//
//    OPENROX   : File e5points.h
//
//    Contents  : API of e5points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_E5P__
#define __OPENROX_E5P__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup  Maths
//! \defgroup Geometry Geometry
//! \brief Structures and functions for geometry.

//! \ingroup Geometry
//! \addtogroup p5p
//! @{

//! This code is an implementation of Nister Essential matrix estimation
//! from 5 calibrated points correspondance. See this paper :
//! An Efficient Solution to the Five-Point Relative Pose Problem
//! \param  [out]  Ematrices      The output 10 possible essential matrices
//! \param  [out]  nbsolutions    How many solutions are found
//! \param  [in ]  ref2D          The 5 2D reference points (right)
//! \param  [in ]  cur2D          The 5 2D current points (left)
//! \return An error code 
//! \warning This algorithm is prone to failure on degeneration
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_essential_from_5_points_nister (
   Rox_Matrix Ematrices[10], 
   Rox_Uint * nbsolutions, 
   const Rox_Point2D_Double  ref2D, 
   const Rox_Point2D_Double  cur2D
);

//! @} 

#endif
