//==============================================================================
//
//    OPENROX   : File p16lines.h
//
//    Contents  : API of p16lines module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_P16LINES__
#define __OPENROX_P16LINES__

#include <generated/array2d_double.h>
#include <generated/dynvec_line3d_plucker.h>

//! Compute a pose between two set of cameras (the reference and and the current one)
//! \param  []  pose       the result computed poses (2 max)
//! \param  []  reflines   the reference reflines which are transformed by the pose
//! \param  []  curlines   the current curlines
//! \return An error code
ROX_API Rox_ErrorCode rox_pose_from_16_lines(Rox_Array2D_Double_Collection poses, Rox_Uint * validposes, Rox_DynVec_Line3D_Plucker reflines, Rox_DynVec_Line3D_Plucker curlines);

#endif
