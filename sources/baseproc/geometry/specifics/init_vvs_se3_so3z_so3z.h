//============================================================================
//
//    OPENROX   : File init_vvs_se3_so3z_so3z.h
//
//    Contents  : API to estimate some specific transformations between four
//                frames. Initially used for the APRA project.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_INIT_VVS_SE3_SO3Z_SO3Z__
#define __OPENROX_INIT_VVS_SE3_SO3Z_SO3Z__

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>
#include <generated/array2d_double.h>

//! Estimate the optimal rotation R around Z to minimize ////vo-R*vi////
//! \param  [out] R
//! \param  [in]  vo
//! \param  [in]  vi
//! \return An error code
ROX_API Rox_ErrorCode rox_matso3_optimal_rz_from_vectors
(
   Rox_Array2D_Double R,
   Rox_Array2D_Double vo,
   Rox_Array2D_Double vi 
);

//! Estimate the closest rotation Rz to R minimizing the Frobenius norm of I-Rz'*R
//! ////M////_F = sqrt(trace(M')*M))
//! Setting M = (I-Rz'*R) -> M'*M = (I-Rz'*R)'*(I-Rz'*R) = 2(I-R'*Rz-Rz'*R)
//! \param  [out] Rz
//! \param  [in]  R
//! \return An error code
ROX_API Rox_ErrorCode rox_matso3_optimal_rz 
(
   Rox_Array2D_Double Rz,
   Rox_Array2D_Double R 
);

// Illustration:
//
//         P
//                   z            z     y
//                 /                \ |
//                /__ x            __\|
//                |                x     G
//                |
//                y
//                                    
//                                       
//      y    z                           \z
//       | /                              \___ x
//    ___|/                               |
//  x                                     |      B
//           S                            y
//                      z
//                       /
//                      /____ x
//                      |
//                      |
//                    y       C
//
//
//  Assumes:
//  . we have 4 frames: p(etit), g(rand), s(mall), b(ig)
//  . the transformations sTp and bTg are only made of:
//    . a translation along Z (p/g frame)
//    . a rotation in the XY plane (p/g frame)
//  . we know:
//    . the translation ptg between p and g
//    . the expression of G's Z axis in P frame
//    . cTs
//    . cTb
//    . the translations between:
//      . p and s
//      . g and b
//  . p frame may be considered as the principal frame
//
//  Finds:
//  . estimation of transformation matrices between all frames
//
//  Note:
//  . Rotation of G's X and Y axes can freely move around Z
//    there there are several solutions to gTp and bTg
//

//! Initialize intermediary transformations to achieve vvs_se3_so3z_so3z
//! Frames' (for p,g,s,b) z axis are supposed to be always "in the same directions" to the
//! one of the camera frame
//! \param  [out]  bTg              transformation between g and b
//! \param  [out]  gTp              transformation between p and g
//! \param  [out]  pTs              transformation between s and p
//! \param  [in ]  cTb              transformation from b frame to camera frame
//! \param  [in ]  cTs              transformation from s frame to camera frame
//! \param  [in ]  ptg_mod          translation between p frame and g frame
//! \param  [in ]  zg_p             Z axis of g frame expressed in p frame
//! \param  [in ]  dzs_p            scalar absolute distance, along z, between s and p frames
//! \param  [in ]  dzb_g            scalar absolute distance, along z, between b and g frames
//! \return An error code
ROX_API Rox_ErrorCode rox_init_se3_so3z_so3z_free_G 
(
   Rox_Array2D_Double bTg,
   Rox_Array2D_Double gTp,
   Rox_Array2D_Double pTs,
   Rox_Array2D_Double cTb,
   Rox_Array2D_Double cTs,
   Rox_Array2D_Double ptg_mod,
   Rox_Array2D_Double zg_p,
   Rox_Double         dzs_p,
   Rox_Double         dzb_g,
   Rox_Sint           force_model 
);

// Illustration: extern face
//
//         P                             G
//                   z            
//                 /                 \z
//                /__ x               \__ x
//                |                   |
//                |                   |
//                y                   y
//                                    
//                                       
//      y    z                           \z
//       | /                              \___ x
//    ___|/                               |
//  x                                     |       B
//           S                            y
//                      z
//                       /
//                      /____ x
//                      |
//                      |
//                    y       C
//
//
//  Assumes:
//  . we have 4 frames: p(etit), g(rand), s(mall), b(ig)
//  . the transformations sTp and bTg are only made of:
//    . a translation along Z (p/g frame)
//    . a rotation in the XY plane (p/g frame)
//  . we know:
//    . the transformation pTg between p and g
//    . cTs
//    . cTb
//    . the translations between:
//      . p and s
//      . g and b
//  . p frame may be considered as the principal frame
//
//  Note:
//  . ptg(y) = gtp(y) = 0
//  . ptg(x) > 0 and gtp(x) < 0 for outside/extern face
//  . ptg(x) < 0 and gtp(x) > 0 for inside/intern face
//  This is neither checked nor enforced by the present procedure
//  since pTg is an input variable
//

//! Initialize intermediary transformations to achieve vvs_se3_so3z_so3z
//! Frames' (for p,g,s,b) z axis are supposed to be always "in the same directions" to the
//! one of the camera frame
//! \param  [out]  bTg              transformation between g and b
//! \param  [out]  pTs              transformation between s and p
//! \param  [in ]  cTb              transformation from b frame to camera frame
//! \param  [in ]  cTs              transformation from s frame to camera frame
//! \param  [in ]  pTg              transformation between p frame and g frame
//! \param  [in ]  dzs_p            scalar absolute distance, along z, between s and p frames
//! \param  [in ]  dzb_g            scalar absolute distance, along z, between b and g frames
//! \return An error code
ROX_API Rox_ErrorCode rox_init_se3_so3z_so3z_fixed_G
(
   Rox_Array2D_Double bTg,
   Rox_Array2D_Double pTs,
   Rox_Array2D_Double cTb,
   Rox_Array2D_Double cTs,
   Rox_Array2D_Double pTg,
   Rox_Double         dzs_p,
   Rox_Double         dzb_g,
   Rox_Sint           force_model 
);

// ptb actually is pTg
ROX_API Rox_ErrorCode rox_init_se3_model_adjustment
(
   Rox_Array2D_Double T, 
   Rox_Array2D_Double pTb_mes, 
   Rox_Array2D_Double pTb_mod
);

//! Compute the rotation matrix supposing vector z_axis as the z-axis,
//! y is the axis perpendicular to the v-z plane and x follows from right-hand rule
//! \param  [out] R
//! \param  [in]  v
//! \param  [in]  z
//! \return An error code
ROX_API Rox_ErrorCode rox_matso3_from_2_vectors
(
   Rox_Array2D_Double Rz, 
   Rox_Array2D_Double v, 
   Rox_Array2D_Double z
);

//! @}

#endif // __OPENROX_INIT_VVS_SE3_SO3Z_SO3Z__
