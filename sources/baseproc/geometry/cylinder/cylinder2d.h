//==============================================================================
//
//    OPENROX   : File cylinder2d.h
//
//    Contents  : Structures of cylinder2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CYLINDER2D__
#define __OPENROX_CYLINDER2D__

#include <generated/dynvec_point2d_double.h>

//! \ingroup Geometry
//! \addtogroup Cylinder2D
//! @{

//! Pointer to the 2D cylinder structure
typedef struct Rox_Cylinder2D_Double_Struct * Rox_Cylinder2D;

//! \param [out]   cylinder2d
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder2d_new(Rox_Cylinder2D * cylinder2d);

//! \param [out]   cylinder2d
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder2d_del(Rox_Cylinder2D * cylinder2d);
 
//! \param [in]   cylinder2d
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder2d_print(Rox_Cylinder2D cylinder2d);

//! \param [out]  dynvec_point2d
//! \param [in]   cylinder2d
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder2d_sample(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Cylinder2D cylinder2d, Rox_Double sampling_step);

//! \param [out]  angle
//! \param [in]   cylinder2d
//! \param [in]   xe
//! \param [in]   ye
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder2d_get_normal_angle(Rox_Double * angle, Rox_Cylinder2D cylinder2d, const Rox_Double xe, const Rox_Double ye);

//! \param [out]  angle
//! \param [in]   cylinder2d
//! \param [in]   xe
//! \param [in]   ye
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder2d_get_tangent_angle(Rox_Double * angle, Rox_Cylinder2D cylinder2d, const Rox_Double xe, const Rox_Double ye);

//! @}

#endif
