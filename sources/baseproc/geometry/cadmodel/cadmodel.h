//==============================================================================
//
//    OPENROX   : File cadmodel.h
//
//    Contents  : API of cadmodel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CADMODEL__
#define __OPENROX_CADMODEL__

#include <generated/config.h> 

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>

#include <system/errors/errors.h>
#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup CadModel
//! @{

//! CAD Model  structure
typedef struct Rox_CadModel_Struct * Rox_CadModel;

//! Create a new cad model constraint object
//! \param  [out]  cadmodel       The pointer to the newly created object
//! \return An error code
ROX_API Rox_ErrorCode rox_cadmodel_new ( Rox_CadModel * cadmodel );

//! Delete a cad model constraint
//! \param  [out]  cadmodel       The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_cadmodel_del ( Rox_CadModel * cadmodel );

//! Compute znear and zfar given a bounding box and minimal distance zmin
//! \param  [out]  znear          The computed znear
//! \param  [out]  zfar           The computed zfar
//! \param  [in ]  zmin           The minimal sdistance
//! \param  [in ]  cTo            The pose
//! \param  [in ]  box_points     The 8 corners of the bounding box
//! \return An error code
ROX_API Rox_ErrorCode rox_compute_znear_zfar ( 
   Rox_Float * znear, 
   Rox_Float * zfar,
   const Rox_Float zmin,
   const Rox_MatSE3 cTo, 
   const Rox_Point3D_Float box_points
);

//! @} 

#endif
