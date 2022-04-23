//==============================================================================
//
//    OPENROX   : File cylinder3d.h
//
//    Contents  : Structures of cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CYLINDER3D__
#define __OPENROX_CYLINDER3D__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/segment/segment3d.h>

//! \ingroup Geometry
//! \addtogroup Cylinder3D
//! @{

//! Pointer to 3D cylinder structure
typedef struct Rox_Cylinder3D_Double_Struct * Rox_Cylinder3D;

//! Create a Rox_Cylinder3D object and initialize it to a = 1.0, b = 1.0, Te = I
//! \param [out]  cylinder3d        The object to create
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_new(Rox_Cylinder3D * cylinder3d);

//! Delete a Rox_Cylinder3D object
//! \param [out]  cylinder3d        The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_del(Rox_Cylinder3D * cylinder3d);

//! Set the parameters of a Rox_Cylinder3D object
//! \param [out]  cylinder3d        The object to delete
//! \param [in]   a
//! \param [in]   b
//! \param [in]   h
//! \param [in]   T
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_set(Rox_Cylinder3D cylinder3d, Rox_Double a, Rox_Double b, Rox_Double h, Rox_MatSE3 T);

//! Copy a Rox_Cylinder3D object
//! \param [out]  cylinder3d_out    The object to copy into
//! \param [out]  cylinder3d_inp    The object to be copied
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_copy(Rox_Cylinder3D cylinder3d_out, Rox_Cylinder3D cylinder3d_inp);

//! Display the parameters of a a Rox_Cylinder3D object
//! \param [out]  cylinder3d        The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_print(Rox_Cylinder3D cylinder3d);

//! Compute the segments obtained from the intersection of planes tangent to the cilinder 
//! and passing through the origin of frame F_cam. The segments are in F_cam coordinates.
//! \param [out]  segment3d _1      The segment 1
//! \param [out]  segment3d _2      The segment 2
//! \param [in]   cam_T_obj         The pose
//! \param [in]   cylinder3dc       The cylinder
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_get_tangent_segments(Rox_Segment3D segment3d_1, Rox_Segment3D segment3d_2, Rox_MatSE3 cam_T_obj, Rox_Cylinder3D cylinder3d);

//! Set the parameters of a Rox_Cylinder3D object
//! \param [out]  cylinder3d        The object to delete
//! \param [in]   a
//! \param [in]   b
//! \param [in]   h
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_set_params(Rox_Cylinder3D cylinder3d, Rox_Double a, Rox_Double b, Rox_Double h);

//! Set the pose of a Rox_Cylinder3D object
//! \param [out]  cylinder3d        The object to delete
//! \param [in]   T
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_cylinder3d_set_pose(Rox_Cylinder3D cylinder3d, Rox_MatSE3 T);

//! @}

#endif
