//============================================================================
//
//    OPENROX   : File model_single_plane_struct.h
//
//    Contents  : API of model_single_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_MODEL_SINGLE_PLANE_STRUCT__
#define __OPENROX_MODEL_SINGLE_PLANE_STRUCT__

#include <generated/array2d_float.h>

#include <baseproc/image/imask/imask.h>
#include <baseproc/maths/linalg/matut3.h>

#include <baseproc/geometry/plane/plane_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

//! \addtogroup Model_Single_Plane
//! @{

//! One non parralel planar patch model description 
struct Rox_Model_Single_Plane_Struct
{
   //! Image of the patch 
   Rox_Image image_template;

   //! Validity of the patch 
   Rox_Imask mask;

   //! From template meters to pixel calibration matrix 
   Rox_MatUT3 calibration_template;

   //! From template pixel to meters calibration matrix 
   Rox_MatUT3 calibration_template_inverse;

   //! Pose of the template plane frame to object reference frame 
   Rox_MatSE3 z0_T_o;

   //! Pose of the template plane frame to object reference frame shifted
   Rox_MatSE3 z1_T_o;

   //! Reference frame to Template plane 
   Rox_MatSE3 o_T_z0;

   //! Reference frame to Template plane 
   Rox_MatSE3 o_T_z1;

   //! Current set pose wrt Reference frame
   Rox_MatSE3 c_T_o;

   //! Current set pose wrt plane
   Rox_MatSE3 c_T_z0;

   //! Current set pose wrt plane
   Rox_MatSE3 c_T_z1;

   //! Reference vertices in frame Fo
   // The 3D coordinates of the 4 vertices of the template image
   // vertices_ref[0] -> top left corner
   // vertices_ref[1] -> top right corner
   // vertices_ref[2] -> bottom right corner
   // vertices_ref[3] -> bottom left corner
   Rox_Point3D_Double_Struct vertices_ref[4];

   //! Current vertices in frame Fc
   // The 3D coordinates of the 4 vertices of the template image
   // vertices_cur[0] -> top left corner
   // vertices_cur[1] -> top right corner
   // vertices_cur[2] -> bottom right corner
   // vertices_cur[3] -> bottom left corner
   Rox_Point3D_Double_Struct vertices_cur[4];

   //! Reference plane in frame Fo
   Rox_Plane3D_Double_Struct plane_ref;

   //! Current plane in frame Fc
   Rox_Plane3D_Double_Struct plane_cur;

   //! Is this patch visible from the camera POV (Point Of View) 
   Rox_Sint is_potentially_visible;

   // Size of the rectangle
   Rox_Double sizex;
   Rox_Double sizey;
};

//! @} 

#endif //__OPENROX_MODEL_SINGLE_PLANE_STRUCT__
