//==============================================================================
//
//    OPENROX   : File plane_from_3_point3d->c
//
//    Contents  : Implementation of plane_3points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "plane_3points.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_plane3d_from_3_point3d ( 
   Rox_Plane3D_Double plane, 
   const Rox_Point3D_Double pt1, 
   const Rox_Point3D_Double pt2, 
   const Rox_Point3D_Double pt3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Compute normal such that z goes away from the camera
   Rox_Double dx1 = pt2->X - pt1->X;
   Rox_Double dx2 = pt3->X - pt1->X;
   Rox_Double dy1 = pt2->Y - pt1->Y;
   Rox_Double dy2 = pt3->Y - pt1->Y;
   Rox_Double dz1 = pt2->Z - pt1->Z;
   Rox_Double dz2 = pt3->Z - pt1->Z;
   
   Rox_Double na = dy1 * dz2 - dz1 * dy2;
   Rox_Double nb = dz1 * dx2 - dx1 * dz2;
   Rox_Double nc = dx1 * dy2 - dy1 * dx2;
   Rox_Double nd = -(na * pt1->X + nb * pt1->Y + nc * pt1->Z);
   
   Rox_Double norm = sqrt(na*na+nb*nb+nc*nc);

   if (fabs(norm) < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   plane->a = na / norm;
   plane->b = nb / norm;
   plane->c = nc / norm;
   plane->d = nd / norm;

function_terminate:
   return error; 
}

Rox_ErrorCode rox_plane3d_from_3_point3d_float ( 
   Rox_Plane3D_Double plane, 
   const Rox_Point3D_Float pt1, 
   const Rox_Point3D_Float pt2, 
   const Rox_Point3D_Float pt3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Compute normal such that z goes away from the camera
   Rox_Double dx1 = pt2->X - pt1->X;
   Rox_Double dx2 = pt3->X - pt1->X;
   Rox_Double dy1 = pt2->Y - pt1->Y;
   Rox_Double dy2 = pt3->Y - pt1->Y;
   Rox_Double dz1 = pt2->Z - pt1->Z;
   Rox_Double dz2 = pt3->Z - pt1->Z;
   
   Rox_Double na = dy1 * dz2 - dz1 * dy2;
   Rox_Double nb = dz1 * dx2 - dx1 * dz2;
   Rox_Double nc = dx1 * dy2 - dy1 * dx2;
   Rox_Double nd = -(na * pt1->X + nb * pt1->Y + nc * pt1->Z);
   
   Rox_Double norm = sqrt(na*na+nb*nb+nc*nc);

   if (fabs(norm) < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   plane->a = na / norm;
   plane->b = nb / norm;
   plane->c = nc / norm;
   plane->d = nd / norm;

function_terminate:
   return error; 
}
