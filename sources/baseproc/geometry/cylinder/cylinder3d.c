//==============================================================================
//
//    OPENROX   : File cylinder3d.c
//
//    Contents  : Implementation Structures of cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cylinder3d.h"

#include <math.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/segment/segment_transform.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_cylinder3d_new(Rox_Cylinder3D * cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder3D ret = NULL;

   if (!cylinder3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *cylinder3d = NULL;

   ret = (Rox_Cylinder3D) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->T = NULL;

   error = rox_matse3_new(&ret->T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->a = 1.0;
   ret->b = 1.0;
   ret->h = 1.0;

   *cylinder3d = ret;
   
function_terminate:
   if(error) rox_cylinder3d_del(&ret);
   return error;
}

Rox_ErrorCode rox_cylinder3d_del(Rox_Cylinder3D * cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder3D todel = NULL;

   if (!cylinder3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *cylinder3d;
   *cylinder3d = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&todel->T);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder3d_set(Rox_Cylinder3D cylinder3d, Rox_Double a, Rox_Double b, Rox_Double h, Rox_MatSE3 T)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder3d || !T)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (a <= 0.0 || b <= 0.0 || h <= 0.0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cylinder3d->a = a;
   cylinder3d->b = b;
   cylinder3d->h = h;

   error = rox_matse3_copy(cylinder3d->T, T);   
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder3d_set_pose(Rox_Cylinder3D cylinder3d, Rox_MatSE3 T)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder3d || !T)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy(cylinder3d->T, T);   
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder3d_set_params(Rox_Cylinder3D cylinder3d, Rox_Double a, Rox_Double b, Rox_Double h)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (a <= 0.0 || b <= 0.0 || h <= 0.0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cylinder3d->a = a;
   cylinder3d->b = b;
   cylinder3d->h = h;

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder3d_copy(Rox_Cylinder3D cylinder3d_out, Rox_Cylinder3D cylinder3d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder3d_out || !cylinder3d_inp)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cylinder3d_out->a = cylinder3d_inp->a;
   cylinder3d_out->b = cylinder3d_inp->b;
   cylinder3d_out->h = cylinder3d_inp->h;

   error = rox_matse3_copy(cylinder3d_out->T, cylinder3d_inp->T);
   ROX_ERROR_CHECK_TERMINATE ( error ); 
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder3d_print(Rox_Cylinder3D cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("cylinder3d : \n");
   rox_log("a = %f \n", cylinder3d->a);
   rox_log("b = %f \n", cylinder3d->b);
   rox_log("h = %f \n", cylinder3d->h);
   rox_log("pose T : \n");
   rox_matse3_print(cylinder3d->T);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment3d_tangent_to_cylinder3d(Rox_Segment3D segment3d_1_cyl, Rox_Segment3D segment3d_2_cyl, Rox_Double a, Rox_Double b, Rox_Double h, Rox_Double O_x, Rox_Double O_y)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment3d_1_cyl || !segment3d_2_cyl) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double a2 = a*a;
   Rox_Double b2 = b*b;
   Rox_Double d2 = O_x*O_x*b2 + O_y*O_y*a2;
   Rox_Double c2 = a2*b2;

   // Compute the tangent at the canonical cylinder passing through the camera optical center
   if ((d2 < 10e-12) || (d2 - c2 < 0.0))
   { 
      error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); 
   }
   else
   {
      Rox_Double Z_cy_min = -h/2;
      Rox_Double Z_cy_max = +h/2;

      // cylinder segment 1 in the object frame
      segment3d_1_cyl->points[0].X = a2*(O_x*b2 - O_y*sqrt(d2 - c2))/d2; // X_cy
      segment3d_1_cyl->points[0].Y = b2*(O_y*a2 + O_x*sqrt(d2 - c2))/d2; // Y_cy 
      segment3d_1_cyl->points[0].Z = Z_cy_min;                           // Z_cy

      segment3d_1_cyl->points[1].X = a2*(O_x*b2 - O_y*sqrt(d2 - c2))/d2; // X_cy
      segment3d_1_cyl->points[1].Y = b2*(O_y*a2 + O_x*sqrt(d2 - c2))/d2; // Y_cy 
      segment3d_1_cyl->points[1].Z = Z_cy_max;                           // Z_cy
      
      // cylinder segment 2 in the object frame
      segment3d_2_cyl->points[0].X = a2*(O_x*b2 + O_y*sqrt(d2 - c2))/d2; // X_cy
      segment3d_2_cyl->points[0].Y = b2*(O_y*a2 - O_x*sqrt(d2 - c2))/d2; // Y_cy
      segment3d_2_cyl->points[0].Z = Z_cy_min;                           // Z_cy

      segment3d_2_cyl->points[1].X = a2*(O_x*b2 + O_y*sqrt(d2 - c2))/d2; // X_cy
      segment3d_2_cyl->points[1].Y = b2*(O_y*a2 - O_x*sqrt(d2 - c2))/d2; // Y_cy
      segment3d_2_cyl->points[1].Z = Z_cy_max;                           // Z_cy
   }
      
function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder3d_get_tangent_segments(Rox_Segment3D segment3d_1_cam, Rox_Segment3D segment3d_2_cam, Rox_MatSE3 cam_T_obj, Rox_Cylinder3D cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment3d_1_cam || !segment3d_2_cam || !cam_T_obj || !cylinder3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the pose if the cylinder relative to the camera : cam_T_cyl
   Rox_MatSE3 obj_T_cyl = cylinder3d->T;
   Rox_MatSE3 cyl_T_cam = NULL;
   Rox_MatSE3 cam_T_cyl = NULL;

   error = rox_matse3_new(&cam_T_cyl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_mulmatmat(cam_T_cyl, cam_T_obj, obj_T_cyl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the pose of the camera relative to the cylinder : cyl_T_cam

   error = rox_matse3_new(&cyl_T_cam);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv(cyl_T_cam, cam_T_cyl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the tangent segments to the cylinder from the point O in cylinder frame
   Rox_Double ** T = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T, cyl_T_cam);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The origin of the camera frame in the cylinder frame
   Rox_Double O_x = T[0][3];
   Rox_Double O_y = T[1][3];

   Rox_Double h = cylinder3d->h;
   Rox_Double a = cylinder3d->a; 
   Rox_Double b = cylinder3d->b; 

   Rox_Segment3D_Struct segment3d_1_cyl;
   Rox_Segment3D_Struct segment3d_2_cyl;

   // Compute the tangent segments in the cylinder frame coordinates
   error =rox_segment3d_tangent_to_cylinder3d(&segment3d_1_cyl, &segment3d_2_cyl, a, b, h, O_x, O_y);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The segments are in the cylinder frame we need them in the camera frame
   error = rox_segment3d_transform(segment3d_1_cam, cam_T_cyl, &segment3d_1_cyl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment3d_transform(segment3d_2_cam, cam_T_cyl, &segment3d_2_cyl);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del(&cyl_T_cam);
   rox_matse3_del(&cam_T_cyl);

   return error;
}