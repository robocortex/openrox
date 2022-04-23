//==============================================================================
//
//    OPENROX   : File line_transform.c
//
//    Contents  : Implementation of line_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <system/errors/errors.h>
#include "line_transform.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/plane/plane_transform.h>
#include <baseproc/geometry/line/line_from_planes.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line3d_planes_transform (
   Rox_Line3D_Planes line_out, 
   const Rox_Line3D_Planes line, 
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Plane3D_Double_Struct pl1, pl2;

   if (!line_out || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size(pose); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_transform ( &pl1.a, &pl1.b, &pl1.c, &pl1.d, pose, line->planes[0].a, line->planes[0].b, line->planes[0].c, line->planes[0].d ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_plane_transform ( &pl2.a, &pl2.b, &pl2.c, &pl2.d, pose, line->planes[1].a, line->planes[1].b, line->planes[1].c, line->planes[1].d ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_line3d_planes_from_2_planes( line_out, &pl1, &pl2 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_line3d_plucker_transform (
   Rox_Line3D_Plucker line_out, 
   const Rox_Line3D_Plucker line, 
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double E[3][3];
   Rox_Double nm1[3], nm2[3], nd[3];
   Rox_Double ** dt = NULL;

   if (!line_out || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size(pose); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   E[0][0] = -dt[2][3] * dt[1][0] + dt[1][3] * dt[2][0];
   E[0][1] = -dt[2][3] * dt[1][1] + dt[1][3] * dt[2][1];
   E[0][2] = -dt[2][3] * dt[1][2] + dt[1][3] * dt[2][2];
   E[1][0] = dt[2][3] * dt[0][0] - dt[0][3] * dt[2][0];
   E[1][1] = dt[2][3] * dt[0][1] - dt[0][3] * dt[2][1];
   E[1][2] = dt[2][3] * dt[0][2] - dt[0][3] * dt[2][2];
   E[2][0] = -dt[1][3] * dt[0][0] + dt[0][3] * dt[1][0];
   E[2][1] = -dt[1][3] * dt[0][1] + dt[0][3] * dt[1][1];
   E[2][2] = -dt[1][3] * dt[0][2] + dt[0][3] * dt[1][2];

   nm1[0] = dt[0][0] * line->moment[0] + dt[0][1] * line->moment[1] + dt[0][2] * line->moment[2];
   nm1[1] = dt[1][0] * line->moment[0] + dt[1][1] * line->moment[1] + dt[1][2] * line->moment[2];
   nm1[2] = dt[2][0] * line->moment[0] + dt[2][1] * line->moment[1] + dt[2][2] * line->moment[2];
   nm2[0] = E[0][0] * line->displacement[0] + E[0][1] * line->displacement[1] + E[0][2] * line->displacement[2];
   nm2[1] = E[1][0] * line->displacement[0] + E[1][1] * line->displacement[1] + E[1][2] * line->displacement[2];
   nm2[2] = E[2][0] * line->displacement[0] + E[2][1] * line->displacement[1] + E[2][2] * line->displacement[2];
   nd[0] = dt[0][0] * line->displacement[0] + dt[0][1] * line->displacement[1] + dt[0][2] * line->displacement[2];
   nd[1] = dt[1][0] * line->displacement[0] + dt[1][1] * line->displacement[1] + dt[1][2] * line->displacement[2];
   nd[2] = dt[2][0] * line->displacement[0] + dt[2][1] * line->displacement[1] + dt[2][2] * line->displacement[2];

   line_out->moment[0] = nm1[0] + nm2[0];
   line_out->moment[1] = nm1[1] + nm2[1];
   line_out->moment[2] = nm1[2] + nm2[2];

   line_out->displacement[0] = nd[0];
   line_out->displacement[1] = nd[1];
   line_out->displacement[2] = nd[2];
   
function_terminate:
   return error;
}
