//==============================================================================
//
//    OPENROX   : File sl3interfrom3dpoints.c
//
//    Contents  : Implementation of sl3interfrom3dpoints module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sl3interfrom3dpoints.h"

#include <baseproc/geometry/plane/plane_3points.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sl3_from_3d_points_double (
   Rox_Plane3D_Double plane, 
   Rox_MatSL3 homography, 
   Rox_MatSE3 pose, 
   const Rox_Point3D_Double vertices, 
   const Rox_Sint width, 
   const Rox_Sint height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double vx, vy, vz, norm, angle;
   Rox_MatSE3 Tplan = NULL, Trot = NULL, Ttotal = NULL;
   Rox_MatSL3 Hi = NULL;
   Rox_Array2D_Double vec = NULL, res = NULL, res2 = NULL;
   Rox_Double **dr = NULL, **dr2 = NULL, **dtr = NULL;
   Rox_Point2D_Double_Struct srcpts[4], destpts[4];

   if (!plane || !homography || !pose || !vertices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute plane in world coordinates
   error = rox_plane3d_from_3_point3d ( plane, &vertices[0], &vertices[1], &vertices[2]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check that fourth point belong to the plane
   Rox_Double dist = plane->a*vertices[3].X + plane->b*vertices[3].Y + plane->c*vertices[3].Z + plane->d;
   if (dist > 1e-3) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create buffers
   error = rox_matse3_new ( &Tplan );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_new ( &Trot );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_new ( &Ttotal );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_new ( &Hi );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new ( &vec, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new (&res, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&res2, 4, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dr2, res2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation parameters so that plane normal become [0 0 1]
   vx = +plane->b;
   vy = -plane->a;
   vz = 0;
   norm = sqrt(vx*vx+vy*vy);
   if (norm > DBL_EPSILON)
   {
      vx = vx / norm;
      vy = vy / norm;
      vz = vz / norm;
      angle = acos(plane->c);

      // rotation
      error = rox_matse3_set_axis_angle ( Tplan, vx, vy, vz, angle ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Z Translation parameter so that plane distance become 0
   rox_array2d_double_set_value(vec, 0, 0, vertices[0].X);
   rox_array2d_double_set_value(vec, 1, 0, vertices[0].Y);
   rox_array2d_double_set_value(vec, 2, 0, vertices[0].Z);
   rox_array2d_double_set_value(vec, 3, 0, 1);
   rox_array2d_double_mulmatmat(res, Tplan, vec);
   rox_array2d_double_set_value(Tplan, 2, 3, -dr[2][0]);

   // Constraint first vector to lie on the orthonormal axis
   rox_array2d_double_set_value(vec, 0, 0, vertices[0].X);
   rox_array2d_double_set_value(vec, 1, 0, vertices[0].Y);
   rox_array2d_double_set_value(vec, 2, 0, vertices[0].Z);
   rox_array2d_double_set_value(vec, 3, 0, 1);
   rox_array2d_double_mulmatmat(res, Tplan, vec);

   rox_array2d_double_set_value(vec, 0, 0, vertices[1].X);
   rox_array2d_double_set_value(vec, 1, 0, vertices[1].Y);
   rox_array2d_double_set_value(vec, 2, 0, vertices[1].Z);
   rox_array2d_double_set_value(vec, 3, 0, 1);
   rox_array2d_double_mulmatmat(res2, Tplan, vec);

   rox_array2d_double_fillunit(Trot);

   Rox_Double dx = dr2[0][0] - dr[0][0];
   Rox_Double dy = dr2[1][0] - dr[1][0];
   norm = sqrt(dx*dx+dy*dy);
   if (norm > DBL_EPSILON)
   {
      dx /= norm;
      //dy /= norm;
      rox_array2d_double_get_data_pointer_to_pointer( &dtr, Trot);

      dtr[0][0] = fabs(dx); dtr[0][1] = sin(acos(fabs(dx)));
      dtr[1][0] = -sin(acos(fabs(dx))); dtr[1][1] = fabs(dx);
   }

   error = rox_matse3_mulmatmat ( Ttotal, Trot, Tplan );
   ROX_ERROR_CHECK_TERMINATE ( error );


   // Estimates four points in the new frame
   for ( Rox_Sint idpt = 0; idpt < 4; idpt++ )
   {
      rox_array2d_double_set_value(vec, 0, 0, vertices[idpt].X);
      rox_array2d_double_set_value(vec, 1, 0, vertices[idpt].Y);
      rox_array2d_double_set_value(vec, 2, 0, vertices[idpt].Z);
      rox_array2d_double_set_value(vec, 3, 0, 1);
      rox_array2d_double_mulmatmat(res, Ttotal, vec);

      srcpts[idpt].u = dr[0][0];
      srcpts[idpt].v = dr[1][0];
  }

   // Find homography between new frame and template
   destpts[0].u = - 0.5; destpts[1].u = width - 0.5; destpts[2].u = width  - 0.5; destpts[3].u = -0.5; 
   destpts[0].v = - 0.5; destpts[1].v =       - 0.5; destpts[2].v = height - 0.5; destpts[3].v = height -0.5;

   error = rox_matsl3_from_4_points_double ( Hi, srcpts, destpts ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_copy ( homography, Hi ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_copy ( pose, Ttotal ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del ( &Tplan );
   rox_array2d_double_del ( &Hi );
   rox_array2d_double_del ( &vec );
   rox_array2d_double_del ( &res );
   rox_array2d_double_del ( &res2 );
   rox_array2d_double_del ( &Trot );
   rox_array2d_double_del ( &Ttotal );

   return error;
}
