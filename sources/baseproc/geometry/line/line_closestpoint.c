//==============================================================================
//
//    OPENROX   : File line_closestpoint.c
//
//    Contents  : Implementation of line_closestpoint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "line_closestpoint.h"
#include "line_from_points.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/line/line3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_segment3d_backproject (
   Rox_Point3D_Double res,
   const Rox_Segment3D segment3d,
   const Rox_Point2D_Double ray
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Line3D_Parametric_Struct line3d1;
   Rox_Line3D_Parametric_Struct line3d2;

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_line3d_parametric_from_2_point3d ( &line3d1, &(segment3d->points[0]), &(segment3d->points[1]) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point3D_Double_Struct pt1;
   Rox_Point3D_Double_Struct pt2;

   pt1.X = 0.0; pt1.Y = 0.0; pt1.Z = 0.0;
   pt2.X = ray->u; pt2.Y = ray->v; pt2.Z = 1.0;

   error = rox_line3d_parametric_from_2_point3d ( &line3d2, &pt1, &pt2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point3d_closests_from_2_lines3d ( res, &pt2, &line3d1, &line3d2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment3d_backproject_old (
   Rox_Point3D_Double res,
   const Rox_Segment3D segment,
   const Rox_Point2D_Double ray
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Line3D_Planes_Struct line;

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_line3d_planes_from_2_point3d ( &line, &(segment->points[0]), &(segment->points[1]) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Normalize to get unit normals
   Rox_Double n1 = sqrt ( line.planes[0].a * line.planes[0].a + line.planes[0].b * line.planes[0].b + line.planes[0].c * line.planes[0].c );
   Rox_Double n2 = sqrt ( line.planes[1].a * line.planes[1].a + line.planes[1].b * line.planes[1].b + line.planes[1].c * line.planes[1].c );

   if (fabs(n1) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (fabs(n2) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   line.planes[0].a /= n1;
   line.planes[0].b /= n1;
   line.planes[0].c /= n1;
   line.planes[0].d /= n1;

   line.planes[1].a /= n2;
   line.planes[1].b /= n2;
   line.planes[1].c /= n2;
   line.planes[1].d /= n2;

   Rox_Double m1 = line.planes[0].a * ray->u + line.planes[0].b * ray->v + line.planes[0].c;
   Rox_Double m2 = line.planes[1].a * ray->u + line.planes[1].b * ray->v + line.planes[1].c;

   Rox_Double m  = ( m1*m1 + m2*m2 );

   if (fabs(m) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double mu = ( m1 * -line.planes[0].d + m2 * -line.planes[1].d ) / m;

   res->X = ray->u * mu;
   res->Y = ray->v * mu;
   res->Z = mu;

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment3d_float_backproject (
   Rox_Point3D_Float res,
   const Rox_Segment3D_Float segment,
   const Rox_Point2D_Float ray
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Line3D_Planes_Struct line;

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_line3d_planes_from_2_point3d_float ( &line, &(segment->points[0]), &(segment->points[1]) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Normalize to get unit normals
   Rox_Double n1 = sqrt(line.planes[0].a * line.planes[0].a + line.planes[0].b * line.planes[0].b + line.planes[0].c * line.planes[0].c);
   Rox_Double n2 = sqrt(line.planes[1].a * line.planes[1].a + line.planes[1].b * line.planes[1].b + line.planes[1].c * line.planes[1].c);

   line.planes[0].a /= n1;
   line.planes[0].b /= n1;
   line.planes[0].c /= n1;
   line.planes[0].d /= n1;

   line.planes[1].a /= n2;
   line.planes[1].b /= n2;
   line.planes[1].c /= n2;
   line.planes[1].d /= n2;

   Rox_Double m1 = line.planes[0].a * ray->u + line.planes[0].b * ray->v + line.planes[0].c;
   Rox_Double m2 = line.planes[1].a * ray->u + line.planes[1].b * ray->v + line.planes[1].c;
   Rox_Double m = (m1*m1+m2*m2);
   Rox_Double mu = (m1 * -line.planes[0].d + m2 * -line.planes[1].d) / m;

   res->X = (Rox_Float) (ray->u * mu);
   res->Y = (Rox_Float) (ray->v * mu);
   res->Z = (Rox_Float) mu;

function_terminate:
   return error;
}

Rox_ErrorCode rox_linsys_closests_points ( Rox_Double * A, Rox_Double * b, Rox_Double * o1, Rox_Double * d1, Rox_Double * o2, Rox_Double * d2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double o[3];
   
   if ( !A || !b )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !o1 || !d1 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !o2 || !d2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !A || !b )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !o1 || !d1 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !o2 || !d2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   o[0] = o2[0] - o1[0];
   o[1] = o2[1] - o1[1];
   o[2] = o2[2] - o1[2];

   // Compute the linear system A * x = b
   A[0] =  (d1[0]*d1[0] + d1[1]*d1[1] + d1[2]*d1[2]);
   A[1] = -(d1[0]*d2[0] + d1[1]*d2[1] + d1[2]*d2[2]);
   A[2] =  (d2[0]*d1[0] + d2[1]*d1[1] + d2[2]*d1[2]);
   A[3] = -(d2[0]*d2[0] + d2[1]*d2[1] + d2[2]*d2[2]);

   b[0] = d1[0]*o[0] + d1[1]*o[1] + d1[2]*o[2];
   b[1] = d2[0]*o[0] + d2[1]*o[1] + d2[2]*o[2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_closests_from_2_lines3d (
   Rox_Point3D_Double pts_line1,
   Rox_Point3D_Double pts_line2,
   const Rox_Line3D_Parametric line3d1,
   const Rox_Line3D_Parametric line3d2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double A[4];
   Rox_Double b[2];

   error = rox_linsys_closests_points ( A, b, line3d1->origin, line3d1->direction, line3d2->origin, line3d2->direction );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Solve the linear system
   Rox_Double determinant = A[0]*A[3] - A[1]*A[2];

   if ( fabs(determinant) < FLT_EPSILON )
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double x1 = (A[3]*b[0] - A[1]*b[1])/determinant;
   Rox_Double x2 = (A[0]*b[1] - A[2]*b[0])/determinant;

   // Compute the point on the line 1
   error = rox_line3d_parametric_get_point3d ( pts_line1, line3d1, x1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the point on the line 2
   error = rox_line3d_parametric_get_point3d ( pts_line2, line3d2, x2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

// Rox_ErrorCode rox_point3d_closests_from_segment3d_line3d (
//    Rox_Point3D_Double pts_line1,
//    Rox_Point3D_Double pts_line2,
//    const Rox_Segment3D segment3d,
//    const Rox_Line3D_Parametric line3d2
// )
// {
//    Rox_ErrorCode error = ROX_ERROR_NONE;

// function_terminate:
//    return error;
// }