//==============================================================================
//
//    OPENROX   : File line_project.c
//
//    Contents  : Implementation of line_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include "line_project.h"
#include <baseproc/geometry/line/line2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line3d_planes_project_ (
   Rox_Line2D_Normal res, 
   Rox_Line3D_Planes line3d, 
   Rox_Double fu, 
   Rox_Double fv, 
   Rox_Double cu, 
   Rox_Double cv
)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Project line3d to line2d in meters
   error = rox_line3d_planes_project_meters ( res, line3d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Convert from meters to pixels
   Rox_Double costh = cos(res->theta);
   Rox_Double sinth = sin(res->theta);
   Rox_Double d = sqrt((fv*costh)*(fv*costh)+(fu*sinth)*(fu*sinth));
   
   if (d < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   res->theta = atan2(fu * sinth, fv * costh);
   res->rho = (fu*fv*res->rho + cu*fv*costh + cv*fu*sinth) / d;
   
function_terminate:
   return error;
}


Rox_ErrorCode rox_line3d_planes_project_meters (
   Rox_Line2D_Normal line2d, 
   Rox_Line3D_Planes line3d
)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !line2d || !line3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Get parameters of the first plane
   Rox_Double a1 = line3d->planes[0].a;
   Rox_Double b1 = line3d->planes[0].b;
   Rox_Double c1 = line3d->planes[0].c;
   Rox_Double d1 = line3d->planes[0].d;

   // Get parameters of the second plane
   Rox_Double a2 = line3d->planes[1].a;
   Rox_Double b2 = line3d->planes[1].b;
   Rox_Double c2 = line3d->planes[1].c;
   Rox_Double d2 = line3d->planes[1].d;

   // Compute the parameters of the projectef 2D line
   Rox_Double a = a1*d2-a2*d1; // = cos(theta)
   Rox_Double b = b1*d2-b2*d1; // = sin(theta)
   Rox_Double c = c1*d2-c2*d1; // = -rho

   Rox_Double s = sqrt( a * a + b * b );
   
   if (s < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   line2d->rho = - c / s ;
   line2d->theta = atan2 ( b, a );
   
function_terminate:
   return error;
}


Rox_ErrorCode rox_line3d_planes_project (
   Rox_Line2D_Normal line2d_pixels, 
   Rox_Line3D_Planes line3d, 
   const Rox_MatUT3 pix_K_met
)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Line2D_Normal_Struct line2d_meters;

   if ( !line2d_pixels || !line3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Project line3d to line2d in meters
   error = rox_line3d_planes_project_meters ( &line2d_meters, line3d );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Convert from meters to pixels 
   error = rox_line2d_transform_meters_to_pixels ( line2d_pixels, &line2d_meters, pix_K_met );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
