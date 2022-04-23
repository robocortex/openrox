//==============================================================================
//
//    OPENROX   : File line_2d.c
//
//    Contents  : Implementation of line 2D module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "line2d.h"
#include "line2d_struct.h"

#include <math.h>
#include <float.h>

#include <system/errors/errors.h>

#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line2d_normal_signed_distance (
   Rox_Double * signed_distance, 
   const Rox_Line2D_Normal line2d_normal, 
   const Rox_Point2D_Double point2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!signed_distance || !line2d_normal || ! point2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the line parameters
   Rox_Double rho   = line2d_normal->rho;
   Rox_Double theta = line2d_normal->theta;

   // Compute sin and cos of angle
   Rox_Double sinth = sin(theta);
   Rox_Double costh = cos(theta);
   
   Rox_Double x = point2d->u;
   Rox_Double y = point2d->v;

   *signed_distance = rho - (x * costh + y * sinth);

function_terminate:
   return error;
}

Rox_ErrorCode rox_line2d_normal_unsigned_distance (
   Rox_Double * unsigned_distance, 
   const Rox_Line2D_Normal line2d_normal, 
   const Rox_Point2D_Double point2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !unsigned_distance || !line2d_normal || ! point2d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the line parameters
   Rox_Double rho   = line2d_normal->rho;
   Rox_Double theta = line2d_normal->theta;

   // Compute sin and cos of angle
   Rox_Double sinth = sin(theta);
   Rox_Double costh = cos(theta);
   
   Rox_Double x = point2d->u;
   Rox_Double y = point2d->v;

   *unsigned_distance = fabs(rho - (x * costh + y * sinth));

function_terminate:
   return error;
}



Rox_ErrorCode rox_line2d_transform_meters_to_pixels (   
   Rox_Line2D_Normal line2d_pixels,  
   Rox_Line2D_Normal line2d_meters,  
   const Rox_MatUT3 pix_K_met
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !line2d_meters || !line2d_pixels || !pix_K_met ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double ** dk = NULL; 
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dk, pix_K_met );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = dk[0][0];
   Rox_Double fv = dk[1][1];
   Rox_Double cu = dk[0][2];
   Rox_Double cv = dk[1][2];

   // Convert from meters to pixels
   Rox_Double costh = cos(line2d_meters->theta);
   Rox_Double sinth = sin(line2d_meters->theta);
   Rox_Double d = sqrt((fv*costh)*(fv*costh)+(fu*sinth)*(fu*sinth));
   
   if (d < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   line2d_pixels->theta = atan2 ( fu * sinth, fv * costh );
   line2d_pixels->rho = (fu*fv*line2d_meters->rho + cu*fv*costh + cv*fu*sinth) / d;

function_terminate:
   return error; 
}
