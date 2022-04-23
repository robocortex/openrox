//==============================================================================
//
//    OPENROX   : File points2d_tools.c
//
//    Contents  : Implementation of points3d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point2d_tools.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <inout/system/errors_print.h>
#include <system/errors/errors.h>
#include <generated/dynvec_point2d_double_struct.h>

#define THRESH_NUMERICAL_ZERO 1e-16

Rox_ErrorCode rox_point2d_double_copy (
   Rox_Point2D_Double output,
   const Rox_Point2D_Double input
   )
{
   // TODO check inputs
   output->u = input->u;
   output->v = input->v;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_point2d_float_convert_double (
   Rox_Point2D_Double output,
   const Rox_Point2D_Float input
)
{
   // TODO check inputs
   output->u = input->u;
   output->v = input->v;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_point2d_convert_pixel_float_to_meter_float (
   Rox_Point2D_Float point_nor,
   const Rox_Point2D_Float point_pix,
   const Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO check inputs
   point_nor->u = (Rox_Float) ((point_pix->u - K_data[0][2])/K_data[0][0]);
   point_nor->v = (Rox_Float) ((point_pix->v - K_data[1][2])/K_data[1][1]);

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_convert_pixel_double_to_meter_double (
   Rox_Point2D_Double point_nor,
   const Rox_Point2D_Double point_pix,
   const Rox_MatUT3 pix_K_met
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point_nor || !pix_K_met || !point_pix ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, pix_K_met );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO check inputs
   point_nor->u = (point_pix->u - K_data[0][2])/K_data[0][0];
   point_nor->v = (point_pix->v - K_data[1][2])/K_data[1][1];

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_convert_pixel_float_to_meter_double (
   Rox_Point2D_Double point_nor,
   const Rox_Point2D_Float point_pix,
   const Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   point_nor->u = ( point_pix->u - K_data[0][2] ) / K_data[0][0];
   point_nor->v = ( point_pix->v - K_data[1][2] ) / K_data[1][1];
 
function_terminate:
   return error;
}


Rox_ErrorCode rox_point2d_convert_meter_double_to_pixel_float (
   Rox_Point2D_Float point_pix, 
   const Rox_Point2D_Double point_nor, 

   const Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   point_pix->u = (Rox_Float) (K_data[0][0]*point_nor->u + K_data[0][2]);
   point_pix->v = (Rox_Float) (K_data[1][1]*point_nor->v + K_data[1][2]);

function_terminate:
   return error;
}


ROX_API Rox_ErrorCode rox_point2d_float_distance (
   Rox_Float * distance, 
   const Rox_Point2D_Float point2d_1, 
   const Rox_Point2D_Float point2d_2 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !distance || !point2d_1 || !point2d_2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Float du = point2d_1->u - point2d_2->u;
   Rox_Float dv = point2d_1->v - point2d_2->v;
 
   *distance = du * du + dv * dv;

   *distance = sqrtf(*distance);


function_terminate:
   return error;
}

ROX_API Rox_ErrorCode rox_point2d_double_distance (
   Rox_Double * distance, 
   const Rox_Point2D_Double point2d_1, 
   const Rox_Point2D_Double point2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !distance || !point2d_1 || !point2d_2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   Rox_Double du = point2d_1->u - point2d_2->u;
   Rox_Double dv = point2d_1->v - point2d_2->v;
 
   *distance = du * du + dv * dv;

   *distance = sqrt(*distance);
 
function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_double_compute_bounding_box ( 
   Rox_Point2D_Double bounding_box, 
   const Rox_Double * points_list,
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !bounding_box || !points_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double min_u = points_list[0];
   Rox_Double min_v = points_list[1];

   Rox_Double max_u = points_list[0];
   Rox_Double max_v = points_list[1];

   for ( Rox_Sint k = 1; k < nb_points; k++ )
   {
      if ( points_list[ 2*k + 0 ] <  min_u  ) min_u = points_list[ 2*k + 0 ];
      if ( points_list[ 2*k + 0 ] >  max_u  ) max_u = points_list[ 2*k + 0 ];
      if ( points_list[ 2*k + 1 ] <  min_v  ) min_v = points_list[ 2*k + 1 ];
      if ( points_list[ 2*k + 1 ] >  max_v  ) max_v = points_list[ 2*k + 1 ];
   }

   // Compute the 8 points of the 3D bounding box
   bounding_box[0].u = min_u; bounding_box[1].u = min_u; 
   bounding_box[0].v = min_v; bounding_box[1].v = max_v; 

   bounding_box[2].u = max_u; bounding_box[3].u = max_u; 
   bounding_box[2].v = min_v; bounding_box[3].v = max_v; 

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point2d_double_compute_bounding_box ( 
   Rox_Point2D_Double bounding_box, 
   const Rox_Point2D_Double points_list,
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !bounding_box || !points_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double min_u = points_list[0].u;
   Rox_Double min_v = points_list[0].v;

   Rox_Double max_u = points_list[1].u;
   Rox_Double max_v = points_list[1].v;

   for ( Rox_Sint k = 1; k < nb_points; k++ )
   {
      if ( points_list[ k ].u <  min_u  ) min_u = points_list[ k ].u;
      if ( points_list[ k ].u >  max_u  ) max_u = points_list[ k ].u;
      if ( points_list[ k ].v <  min_v  ) min_v = points_list[ k ].v;
      if ( points_list[ k ].v >  max_v  ) max_v = points_list[ k ].v;
   }

   // Compute the 4 points of the 3D bounding box
   bounding_box[0].u = min_u; bounding_box[1].u = min_u; 
   bounding_box[0].v = min_v; bounding_box[1].v = max_v; 

   bounding_box[2].u = max_u; bounding_box[3].u = max_u; 
   bounding_box[2].v = min_v; bounding_box[3].v = max_v; 

function_terminate:
   return error;
}
