//==============================================================================
//
//    OPENROX   : File interaction_point2d_nor_matse3.c
//
//    Contents  : Implementation of interaction_point2d_nor_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "interaction_point2d_nor_matse3.h"

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_interaction_matse3_point2d_nor (
   Rox_Matrix L, 
   const Rox_Point2D_Double pts, 
   const Rox_Double * z, 
   const Rox_Sint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!L || !pts || !z) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(L, 2*count, 6); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL, L);

   for ( Rox_Sint i = 0; i < count; i++)
   {
      Rox_Double zi = 1.0 / z[i];

      // Translation : 1st row 
      dL[2*i][0] = -zi ;
      dL[2*i][1] = 0.0 ;
      dL[2*i][2] = +zi * pts[i].u;

      // Rotation : 1st row 
      dL[2*i][3] = + pts[i].u * pts[i].v ;
      dL[2*i][4] = - pts[i].u * pts[i].u - 1.0;
      dL[2*i][5] = + pts[i].v ;

      // Translation : 2nd row 
      dL[2*i+1][0] =  0.0 ;
      dL[2*i+1][1] = -zi ;
      dL[2*i+1][2] = +zi * pts[i].v ;

      // Rotation : 2nd row 
      dL[2*i+1][3] = + pts[i].v * pts[i].v + 1.0 ;
      dL[2*i+1][4] = - pts[i].u * pts[i].v ;
      dL[2*i+1][5] = - pts[i].u ;
   }

function_terminate:
   return error;
}
