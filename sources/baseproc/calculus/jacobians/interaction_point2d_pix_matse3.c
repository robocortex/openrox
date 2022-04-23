//==============================================================================
//
//    OPENROX   : File interaction_point2d_pix_matse3.c   
//
//    Contents  : Implementation of interaction_point2d_pix_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "interaction_point2d_pix_matse3.h"

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_interaction_matse3_point2d_pix (
   Rox_Matrix L, 
   const Rox_Point2D_Double pts, 
   const Rox_Double * z, 
   const Rox_MatUT3 K, 
   const Rox_Sint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !L || !pts || !z || !K ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size(L, 2*count, 6); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dL, L );   
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dK = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dK, K );   
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double k11 = dK[0][0]; 
   Rox_Double k12 = dK[0][1]; 
   Rox_Double k13 = dK[0][2]; 
   Rox_Double k22 = dK[1][1]; 
   Rox_Double k23 = dK[1][2];
   
   for ( Rox_Sint i = 0; i < count; i++)
   {  
      Rox_Double u  = pts[i].u;
      Rox_Double v  = pts[i].v;
      
      // Compute the inverse depth
      Rox_Double zi = 1.0 / z[i];
       
      // Translation : 1st row  
      dL[2*i][0] = -k11 * zi; 
      dL[2*i][1] = -k12 * zi; 
      dL[2*i][2] = -k13 * zi + u * zi;  
      
      // Rotation : 1st row  
      dL[2*i][3] = +k12 + ((k13 - u)*(k23 - v))/k22;
      dL[2*i][4] = -k11 + ((k13 - u)*(k12*k23 - k13*k22 + k22*u - k12*v))/(k11*k22);
      dL[2*i][5] = -((k23 - v)*(k11*k11 + k12*k12))/(k11*k22) - (k12*k13 - k12*u)/k11;
      
      // Translation : 2nd row  
      dL[2*i+1][0] =  0.0 ; 
      dL[2*i+1][1] = -k22 * zi ; 
      dL[2*i+1][2] = -k23 * zi + v * zi ;  
      
      // Rotation : 2nd row  
      dL[2*i+1][3] = k22 + (k23 - v)*(k23 - v)/k22;
      dL[2*i+1][4] = - ((k13 - u)*(k23 - v))/k11 + ((k23 - v)*(k12*k23 - k12*v))/(k11*k22);
      dL[2*i+1][5] = - (k12*k23 - k13*k22 + k22*u - k12*v)/k11;
   }

function_terminate:
   return error;
}