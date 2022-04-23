//==============================================================================
//
//    OPENROX   : File algse3.h
//
//    Contents  : API of algse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "algse3.h"
#include "ansi_algse3.h"

#include <baseproc/array/multiply/mulmatmat.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_algse3_set_velocity ( Rox_Array2D_Double algebra, Rox_Array2D_Double vector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !algebra ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }
   
   if ( !vector  ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_check_size ( algebra, 4, 4 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( vector, 6, 1 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** alg_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &alg_data, algebra );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** vec_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &vec_data, vector );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef old
   alg_data[0][0] = 0;
   alg_data[0][1] = -vec_data[5][0];
   alg_data[0][2] =  vec_data[4][0];
   alg_data[0][3] =  vec_data[0][0];

   alg_data[1][0] =  vec_data[5][0];
   alg_data[1][1] = 0;
   alg_data[1][2] = -vec_data[3][0];
   alg_data[1][3] =  vec_data[1][0];

   alg_data[2][0] = -vec_data[4][0];
   alg_data[2][1] =  vec_data[3][0];
   alg_data[2][2] = 0;
   alg_data[2][3] =  vec_data[2][0];

   alg_data[3][0] = 0;
   alg_data[3][1] = 0;
   alg_data[3][2] = 0;
   alg_data[3][3] = 0;
#else
   error = rox_ansi_array2d_double_algse3_set_velocity ( alg_data, vec_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

#endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_algse3_get_velocity ( Rox_Array2D_Double vector, Rox_Array2D_Double algebra )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !algebra ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }
   
   if ( !vector  ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_check_size ( algebra, 4, 4 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( vector, 6, 1 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** alg_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &alg_data, algebra );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** vec_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &vec_data, vector );
   ROX_ERROR_CHECK_TERMINATE ( error );

   vec_data[0][0] = alg_data[0][3];
   vec_data[1][0] = alg_data[1][3];
   vec_data[2][0] = alg_data[2][3];
   vec_data[3][0] = alg_data[2][1];
   vec_data[4][0] = alg_data[0][2];
   vec_data[5][0] = alg_data[1][0];

function_terminate:
   return error;
}
