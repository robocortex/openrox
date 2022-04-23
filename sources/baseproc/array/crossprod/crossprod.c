//==============================================================================
//
//    OPENROX   : File crossprod.c
//
//    Contents  : Implementation of crossprod module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "crossprod.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_crossprod (
   Rox_Array2D_Double res, 
   const Rox_Array2D_Double one, 
   const Rox_Array2D_Double two
)
{
   // TODO: find better check of size and avoid display errors when verbose
   Rox_ErrorCode error = ROX_ERROR_NONE;
      
   if (!res) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   if (!one) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   if (!two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint nbrows_res = 0, nbcols_res = 0;
   Rox_Sint nbrows_one = 0, nbcols_one = 0;
   Rox_Sint nbrows_two = 0, nbcols_two = 0;
   
   error = rox_array2d_double_get_size(&nbrows_res, &nbcols_res, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&nbrows_one, &nbcols_one, one);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&nbrows_two, &nbcols_two, two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Bool valid_sizes = 0;
   if (  (nbrows_res == nbrows_one) && (nbrows_res == nbrows_two)
      && (nbcols_res == nbcols_one) && (nbcols_res == nbcols_two))
   {
      if (  ((nbrows_res == 3) && (nbcols_res == 1))
         || ((nbrows_res == 1) && (nbcols_res == 3)))
      {
         valid_sizes = 1;
      }
   }

   if (valid_sizes == 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double ** dres = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** done = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &done, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dtwo = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dtwo, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double x1 = done[0][0];
   Rox_Double y1 = done[0][1];
   Rox_Double z1 = done[0][2];
   
   Rox_Double x2 = dtwo[0][0];
   Rox_Double y2 = dtwo[0][1];
   Rox_Double z2 = dtwo[0][2];
   
   dres[0][0] = y1 * z2 - z1 * y2;
   dres[0][1] = z1 * x2 - x1 * z2;
   dres[0][2] = x1 * y2 - y1 * x2;

function_terminate:
   return error;
}