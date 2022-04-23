//==============================================================================
//
//    OPENROX   : File combination.c
//
//    Contents  : Implementation of combination module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "combination.h"
#include "combination_struct.h"

#include <stdio.h>

#include <generated/array2d_uint.h>
#include <generated/dynvec_array2d_uint.h>
#include <generated/dynvec_array2d_uint_struct.h>
#include <generated/dynvec_uint_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>
#include <baseproc/maths/base/basemaths.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/geometry/point/dynvec_point3d_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/numeric/dynvec_print.h>

Rox_ErrorCode rox_combination_new(Rox_Combination * combination, Rox_Uint nb_items, Rox_Uint nb_draws)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Combination ret = NULL;

   if (!combination) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (nb_items == 0 || nb_draws > nb_items) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Combination) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->bag = NULL;
   ret->draw = NULL;
   ret->combination_size = nb_draws;

   error = rox_dynvec_uint_new(&ret->bag, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_uint_new(&ret->draw, nb_draws); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint k = 0; k < nb_items; k++)
   {
      error = rox_dynvec_uint_append(ret->bag, &k);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *combination = ret;

function_terminate:
   if (error) rox_combination_del(&ret);
   return error;
}

Rox_ErrorCode rox_combination_del(Rox_Combination * combination)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Combination todel = NULL;

   if (!combination) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *combination;
   *combination = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_uint_del(&todel->bag);
   rox_dynvec_uint_del(&todel->draw);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_combination_draw(Rox_Combination combination)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!combination) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Uint count = combination->bag->used;
   rox_dynvec_uint_reset(combination->draw);

   for (Rox_Uint k = 0; k < combination->combination_size; k++)
   {
      Rox_Uint drawn = rox_rand() % count;

      Rox_Uint swap = combination->bag->data[drawn];

      combination->bag->data[drawn] = combination->bag->data[count-1];
      combination->bag->data[count-1] = swap;

      error = rox_dynvec_uint_append(combination->draw, &swap);
      ROX_ERROR_CHECK_TERMINATE ( error );

      count--;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_combination_get_draw(Rox_DynVec_Uint *draw, Rox_Combination combination)
{
   // TODO: get the draw from combination->draw
   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_combination_print_draw(Rox_Combination combination)
{
   rox_dynvec_uint_print(combination->draw);
   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_combination_print_bag(Rox_Combination combination)
{
   rox_dynvec_uint_print(combination->bag);
   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_combination_recursive(Rox_DynVec_Array2D_Uint comb, Rox_Array2D_Uint set, Rox_Array2D_Uint buffer, int start, int end, int index, int r);
Rox_ErrorCode rox_combination_create(Rox_Array2D_Uint set, Rox_Uint n, Rox_Uint r);

Rox_ErrorCode rox_combination_print(Rox_Combination combination)
{
   int n = combination->bag->used;
   int r = combination->combination_size;

   Rox_Array2D_Uint set = NULL;
   rox_array2d_uint_new(&set, 1, n);

   for ( int k = 0; k < n ; ++k)
   {
     rox_array2d_uint_set_value(set, 0, k, k+1);
   }

   rox_combination_create(set, n, r);

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_dynvec_array2d_uint_print(Rox_DynVec_Array2D_Uint input)
{
   for (Rox_Uint k=0; k<input->used; ++k)
   {
      rox_array2d_uint_print(input->data[k]);
   }

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_combination_create(Rox_Array2D_Uint set, Rox_Uint n, Rox_Uint r)
{
   Rox_DynVec_Array2D_Uint comb = NULL;

   // A temporary array to store all combination one by one
   Rox_Array2D_Uint buffer = NULL ;

   int size = rox_factorial(n)/(rox_factorial(n-r)*rox_factorial(r));

   rox_array2d_uint_new(&buffer, 1, r);

   rox_dynvec_array2d_uint_new(&comb, size);

   // Create and print all combination using temporary array 'buffer[]'
   rox_combination_recursive(comb, set, buffer, 0, n-1, 0, r);

   // rox_dynvec_array2d_uint_print(comb);

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_combination_recursive(Rox_DynVec_Array2D_Uint comb, Rox_Array2D_Uint set, Rox_Array2D_Uint buffer, int start, int end, int index, int r)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Current combination is ready to be printed, print it
   // Current combination is ready to be stored, store it
   if (index == r)
   {
      Rox_Array2D_Uint buffer_copy = NULL ;

      rox_array2d_uint_new(&buffer_copy, 1, r);

      rox_array2d_uint_copy(buffer_copy, buffer);

      rox_dynvec_array2d_uint_append(comb, &buffer_copy);

      rox_array2d_uint_print(buffer_copy);

      return ROX_ERROR_NONE;
   }

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data, buffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data_set = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data_set, set);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // replace index with all possible elements. The condition
   // "end-i+1 >= r-index" makes sure that including one element
   // at index will make a combination with remaining elements
   // at remaining positions
   for (int i=start; i<=end && end-i+1 >= r-index; i++)
   {
       data[0][index] = data_set[0][i];
       rox_combination_recursive(comb, set, buffer, i+1, end, index+1, r);
   }

function_terminate:
   return(error);
}
