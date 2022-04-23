//==============================================================================
//
//    OPENROX   : File svdsort.c
//
//    Contents  : Implementation of svdsort module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "svdsort.h"
#include <inout/system/errors_print.h>

//! Structure 
struct Rox_SVD_Order
{
   //! To be commented  
   int pos;
   //! To be commented  
   Rox_Double val;
};

int sort_svd_order(struct Rox_SVD_Order * one, struct Rox_SVD_Order * two);

int sort_svd_order(struct Rox_SVD_Order * one, struct Rox_SVD_Order * two)
{
   if (one->val > two->val) return -1;
   else if (one->val < two->val) return 1;
   else return 0;
}

Rox_ErrorCode rox_array2d_double_svd_sort(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   struct Rox_SVD_Order * bestorder = NULL;
   Rox_Double * buffer = NULL;
   Rox_Double ** ds = NULL, **du = NULL, **dv = NULL;

   if (!U || !S || !V) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint uh = 0, uw = 0;
   error = rox_array2d_double_get_size(&uh, &uw, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint vh = 0, vw = 0;
   error = rox_array2d_double_get_size(&vh, &vw, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint sh = 0, sw = 0;
   error = rox_array2d_double_get_size(&sh, &sw, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (sw != 1) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (vw < sh) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (uh < sh) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   bestorder = (struct Rox_SVD_Order *) rox_memory_allocate(sizeof(struct Rox_SVD_Order), sh); 
   if (!bestorder) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   buffer = (Rox_Double *) rox_memory_allocate(sizeof(Rox_Double), sh);
   if (!buffer)
   { rox_memory_delete(bestorder);
     {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   error = rox_array2d_double_get_data_pointer_to_pointer ( &ds, S );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &du, U );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dv, V );

   for (Rox_Sint k = 0; k < sh; k++)
   {
      bestorder[k].pos = k;
      bestorder[k].val = ds[k][0];
   }

   qsort(bestorder, sh, sizeof(struct Rox_SVD_Order), (int(*)(const void*, const void*))sort_svd_order);

   for (Rox_Sint k = 0; k < sh; k++)
   {
      ds[k][0] = bestorder[k].val;
   }

   for (Rox_Sint l = 0; l < uh; l++)
   {
      for (Rox_Sint k = 0; k < sh; k++)
      {
         buffer[k] = du[l][bestorder[k].pos];
      }

      for (Rox_Sint k = 0; k < sh; k++)
      {
         du[l][k] = buffer[k];
      }

      for (Rox_Sint k = sh; k < uw; k++)
      {
         du[l][k] =0;
      }
   }

   for (Rox_Sint l = 0; l < vh; l++)
   {
      for (Rox_Sint k = 0; k < sh; k++)
      {
         buffer[k] = dv[l][bestorder[k].pos];
      }

      for (Rox_Sint k = 0; k < sh; k++)
      {
         dv[l][k] = buffer[k];
      }

      for (Rox_Sint k = sh; k < vw; k++)
      {
         dv[l][k] = 0;
      }
   }

function_terminate:

   rox_memory_delete(bestorder);
   rox_memory_delete(buffer);
   
   return error;
}

Rox_ErrorCode rox_array2d_double_svd_sort_SV(Rox_Array2D_Double S, Rox_Array2D_Double V)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   struct Rox_SVD_Order * bestorder = NULL;
   Rox_Double * buffer = NULL;

   if (!S || !V) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint vh = 0, vw = 0;
   error = rox_array2d_double_get_size(&vh, &vw, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint sh = 0, sw = 0;
   error = rox_array2d_double_get_size(&sh, &sw, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (sw != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (vh < sh) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   bestorder = (struct Rox_SVD_Order *) rox_memory_allocate(sizeof(struct Rox_SVD_Order), vh);
   if (!bestorder) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   buffer = (Rox_Double *)rox_memory_allocate(sizeof(Rox_Double), vh);
   if (!buffer)
   {
      rox_memory_delete(buffer);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   Rox_Double ** ds = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &ds, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dv, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k = 0; k < sh; k++)
   {
      bestorder[k].pos = k;
      bestorder[k].val = ds[k][0];
   }
   for (Rox_Sint k = sh; k < vh; k++)
   {
      bestorder[k].pos = k;
      bestorder[k].val = 0;
   }

   qsort(bestorder, vh, sizeof(struct Rox_SVD_Order), (int(*)(const void*, const void*))sort_svd_order);

   for (Rox_Sint k = 0; k < sh; k++)
   {
      ds[k][0] = bestorder[k].val;
   }

   for (Rox_Sint l = 0; l < vh; l++)
   {
      for (Rox_Sint k = 0; k < vw; k++)
      {
         buffer[k] = dv[l][bestorder[k].pos];
      }

      for (Rox_Sint k = 0; k < vw; k++)
      {
         dv[l][k] = buffer[k];
      }
   }

function_terminate:
   rox_memory_delete(bestorder);
   rox_memory_delete(buffer);
   return error;
}
