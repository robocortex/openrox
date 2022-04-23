//==============================================================================
//
//    OPENROX   : File connectivity.c
//
//    Contents  : Implementation of connectivity module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "connectivity.h"
#include <generated/dynvec_point2d_sint_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uchar_connectivity (
   Rox_ObjSet_DynVec_Point2D_Sint lists, 
   Rox_Array2D_Uchar source, 
   Rox_Uint min_length
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Sint addedlist = NULL, stack = NULL;
   Rox_Point2D_Sint_Struct toadd, top;

   if (!lists || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_dynvec_point2d_sint_reset(lists);
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_sint_new(&stack, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         if (ds[i][j] == 0) continue;
         ds[i][j] = 0;

         if (addedlist)
         {
            rox_dynvec_point2d_sint_reset(addedlist);
         }
         else
         {
            error = rox_dynvec_point2d_sint_new(&addedlist, 10);
            ROX_ERROR_CHECK_TERMINATE ( error );
         }

         toadd.u = j;
         toadd.v = i;
         rox_dynvec_point2d_sint_append(stack, &toadd);

         while (stack->used)
         {
            // Pop element from stack
            top = stack->data[stack->used - 1];
            stack->used--;

            rox_dynvec_point2d_sint_append(addedlist, &top);


            for (Rox_Sint di = -1; di <= 1; di++)
            {
               Rox_Sint cy = top.v + di;
               if (cy < 0 || cy >= rows) continue;

               for (Rox_Sint dj = -1; dj <= 1; dj++)
               {
                  if (di == 0 && dj == 0) continue;
                  Rox_Sint cx = top.u + dj;

                  if (cx < 0 || cx >= cols) continue;

                  if (ds[cy][cx] != 0)
                  {
                     ds[cy][cx] = 0;

                     toadd.u = cx;
                     toadd.v = cy;
                     error = rox_dynvec_point2d_sint_append(stack, &toadd);
                     ROX_ERROR_CHECK_TERMINATE ( error );
                  }
               }
            }
         }

         if (addedlist->used > min_length)
         {
            error = rox_objset_dynvec_point2d_sint_append(lists, addedlist);
            ROX_ERROR_CHECK_TERMINATE ( error );
            addedlist = NULL;
         }
      }
   }

function_terminate:
   rox_dynvec_point2d_sint_del(&stack);
   rox_dynvec_point2d_sint_del(&addedlist);

   return error;
}

Rox_ErrorCode rox_array2d_uint_connectivity_value ( 
   Rox_ObjSet_DynVec_Point2D_Sint lists, 
   Rox_Array2D_Uint source, 
   Rox_Uint min_length
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint val;
   Rox_DynVec_Point2D_Sint addedlist = NULL, stack = NULL;
   Rox_Point2D_Sint_Struct toadd, top;

   if (!lists || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_dynvec_point2d_sint_reset(lists);

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_sint_new(&stack, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         if (ds[i][j] == 0) continue;
         val = ds[i][j];
         ds[i][j] = 0;

         if (addedlist)
         {
            rox_dynvec_point2d_sint_reset(addedlist);
         }
         else
         {
            error = rox_dynvec_point2d_sint_new(&addedlist, 10);
            ROX_ERROR_CHECK_TERMINATE(error)
         }

         toadd.u = j;
         toadd.v = i;
         rox_dynvec_point2d_sint_append(stack, &toadd);

         while (stack->used)
         {
            // Pop element from stack
            top = stack->data[stack->used - 1];
            stack->used--;

            rox_dynvec_point2d_sint_append(addedlist, &top);


            for (Rox_Sint di = -1; di <= 1; di++)
            {
               Rox_Sint cy = top.v + di;
               if (cy < 0 || cy >= rows) continue;

               for (Rox_Sint dj = -1; dj <= 1; dj++)
               {
                  if (di == 0 && dj == 0) continue;
                  Rox_Sint cx = top.u + dj;

                  if (cx < 0 || cx >= cols) continue;

                  if (ds[cy][cx] == val)
                  {
                     ds[cy][cx] = 0;

                     toadd.u = cx;
                     toadd.v = cy;
                     error = rox_dynvec_point2d_sint_append(stack, &toadd);
                     ROX_ERROR_CHECK_TERMINATE ( error );
                  }
               }
            }
         }

         if ( addedlist->used > min_length )
         {
            error = rox_objset_dynvec_point2d_sint_append(lists, addedlist);
            ROX_ERROR_CHECK_TERMINATE ( error );
            addedlist = NULL;
         }
      }
   }

function_terminate:
   rox_dynvec_point2d_sint_del(&stack);
   rox_dynvec_point2d_sint_del(&addedlist);

   return error;
}
