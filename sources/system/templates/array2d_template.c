//============================================================================
//
//    OPENROX   : File array2d_@LARRAY2DTYPENAME@.c
//
//    Contents  : Implementation of array2d_@LARRAY2DTYPENAME@ module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

typedef struct Rox_Array2D_Struct _Rox_Array2D_@ARRAY2DTYPENAME@;
typedef struct Rox_DynVec_Array2D_Struct _Rox_Array2D_@ARRAY2DTYPENAME@_Collection;

#define DIRECTINCLUDE_ARRAY2D

#include "generated/array2d_@LARRAY2DTYPENAME@.h"
#include <generated/dynvec_array2d.h>
#include <generated/dynvec_array2d_struct.h>
#include <generated/dynvec_@LARRAY2DTYPENAME@_@ARRAY2DSTRUCTNAME@.h>
#include <inout/system/errors_print.h>
#include <system/memory/array2d_struct.h>

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_new(Rox_Array2D_@ARRAY2DTYPENAME@ * obj, const Rox_Sint rows, const Rox_Sint cols)
{
   return rox_array2d_new((Rox_Array2D *) obj, (Rox_Datatype_Description)@ARRAY2DTYPEDESC@, rows, cols);
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_del(Rox_Array2D_@ARRAY2DTYPENAME@ * ptr)
{
   return rox_array2d_del((Rox_Array2D *) ptr);
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_new_copy(Rox_Array2D_@ARRAY2DTYPENAME@ * obj, const Rox_Array2D_@ARRAY2DTYPENAME@ input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_new(obj, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_copy(*obj, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_new_frombuffer(Rox_Array2D_@ARRAY2DTYPENAME@ * obj, const Rox_Sint rows, const Rox_Sint cols, Rox_Void * buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !buffer)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_new_frombuffer((Rox_Array2D *) obj, (Rox_Datatype_Description)@ARRAY2DTYPEDESC@, rows, cols, buffer);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_new_subarray2d (
   Rox_Array2D_@ARRAY2DTYPENAME@ * sub,
   Rox_Array2D_@ARRAY2DTYPENAME@ parent,
   const Rox_Sint initial_row,
   const Rox_Sint initial_col,
   const Rox_Sint rows,
   const Rox_Sint cols
)
{
   return rox_array2d_new_subarray2d((Rox_Array2D *) sub, (Rox_Array2D) parent, initial_row, initial_col, rows, cols);
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_subarray2d_shift(Rox_Array2D_@ARRAY2DTYPENAME@ sub, Rox_Array2D_@ARRAY2DTYPENAME@ parent, Rox_Sint initial_row, Rox_Sint initial_col)
{
   return rox_array2d_subarray2d_shift((Rox_Array2D) sub, (Rox_Array2D) parent, initial_row, initial_col);
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_create_blocksptr(Rox_Array2D_@ARRAY2DTYPENAME@ ptr, Rox_Sint rows)
{
   return rox_array2d_create_blocksptr((Rox_Array2D) ptr, rows);
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_data_pointer_to_pointer ( @ARRAY2DTYPE@ *** data_pointer, Rox_Array2D_@ARRAY2DTYPENAME@ array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_get_data_pointer_to_pointer( (Rox_Void ***) data_pointer, (Rox_Array2D) array);

   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_data_pointer( @ARRAY2DTYPE@ ** data_pointer, Rox_Array2D_@ARRAY2DTYPENAME@ array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_get_data_pointer ( (Rox_Void **) data_pointer, (Rox_Array2D) array);

   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_blocks ( @ARRAY2DTYPE@ **** blocks_pointer, Rox_Array2D_@ARRAY2DTYPENAME@ array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_get_blocks_pointer_to_pointer( (Rox_Void ****) blocks_pointer, (Rox_Array2D) array);

   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_size(Rox_Sint * rows, Rox_Sint * cols, const Rox_Array2D_@ARRAY2DTYPENAME@ array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_get_rows(rows, (Rox_Array2D) array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_get_cols(cols, (Rox_Array2D) array);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_rows(Rox_Sint * rows, const Rox_Array2D_@ARRAY2DTYPENAME@ array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !rows) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_get_rows(rows, (Rox_Array2D) array);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_cols(Rox_Sint * cols, const Rox_Array2D_@ARRAY2DTYPENAME@ array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !cols) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_get_cols(cols, (Rox_Array2D) array);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_stride(Rox_Sint * stride, const Rox_Array2D_@ARRAY2DTYPENAME@ array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !stride) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_get_stride(stride, (Rox_Array2D) array);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_match_size(Rox_Array2D_@ARRAY2DTYPENAME@ array1, Rox_Array2D_@ARRAY2DTYPENAME@ array2)
{
   return rox_array2d_match_size((Rox_Array2D) array1, (Rox_Array2D) array2);
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_check_size(Rox_Array2D_@ARRAY2DTYPENAME@ input, const Rox_Sint height, const Rox_Sint width)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols ( &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( cols != width ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows ( &rows, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( rows != height )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_copy(Rox_Array2D_@ARRAY2DTYPENAME@ dest, Rox_Array2D_@ARRAY2DTYPENAME@ source)
{
   return rox_array2d_copy ( (Rox_Array2D) dest, (Rox_Array2D) source );
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_set_value(Rox_Array2D_@ARRAY2DTYPENAME@ array, const Rox_Sint row, const Rox_Sint col, const @ARRAY2DTYPE@ val)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (row >= rows) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (col >= cols) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ((@ARRAY2DTYPE@ **)(((Rox_Array2D) array)->rows_ptr)) [row][col] = val;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_set_row(Rox_Array2D_@ARRAY2DTYPENAME@ array, const Rox_Sint i, const Rox_Array2D_@ARRAY2DTYPENAME@ row)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !row) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ((i >= rows) || (i<0))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_check_size(row, 1, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint j=0; j<cols; j++)
   {
      ((@ARRAY2DTYPE@ **)(((Rox_Array2D) array)->rows_ptr)) [i][j] = ((@ARRAY2DTYPE@ **)(((Rox_Array2D) row)->rows_ptr)) [0][j];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_set_col(Rox_Array2D_@ARRAY2DTYPENAME@ array, const Rox_Sint j, const Rox_Array2D_@ARRAY2DTYPENAME@ col)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !col) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ((j >= cols) || (j<0))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_check_size(col, rows, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i=0; i<rows; i++)
   {
      ((@ARRAY2DTYPE@ **)(((Rox_Array2D) array)->rows_ptr)) [i][j] = ((@ARRAY2DTYPE@ **)(((Rox_Array2D) col)->rows_ptr)) [i][0];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_col(Rox_Array2D_@ARRAY2DTYPENAME@ col, const Rox_Array2D_@ARRAY2DTYPENAME@ array, const Rox_Sint j)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !col) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ((j >= cols) || (j<0))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_check_size(col, rows, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i=0; i<rows; i++)
   {
      ((@ARRAY2DTYPE@ **)(((Rox_Array2D) col)->rows_ptr)) [i][0] = ((@ARRAY2DTYPE@ **)(((Rox_Array2D) array)->rows_ptr)) [i][j];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_value(@ARRAY2DTYPE@ *val, const Rox_Array2D_@ARRAY2DTYPENAME@ array, const Rox_Sint row, const Rox_Sint col)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint rows = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (row >= rows) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint cols = 0;
   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (col >= cols) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   *val = ((@ARRAY2DTYPE@ **)(((Rox_Array2D) array)->rows_ptr)) [row][col];

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_set_buffer_no_stride(Rox_Array2D_@ARRAY2DTYPENAME@ array, const @ARRAY2DTYPE@ * buf)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !buf)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   @ARRAY2DTYPE@ ** dout = NULL;

   error = rox_array2d_@LARRAY2DTYPENAME@_get_data_pointer_to_pointer ( &dout, array );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint pos = 0;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dout[i][j] = buf[pos];
         pos++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_get_buffer_no_stride(@ARRAY2DTYPE@ * buf, const Rox_Array2D_@ARRAY2DTYPENAME@ array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !buf)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   @ARRAY2DTYPE@ ** din = NULL;

   error = rox_array2d_@LARRAY2DTYPENAME@_get_data_pointer_to_pointer ( &din, array );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint pos = 0;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         buf[pos] = din[i][j];
         pos++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_set_buffer_no_stride_vertical_flip ( Rox_Array2D_@ARRAY2DTYPENAME@ array, const @ARRAY2DTYPE@ * buf )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array || !buf)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_@LARRAY2DTYPENAME@_get_cols(&cols, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_@LARRAY2DTYPENAME@_get_rows(&rows, array);
   ROX_ERROR_CHECK_TERMINATE ( error );

   @ARRAY2DTYPE@ ** dout = NULL;

   error = rox_array2d_@LARRAY2DTYPENAME@_get_data_pointer_to_pointer ( &dout, array );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint pos = 0;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dout[rows-i-1][j] = buf[pos];
         pos++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_collection_new(Rox_Array2D_@ARRAY2DTYPENAME@_Collection *obj, const Rox_Sint count, const Rox_Sint rows, const Rox_Sint cols)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_@ARRAY2DTYPENAME@_Collection ret = NULL;

   error = rox_dynvec_array2d_new(&ret, count);
   ROX_ERROR_CHECK_TERMINATE(error)

   for (Rox_Sint i = 0; i < count; i++)
   {
      Rox_Array2D_@ARRAY2DTYPENAME@ toadd;

      error = rox_array2d_@LARRAY2DTYPENAME@_new(&toadd, rows, cols);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_dynvec_array2d_append(ret, &toadd);
      if (error)
      {
         rox_array2d_@LARRAY2DTYPENAME@_del(&toadd);
         ROX_ERROR_CHECK_TERMINATE(error)
      }
   }

   *obj = ret;

function_terminate:
   if (error)
   {
      for (Rox_Uint i = 0; i < ret->used; i++)
      {
         rox_array2d_@LARRAY2DTYPENAME@_del(&ret->data[i]);
      }

      rox_dynvec_array2d_del(&ret);
   }

   return error;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_collection_del(Rox_Array2D_@ARRAY2DTYPENAME@_Collection * ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_@ARRAY2DTYPENAME@_Collection obj = NULL;

   if (!ptr) { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   obj = *ptr;
   *ptr = NULL;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   for (Rox_Uint i = 0; i < obj->used; i++)
   {
      rox_array2d_del(&obj->data[i]);
   }

   rox_dynvec_array2d_del(&obj);

function_terminate:
   return error;
}

Rox_Array2D_@ARRAY2DTYPENAME@ rox_array2d_@LARRAY2DTYPENAME@_collection_get(Rox_Array2D_@ARRAY2DTYPENAME@_Collection ptr, const Rox_Sint id)
{
   Rox_Array2D_@ARRAY2DTYPENAME@ result = NULL;

   if (!ptr) goto function_terminate;
   if (ptr->used <= (Rox_Uint) id) goto function_terminate;

   result = (Rox_Array2D_@ARRAY2DTYPENAME@) ptr->data[id];

function_terminate:
   return result;
}

Rox_Sint rox_array2d_@LARRAY2DTYPENAME@_collection_get_count(Rox_Array2D_@ARRAY2DTYPENAME@_Collection ptr)
{
   if (!ptr) return 0;

   return ptr->used;
}

Rox_ErrorCode rox_array2d_@LARRAY2DTYPENAME@_collection_cycleleft(Rox_Array2D_@ARRAY2DTYPENAME@_Collection ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_@ARRAY2DTYPENAME@ bck;

   if (!ptr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (ptr->used <= 1) {error = ROX_ERROR_NONE; goto function_terminate;}

   bck = ptr->data[0];

   for (Rox_Uint id = 0; id < ptr->used - 1; id++)
   {
      ptr->data[id] = ptr->data[id + 1];
   }

   ptr->data[ptr->used - 1] = bck;

function_terminate:
   return error;
}
