//==============================================================================
//
//    OPENROX   : File array2d.c
//
//    Contents  : Implementation of array2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d.h"
#include "array2d_struct.h"
#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_new_internal (
   Rox_Array2D * obj, 
   Rox_Datatype_Description datatype, 
   Rox_Sint rows, 
   Rox_Sint cols, 
   Rox_Void * buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D ret_array = NULL;
   Rox_Sint step = 0, mod = 0, modcols, minbytes;
   Rox_Sint bcount = 0, rcols = 0, dsize = 0, transpose = 0;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rows == 0 || cols == 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( (Rox_Sint) datatype == 0)
   { error = ROX_ERROR_BAD_TYPE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   // If it is a column vector, compute params as if row vector
   transpose = 0;
   if (cols == 1)
   {
      // swap cols and rows
      transpose = rows;
      rows = cols;
      cols = transpose;

      // set flag
      transpose = 1;
   }

   dsize = ROX_DATATYPE_BYTECOUNT(datatype);  // type specific size in bytes

   modcols = cols % 4;
   minbytes = (cols + ((modcols) ? (4 - modcols) : 0)) * dsize;

   bcount = (Rox_Sint) (dsize * cols);

   mod = bcount % ROX_ROW_BYTECOUNT_MULTIPLIER;
   step = bcount + ((mod) ? (ROX_ROW_BYTECOUNT_MULTIPLIER - mod) : 0);
   if (step < minbytes) step = minbytes;

   // The number of true cols that will be allocated
   rcols = step / dsize;

   // Create structure
   ret_array = (Rox_Array2D) rox_memory_allocate(sizeof(*ret_array), 1);
   if (!ret_array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Allocate data
   if (buffer == NULL)
   {
      error = rox_array_new ( &ret_array->data, datatype, rcols * rows );
   }
   else
   {
      error = rox_array_new_frombuffer(&ret_array->data, datatype, rcols * rows, buffer);
   }
   if (error)
   {
      rox_memory_delete(ret_array);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   //if transposed, reverse back for row pointers ... we want the same access pattern for column vector or row vectors
   if (transpose)
   {
      transpose = rows;
      rows = cols;
      cols = transpose;
      step = dsize;
   }

   //Create rows pointers
   ret_array->rows_ptr = rox_array2d_create_rowsptr ( ret_array->data->base_ptr, step, rows );
   if (!ret_array->rows_ptr)
   {
      rox_array_del(&ret_array->data);
      rox_memory_delete(ret_array);
      return 0;
   }

   //set block of rows pointers to 0 by default
   ret_array->block_ptr = 0;
   ret_array->nbblocks = 0;

   //Set information
   ret_array->cols = cols;
   ret_array->rows = rows;
   ret_array->step = step;
   ret_array->align_shift = 0;

   *obj = ret_array;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_new (
   Rox_Array2D * obj, 
   Rox_Datatype_Description datatype, 
   Rox_Sint rows, 
   Rox_Sint cols
)
{
   return rox_array2d_new_internal(obj, datatype, rows, cols, NULL);
}

Rox_ErrorCode rox_array2d_new_frombuffer (
   Rox_Array2D * obj, Rox_Datatype_Description datatype, const Rox_Sint rows, const Rox_Sint cols, Rox_Void * buffer)
{
   return rox_array2d_new_internal(obj, datatype, rows, cols, buffer);
}

Rox_ErrorCode rox_array2d_new_subarray2d ( Rox_Array2D * sub, Rox_Array2D parent, Rox_Sint initial_row, Rox_Sint initial_col, Rox_Sint rows, Rox_Sint cols)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D ret_array = NULL;
   Rox_Size dsize = 0;
   Rox_Schar *base_ptr = NULL;

   if (!parent || !sub) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *sub = 0;

   //Check ROI validity
   if (rows < 1 || rows > parent->rows)   
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (cols < 1 || cols > parent->cols)   
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (initial_row + rows > parent->rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (initial_col + cols > parent->cols) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Allocate new structure
   ret_array = (Rox_Array2D) rox_memory_allocate(sizeof(struct Rox_Array2D_Struct), 1);
   if (!ret_array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Use the parent rox_array
   rox_array_reference(&(parent->data));
   ret_array->data = parent->data;

   dsize = ROX_DATATYPE_BYTECOUNT(parent->data->datatype);

   // Set rows pointers
   base_ptr = ((Rox_Schar *) parent->rows_ptr[initial_row]) + dsize * initial_col;
   ret_array->rows_ptr = rox_array2d_create_rowsptr(base_ptr, parent->step, rows);
   if (!ret_array->rows_ptr)
   {
      rox_array_dereference(&(ret_array->data));
      rox_memory_delete(ret_array);
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   // Set information
   ret_array->cols = cols;
   ret_array->rows = rows;
   ret_array->step = parent->step;
   ret_array->align_shift = 0; // TODO to force alignment;
   ret_array->block_ptr = 0;
   ret_array->nbblocks = 0;

   *sub = ret_array;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_subarray2d_shift(Rox_Array2D sub, Rox_Array2D parent, Rox_Sint initial_row, Rox_Sint initial_col)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!parent || !sub) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //First check if wanted coordinates are ok
   if (initial_row + sub->rows >= parent->rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (initial_col + sub->cols >= parent->cols) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Size dsize = ROX_DATATYPE_BYTECOUNT(parent->data->datatype);
   Rox_Schar * cur_ptr = ((Rox_Schar *) parent->rows_ptr[initial_row]) + dsize * initial_col;

   //Update rows ptr
   for (Rox_Sint iter = 0; iter < sub->rows; iter++)
   {
      sub->rows_ptr[iter] = cur_ptr;
      cur_ptr += parent->step;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_del(Rox_Array2D *ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D todel = NULL;

   if (!ptr) 
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *ptr;
   *ptr = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   if (todel->data)
   {
      // Only dereference data array, delete if needed
      rox_array_dereference(&(todel->data));
   }

   // Delete rows
   if (todel->rows_ptr)
   {
      rox_array2d_delete_rowsptr(todel->rows_ptr, todel->rows);
   }

   // Delete block of rows
   if (todel->block_ptr)
   {
      rox_array2d_delete_blocksptr(todel->block_ptr, todel->nbblocks);
   }

   // Delete structure
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_Void **rox_array2d_create_rowsptr(Rox_Void * ptr_start, Rox_Sint step, Rox_Sint nbrows)
{
   Rox_Void ** ret = NULL;
   Rox_Schar * curptr = NULL;

   if (!ptr_start) return 0;

   ret = (Rox_Void **) rox_memory_allocate(sizeof(Rox_Void *), nbrows);
   if (!ret) return 0;

   curptr = (Rox_Schar *) ptr_start;
   for (Rox_Sint iter = 0; iter < nbrows; iter++)
   {
      ret[iter] = curptr;

      // Increment pointer by step
      curptr += step;
   }

   return ret;
}

Rox_Void rox_array2d_delete_rowsptr(Rox_Void **ptr, Rox_Sint nbrows)
{
   if (!ptr) return;

   rox_memory_delete(ptr);
}

Rox_ErrorCode rox_array2d_create_blocksptr(Rox_Array2D ptr, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Void ** curptr = NULL;

   if (!ptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Array rows must be a multiple of rows
   Rox_Sint modulo = ptr->rows % rows;
   if (modulo) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint countblocks = ptr->rows / rows;
   // Delete existing blocks pointers if any
   if (ptr->block_ptr) rox_array2d_delete_blocksptr(ptr->block_ptr, ptr->nbblocks);

   // Allocate
   ptr->block_ptr = (Rox_Void ** *)rox_memory_allocate(sizeof(Rox_Void **), countblocks);
   if (ptr->block_ptr == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Assign pointers
   curptr = &ptr->rows_ptr[0];
   for (Rox_Sint i = 0; i < countblocks; i++)
   {
      ptr->block_ptr[i] = curptr;
      curptr += rows;
   }

   ptr->nbblocks = countblocks;

function_terminate:
   return error;
}

Rox_Void rox_array2d_delete_blocksptr(Rox_Void *** ptr, Rox_Sint nbrows)
{
   if (!ptr) return;

   rox_memory_delete(ptr);
}

Rox_ErrorCode rox_array2d_copy(Rox_Array2D dest, Rox_Array2D source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint size = 0;

   if( !source )
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   if(!dest)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match(dest, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   size = source->cols * ROX_DATATYPE_BYTECOUNT(source->data->datatype);

   // To replace for full compliance with submatrices
   for (Rox_Sint i = 0; i < dest->rows; i++)
   {
      memcpy(dest->rows_ptr[i], source->rows_ptr[i], size);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_match(Rox_Array2D array1, Rox_Array2D array2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (array1 == 0 || array2 == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (array1->data == 0 || array2->data == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (array1->rows != array2->rows) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (array1->cols != array2->cols) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (array1->data->datatype != array2->data->datatype) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_match_size(Rox_Array2D array1, Rox_Array2D array2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (array1 == 0 || array2 == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (array1->rows != array2->rows) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (array1->cols != array2->cols) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_get_blocks_pointer_to_pointer ( Rox_Void **** blocks_pointer, Rox_Array2D array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !array )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *blocks_pointer = array->block_ptr;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_get_data_pointer ( Rox_Void ** pointer, Rox_Array2D array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !array )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *pointer = array->rows_ptr[0];

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_get_data_pointer_to_pointer ( Rox_Void *** pointer, Rox_Array2D array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (array == 0)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *pointer = array->rows_ptr;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_get_cols(Rox_Sint * cols, Rox_Array2D array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *cols = array->cols;

function_terminate:
      return error;
}

Rox_ErrorCode rox_array2d_get_rows(Rox_Sint * rows, Rox_Array2D array)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *rows = array->rows;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_get_stride ( Rox_Sint * stride, Rox_Array2D array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *stride = array->step;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_get_type ( Rox_Datatype_Description * dataype, Rox_Array2D array )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array) return (Rox_Datatype_Description) 0;
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array->data) return (Rox_Datatype_Description)0;
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *dataype = array->data->datatype;

function_terminate:
   return error;
}

