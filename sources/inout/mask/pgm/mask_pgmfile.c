//============================================================================
//
//    OPENROX   : File mask_pgmfile.h
//
//    Contents  : API of mask_pgmfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "mask_pgmfile.h"

#include <baseproc/array/conversion/array2d_uint_from_uchar.h>
#include <baseproc/array/conversion/array2d_uchar_from_uint.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_mask_new_pgm(Rox_Array2D_Uint * out, const char * path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar source = NULL;
   Rox_Array2D_Uint ret = NULL;

   if(!out || !path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *out = NULL;

   error = rox_array2d_uchar_new_pgm(&source, path); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_from_uchar_mask(ret, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *out = ret;

function_terminate:
   if (error) rox_array2d_uint_del(&ret);
   rox_array2d_uchar_del(&source);
   return error;
}

Rox_ErrorCode rox_array2d_uint_mask_read_pgm(Rox_Array2D_Uint out, const char * path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar source = NULL;

   if(!out || !path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_new_pgm(&source, path); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(out, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_from_uchar_mask(out, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_uchar_del(&source);
   return error;
}

Rox_ErrorCode rox_array2d_uint_mask_save_pgm(const char * path, Rox_Array2D_Uint in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar source = NULL;

   if(!in || !path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&source, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_from_uint_mask(source, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_save_pgm(path, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_uchar_del(&source);
   return error;
}
