//==============================================================================
//
//    OPENROX   : File ssd.c
//
//    Contents  : Implementation of ssd module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ssd.h"
#include "ansi_ssd.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_ss_mask (
   Rox_Double * ss, 
   Rox_Array2D_Float array2d, 
   Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ss) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   if (!array2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   if (!mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, array2d); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &array2d_data, array2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   #ifdef old

   Rox_Double ss_tmp = 0.0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (mask_data[i][j] == 0) continue;

         ss_tmp += array2d_data[i][j] * array2d_data[i][j];
      }
   }

   *ss = ss_tmp;

   #else
   error = rox_ansi_array2d_float_ss_mask ( ss, array2d_data, rows, cols, mask_data );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_ssd (
   Rox_Double * ssd,
   const Rox_Array2D_Double array2d_1,
   const Rox_Array2D_Double array2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ssd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_1_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      #ifdef old

   Rox_Double ssd_tmp = 0.0;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Double dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         ssd_tmp += dif*dif;
      }
   }

   *ssd = ssd_tmp;

   #else
   error = rox_ansi_array2d_double_ssd ( ssd, array2d_1_data, array2d_2_data, rows, cols );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_ssd (
   Rox_Float * ssd,
   const Rox_Array2D_Float array2d_1,
   const Rox_Array2D_Float array2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ssd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_1_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_2_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   #ifdef old

   Rox_Float ssd_tmp = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         ssd_tmp += dif*dif;
      }
   }

   *ssd = ssd_tmp;

   #else
   error = rox_ansi_array2d_float_ssd ( ssd, array2d_1_data, array2d_2_data, rows, cols );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_ssd (
   Rox_Float * ssd,
   const Rox_Array2D_Uchar array2d_1,
   const Rox_Array2D_Uchar array2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ssd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_1_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_2_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   #ifdef old

   Rox_Float ssd_tmp = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = (Rox_Float) array2d_2_data[r][c] - (Rox_Float) array2d_1_data[r][c];

         ssd_tmp += dif*dif;
      }
   }

   *ssd = ssd_tmp;

   #else
   error = rox_ansi_array2d_uchar_ssd ( ssd, array2d_1_data, array2d_2_data, rows, cols );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_ssd_mask (
   Rox_Double * ssd,
   const Rox_Array2D_Double array2d_1,
   const Rox_Array2D_Double array2d_2,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ssd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_1_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   #ifdef old

   Rox_Double ssd_tmp = 0.0;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Double dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            ssd_tmp += dif*dif;
         }
      }
   }

   *ssd = ssd_tmp;

   #else
   error = rox_ansi_array2d_double_ssd_mask ( ssd, array2d_1_data, array2d_2_data, rows, cols, mask_data );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_ssd_mask (
   Rox_Float * ssd,
   const Rox_Array2D_Float array2d_1,
   const Rox_Array2D_Float array2d_2,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ssd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_1_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_2_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );
   #ifdef old

   Rox_Float ssd_tmp = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            ssd_tmp += dif*dif;
         }
      }
   }

   *ssd = ssd_tmp;

   #else
   error = rox_ansi_array2d_float_ssd_mask ( ssd, array2d_1_data, array2d_2_data, rows, cols, mask_data );
   #endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_ssd_mask (
   Rox_Float * ssd,
   const Rox_Array2D_Uchar array2d_1,
   const Rox_Array2D_Uchar array2d_2,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ssd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_1_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_2_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );
   #ifdef old

   Rox_Float ssd_tmp = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = (Rox_Float) array2d_2_data[r][c] - (Rox_Float) array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            ssd_tmp += dif*dif;
         }
      }
   }

   *ssd = ssd_tmp;

   #else
   error = rox_ansi_array2d_uchar_ssd_mask ( ssd, array2d_1_data, array2d_2_data, rows, cols, mask_data );
   #endif

function_terminate:
   return error;
}
