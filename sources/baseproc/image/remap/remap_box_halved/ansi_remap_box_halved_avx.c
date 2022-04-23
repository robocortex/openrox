//============================================================================
//
//    OPENROX   : File ansi_remap_box_halved_avx.c
//
//    Contents  : Implementation of remap_box_halved module with AVX optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_remap_box_halved.h"
#include <system/vectorisation/avx.h>
#include <inout/system/errors_print.h>

int rox_ansi_remap_box_nomask_uchar_to_uchar_halved (
   unsigned char ** dd,
   unsigned char ** ds,
   int hrows, // rows of the halved image
   int hcols  // cols of the halved image
)
{
   int error = 0;
   unsigned short buffer1[8];
   unsigned short buffer2[8];

   __m128i add_rows;
   __m128i add_cols;
   __m128i mean;
   __m128i avx_ds, avx_dsn;
   
   for (int v = 0; v < hrows; v++)
   {
      unsigned char * ptr_ds  = ds[2*v];
      unsigned char * ptr_dsn = ds[2*v+1];

      for (int u = 0; u < hcols; u+=4)
      {
         for (int k=0; k<8; k++) buffer1[k] = (unsigned short) ptr_ds[k];
         for (int k=0; k<8; k++) buffer2[k] = (unsigned short) ptr_dsn[k];

         avx_ds = _mm_loadu_si128(( __m128i* )buffer1);   // 
         avx_dsn = _mm_loadu_si128(( __m128i* )buffer2);

         // Add two rows
         add_rows = _mm_add_epi16(avx_ds, avx_dsn);

         // Add two cols
         add_cols = _mm_hadd_epi16 ( add_rows, add_rows );

         // Divide by 4.0
         mean = _mm_srai_epi16 ( add_cols, 2 );

         // Get the result
         _mm_storeu_si128 ( ( __m128i* ) buffer1, mean );

         // dd[v][u] = (ds[dv][du] + ds[dv + 1][du] + ds[dv][du + 1] + ds[dv + 1][du + 1]) / 4;

         dd[v][u  ] = (unsigned char) buffer1[0];
         dd[v][u+1] = (unsigned char) buffer1[1];
         dd[v][u+2] = (unsigned char) buffer1[2];
         dd[v][u+3] = (unsigned char) buffer1[3];

         ptr_ds += 8;
         ptr_dsn += 8;
      }
   }
   return error;
}

#ifdef old
Rox_ErrorCode rox_remap_box_nomask_uchar_to_uchar_halved (
   Rox_Image dest, 
   const Rox_Image source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uchar_get_size ( &rows, &cols, source ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_uchar_check_size ( dest, hrows, hcols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_remap_box_nomask_uchar_to_uchar_halved ( dd, ds, hrows, hcols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
#endif 

int rox_ansi_array2d_float_remap_halved_box_suboptimal_data_store_avx (
   float ** dd,
   int hrows, // rows of the halved image
   int hcols, // cols of the halved image
   float ** ds
)
{
   int error = 0;
   union avx_vector buffer_8_bytes;

   __m256 avx_4 = _mm256_set1_ps(4);

   for (int v = 0; v < hrows; v++)
   {
      float * ptr_ds0 = ds[2*v  ];
      float * ptr_ds1 = ds[2*v+1];

      for (int u = 0; u < hcols; u+=4)
      {
         // Could be changed to _mm256_load_ps if we force 32 bits memory allocaltion alignment
         __m256 avx_ds0 = _mm256_loadu_ps ( ptr_ds0 );
         __m256 avx_ds1 = _mm256_loadu_ps ( ptr_ds1 );

         // Add two rows
         __m256 add_rows = _mm256_add_ps ( avx_ds0, avx_ds1 );

         // Add two cols
         __m256 add_cols = _mm256_hadd_ps ( add_rows, add_rows );

         // Divide by 4.0
         __m256 mean = _mm256_div_ps ( add_cols, avx_4 );

         // Get the result
         _mm256_storeu_ps ( buffer_8_bytes.tab, mean );

         // Store the result
         // dd[v][u] = (ds[dv][du] + ds[dv + 1][du] + ds[dv][du + 1] + ds[dv + 1][du + 1]) / 4.0f;

         dd[v][u  ] = buffer_8_bytes.tab[0];
         dd[v][u+1] = buffer_8_bytes.tab[1];
         dd[v][u+2] = buffer_8_bytes.tab[4];
         dd[v][u+3] = buffer_8_bytes.tab[5];

         // Increment pointers
         ptr_ds0 += 8;
         ptr_ds1 += 8;
      }
   }

   return error;
}

int rox_ansi_array2d_float_remap_halved_box_optimal_data_store_avx (
   float ** dd, 
   int hrows, // rows of the halved image
   int hcols, // cols of the halved image
   float ** ds
)
{
   int error = 0;

   union avx_vector buffer;

   __m256 avx_4 = _mm256_set1_ps(4);

   for ( int v = 0; v < hrows; v++ )
   {
      float * ptr_ds0 = ds[2*v  ];
      float * ptr_ds1 = ds[2*v+1];

      for ( int u = 0; u < hcols; u+=8 )
      {
         // Load 8 float of the two rows 
         __m256 avx_ds0 = _mm256_loadu_ps(ptr_ds0);
         __m256 avx_ds1 = _mm256_loadu_ps(ptr_ds1);
         
         // Increment pointers
         ptr_ds0 += 8;
         ptr_ds1 += 8;

         // Add two rows
         __m256 add_rows01 = _mm256_add_ps(avx_ds0, avx_ds1);

         // Add two cols
         __m256 add_cols01 = _mm256_hadd_ps(add_rows01, add_rows01);

         // Load 8 float of the two rows 
         __m256 avx_ds2 = _mm256_loadu_ps ( ptr_ds0 );   
         __m256 avx_ds3 = _mm256_loadu_ps ( ptr_ds1 );
         
         // Increment pointers
         ptr_ds0 += 8;
         ptr_ds1 += 8;

         // Add two rows
         __m256 add_rows23 = _mm256_add_ps(avx_ds2, avx_ds3);

         // Add two cols
         __m256 add_cols23 = _mm256_hadd_ps(add_rows23, add_rows23);

         // Blend results
         __m256 mask = _mm256_set_ps ( ~0, ~0, 0, 0, ~0, ~0, 0, 0 );
         __m256 add_cols = _mm256_blendv_ps (add_cols01, add_cols23, mask);

         // Divide by 4.0
         __m256 mean = _mm256_div_ps ( add_cols, avx_4 );

         // Get the result
         _mm256_storeu_ps ( buffer.tab, mean );

         // Store the result in the correct order
         dd[v][u  ] = buffer.tab[0];
         dd[v][u+1] = buffer.tab[1];
         dd[v][u+2] = buffer.tab[4];
         dd[v][u+3] = buffer.tab[5];
         dd[v][u+4] = buffer.tab[2];
         dd[v][u+5] = buffer.tab[3];
         dd[v][u+6] = buffer.tab[6];
         dd[v][u+7] = buffer.tab[7];
      }
   }

   return error;
}

int rox_ansi_remap_box_nomask_float_to_float_halved (
   float ** dd,
   float ** ds,
   int hrows, // rows of the halved image
   int hcols  // cols of the halved image
)
{
   return rox_ansi_array2d_float_remap_halved_box_optimal_data_store_avx ( dd, hrows, hcols, ds );
}

#ifdef old
Rox_ErrorCode rox_remap_box_nomask_float_to_float_halved (
   Rox_Array2D_Float dest, 
   const Rox_Array2D_Float source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_float_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_float_check_size(dest, hrows, hcols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_remap_halved_box ( dd, hrows, hcols, ds );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
#endif