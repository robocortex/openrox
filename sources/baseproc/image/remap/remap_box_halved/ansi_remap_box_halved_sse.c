//============================================================================
//
//    OPENROX   : File remap_box_halved_sse.c
//
//    Contents  : Implementation of remap_box_halved module with SSE optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "remap_box_halved.h"
#include <inout/system/errors_print.h>
#include <system/vectorisation/sse.h>

// TO BE IMPLEMENTED
#ifdef new

int rox_ansi_remap_box_nomask_uchar_to_uchar_halved (
   unsigned char ** dd,
   unsigned char ** ds,
   int hrows,
   int hcols
)
{
   int error = 0;

   unsigned short buffer1[8];
   unsigned short buffer2[8];

   __m128i add_rows;
   __m128i add_cols;
   __m128i mean;
   __m128i sse_ds, sse_dsn;

   for (int v = 0; v < hrows; v++)
   {
      unsigned char *ptr_ds0 = ds[2*v];
      unsigned char *ptr_ds1 = ds[2*v+1];

      for (int u = 0; u < hcols; u+=4)
      {
         for (int k=0; k<8; k++) buffer1[k] = (unsigned short) ptr_ds0[k];
         for (int k=0; k<8; k++) buffer2[k] = (unsigned short) ptr_ds1[k];

         __m128i sse_ds0 = _mm_loadu_si128(( __m128i* ) buffer1);   //
         __m128i sse_ds1 = _mm_loadu_si128(( __m128i* ) buffer2);

         // Add two rows
         add_rows = _mm_add_epi16(sse_ds0, sse_ds1);

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

         ptr_ds0 += 8;
         ptr_ds1 += 8;
      }
   }

function_terminate:
   return error;
}

#else

int rox_ansi_remap_box_nomask_uchar_to_uchar_halved (
   unsigned char ** dd,
   unsigned char ** ds,
   int hrows,
   int hcols
)
{
   int error = 0;
   for ( int i = 0; i < hrows; i++ )
   {
      int di = i * 2;
      for ( int j = 0; j < hcols; j++ )
      {
         int dj = j * 2;
         unsigned short buffer = (unsigned short) ds[di][dj];
         // Compute average gray level over the 4 neirest neighboor pixels
         buffer += (unsigned short) ds[di][dj + 1];
         buffer += (unsigned short) ds[di + 1][dj];
         buffer += (unsigned short) ds[di + 1][dj + 1];
         buffer = buffer / 4;
         dd[i][j] = (unsigned char) buffer;
      }
   }
   return error;
}

#endif

int rox_sse_array2d_float_remap_halved_box_suboptimal_data_store (
// int rox_sse_array2d_float_remap_halved_box (
   float ** dd,
   float ** ds,
   int hrows,
   int hcols
)
{
   int error = 0;   
   union ssevector buffer_4_bytes;

   __m128 sse_4 = _mm_set_ps1(4);

   for ( int v = 0; v < hrows; v++ )
   {
      Rox_Float * ptr_ds0 = ds[2*v  ];
      Rox_Float * ptr_ds1 = ds[2*v+1];

      for ( int u = 0; u < hcols; u+=2 )
      {
         __m128 sse_ds0 = _mm_loadu_ps ( ptr_ds0 );
         __m128 sse_ds1 = _mm_loadu_ps ( ptr_ds1 );

         // Add two rows
         __m128 add_rows = _mm_add_ps ( sse_ds0, sse_ds1 );

         // Add two cols
         __m128 add_cols = _mm_hadd_ps ( add_rows, add_rows );

         // Divide by 4.0
         __m128 mean = _mm_div_ps ( add_cols, sse_4 );

         // Get the result
         _mm_storeu_ps ( buffer_4_bytes.tab, mean );

         // Store the result
         //dd[v][u] = (ds[dv][du] + ds[dv + 1][du] + ds[dv][du + 1] + ds[dv + 1][du + 1]) / 4.0f;

         dd[v][u  ] = buffer_4_bytes.tab[0];
         dd[v][u+1] = buffer_4_bytes.tab[1];

         // Increment pointers
         ptr_ds0 += 4;
         ptr_ds1 += 4;
      }
   }
   return error;
}

int rox_sse_array2d_float_remap_halved_box_optimal_data_store (
//int rox_ansi_array2d_float_remap_halved_box_sse (
   float ** dd,
   float ** ds,
   int hrows,
   int hcols
)
{
   int error = 0;

   union ssevector buffer;

   __m128 sse_4 = _mm_set_ps1(4);

   for ( int v = 0; v < hrows; v++ )
   {
      Rox_Float * ptr_ds0 = ds[2*v  ];
      Rox_Float * ptr_ds1 = ds[2*v+1];

      for ( int u = 0; u < hcols; u+=4 )
      {
         // Load 4 float of the two rows
         __m128 sse_ds0 = _mm_loadu_ps ( ptr_ds0 );
         __m128 sse_ds1 = _mm_loadu_ps ( ptr_ds1 );

         // Increment pointers
         ptr_ds0 += 4;
         ptr_ds1 += 4;

         // Add two rows
         __m128 add_rows01 = _mm_add_ps ( sse_ds0, sse_ds1 );

         // Add two cols
         __m128 add_cols01 = _mm_hadd_ps ( add_rows01, add_rows01 );

         // Load 4 float of the two rows
         __m128 sse_ds2 = _mm_loadu_ps ( ptr_ds0 );
         __m128 sse_ds3 = _mm_loadu_ps ( ptr_ds1 );

         // Increment pointers
         ptr_ds0 += 4;
         ptr_ds1 += 4;

         // Add two rows
         __m128 add_rows23 = _mm_add_ps ( sse_ds2, sse_ds3 );

         // Add two cols
         __m128 add_cols23 = _mm_hadd_ps ( add_rows23, add_rows23 );

         // Move the lower two bytes of add_cols23 to the upper two bytes of the result.
         // The lower two bytes of add_cols01 are passed through to the result.
         // NB: observed that if we inverse the order of add_cols01 nad add_cols23 it takes longer
         __m128 add_cols = _mm_movehl_ps ( add_cols01, add_cols23 );

         // Divide by 4.0
         __m128 mean = _mm_div_ps ( add_cols, sse_4 );

         // Get the result
         _mm_storeu_ps ( buffer.tab, mean );

         // Store the result in the correct order
         dd[v][u  ] = buffer.tab[2];
         dd[v][u+1] = buffer.tab[3];
         dd[v][u+2] = buffer.tab[0];
         dd[v][u+3] = buffer.tab[1];
      }
   }

   return error;
}

int rox_ansi_remap_box_nomask_float_to_float_halved (
   float ** dd,
   float ** ds,
   int hrows,
   int hcols
)
{
   // return rox_sse_array2d_float_remap_halved_box_suboptimal_data_store ( dd, ds, hrows, hcols );
      return rox_sse_array2d_float_remap_halved_box_optimal_data_store (dd, ds, hrows, hcols);
}