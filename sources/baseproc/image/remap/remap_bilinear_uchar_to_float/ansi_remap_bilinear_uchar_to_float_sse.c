//==============================================================================
//
//    OPENROX   : File remap_bilinear_uchar_to_float_sse.c
//
//    Contents  : Implementation of remap_bilinear_uchar_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_remap_bilinear_uchar_to_float.h"

//#include <baseproc/geometry/point/point2d.h>
//#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

//#include <inout/system/errors_print.h>
#include <system/vectorisation/sse.h>

int rox_ansi_remap_bilinear_uchar_to_float (
   float ** image_out_data,
   unsigned int ** imask_out_data,
   unsigned int ** imask_out_ini_data,
   int rows_out,
   int cols_out,
   unsigned char ** image_inp_data,
   unsigned int ** inp_mask_data,
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
)
{
   int error = 0;

   int cols4 = cols_out / 4;
   if (cols_out % 4) cols4++;

   union ssevector maskaccess, puaccess, pvaccess;
   union ssevector val1, val2, val3, val4;

   __m128 sse_zero = _mm_set_ps1(0);
   __m128 sse_4 = _mm_set_ps1(4);
   __m128 sse_cols = _mm_set_ps1((float) cols_out);
   __m128 sse_icols = _mm_set_ps1((float) (cols_inp - 1));
   __m128 sse_irows = _mm_set_ps1((float) (rows_inp - 1));

   for (int v = 0; v < rows_out; v++)
   {
      Rox_Float * ptr_out = image_out_data[v];

      Rox_Uint * ptr_imask_out = imask_out_data[v];
      Rox_Uint * ptr_imask_out_ini = imask_out_ini_data[v];

      Rox_Float * ptr_u = grid_u_data[v];
      Rox_Float * ptr_v = grid_v_data[v];

      __m128 sse_u = _mm_set_ps(3,2,1,0);

      for (int u = 0; u < cols4; u++)
      {
         __m128 mask = _mm_load_ps((float*) ptr_imask_out_ini);
         mask = _mm_and_ps(mask, _mm_cmplt_ps(sse_u, sse_cols));

         int gmask = _mm_movemask_ps(mask);
         _mm_store_ps(ptr_out, sse_zero);

         if (gmask)
         {
            __m128 sse_uuuu = _mm_load_ps( ptr_u );
            __m128 sse_vvvv = _mm_load_ps( ptr_v );

            mask = _mm_and_ps(mask, _mm_cmpge_ps(sse_uuuu, sse_zero));
            mask = _mm_and_ps(mask, _mm_cmpge_ps(sse_vvvv, sse_zero));
            mask = _mm_and_ps(mask, _mm_cmplt_ps(sse_uuuu, sse_icols));
            mask = _mm_and_ps(mask, _mm_cmplt_ps(sse_vvvv, sse_irows));
            gmask = _mm_movemask_ps(mask);

            if (gmask)
            {
               sse_uuuu = _mm_and_ps(mask, sse_uuuu);
               sse_vvvv = _mm_and_ps(mask, sse_vvvv);

               __m128i sse_iu = _mm_cvttps_epi32(sse_uuuu);
               __m128i sse_iv = _mm_cvttps_epi32(sse_vvvv);

               puaccess.sse = sse_uuuu;
               pvaccess.sse = sse_vvvv;
               maskaccess.sse = mask;

               for (int k = 0; k < 4; k++)
               {
                  int pu = (int) puaccess.tab[k];
                  int pv = (int) pvaccess.tab[k];

                  Rox_Uint m = inp_mask_data[pv][pu];
                  // m &= inp_mask_data[pv][pu+1];
                  // m &= inp_mask_data[pv+1][pu];
                  // m &= inp_mask_data[pv+1][pu+1];
                  if (!m) maskaccess.tab[k] = 0;

                  val1.tab[k] = (float) image_inp_data[pv][pu];
                  val2.tab[k] = (float) image_inp_data[pv][pu+1];
                  val3.tab[k] = (float) image_inp_data[pv+1][pu];
                  val4.tab[k] = (float) image_inp_data[pv+1][pu+1];
               }

               mask = maskaccess.sse;
               gmask = _mm_movemask_ps(mask);

               if (gmask)
               {
                  __m128 sse_du = _mm_sub_ps(sse_uuuu, _mm_cvtepi32_ps(sse_iu));
                  __m128 sse_dv = _mm_sub_ps(sse_vvvv, _mm_cvtepi32_ps(sse_iv));

                  __m128 sse_b1 = val1.sse;
                  __m128 sse_b2 = _mm_sub_ps(val2.sse, sse_b1);
                  __m128 sse_b3 = _mm_sub_ps(val3.sse, sse_b1);
                  __m128 sse_b4 = _mm_add_ps(sse_b1, _mm_sub_ps(val4.sse, _mm_add_ps(val3.sse, val2.sse)));

                  sse_b2 = _mm_mul_ps(sse_b2, sse_du);
                  sse_b3 = _mm_mul_ps(sse_b3, sse_dv);
                  sse_b4 = _mm_mul_ps(sse_b4, _mm_mul_ps(sse_du, sse_dv));

                  __m128 sse_res = _mm_add_ps(sse_b1, sse_b2);
                  sse_res = _mm_add_ps(sse_res, sse_b3);
                  sse_res = _mm_add_ps(sse_res, sse_b4);

                  _mm_store_ps(ptr_out, sse_res);
               }
            }
         }

         _mm_store_ps((float*)ptr_imask_out, mask);

         ptr_u += 4;
         ptr_v += 4;

         ptr_out += 4;
         ptr_imask_out_ini += 4;
         ptr_imask_out += 4;

         sse_u = _mm_add_ps(sse_u, sse_4);
      }
   }

   return error;
}

#ifdef old

Rox_ErrorCode rox_remap_bilinear_uchar_to_float (
   Rox_Array2D_Float image_out,
   Rox_Imask imask_out,
   const Rox_Imask imask_out_ini,
   const Rox_Image image_inp,
   const Rox_Imask imask_inp,
   const Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !image_out || !imask_out_ini || !imask_out )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !imask_inp || !image_inp || !grid )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   int cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, image_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   int cols_inp = 0, rows_inp = 0;
   error = rox_image_get_size ( &rows_inp, &cols_inp, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( imask_inp, rows_inp, cols_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( imask_out, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( imask_out_ini, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   int grid_cols = 0, grid_rows = 0;
   error = rox_meshgrid2d_float_get_size ( &grid_rows, &grid_cols, grid );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (grid_cols != cols)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (grid_rows != rows)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float ** image_out_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&image_out_data, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** inp_mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&inp_mask_data, imask_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** imask_out_ini_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&imask_out_ini_data, imask_out_ini);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** imask_out_data = NULL;

   error = rox_array2d_uint_get_data_pointer_to_pointer(&imask_out_data, imask_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** image_inp_data = NULL;
   error = rox_image_get_data_pointer_to_pointer ( &image_inp_data, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_sse_remap_bilinear_uchar_to_float ( image_out_data, imask_out_data, imask_out_ini_data, rows, cols, image_inp_data, inp_mask_data, rows_inp, cols_inp, grid_u_data, grid_v_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
#endif