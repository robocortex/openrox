//==============================================================================
//
//    OPENROX   : File remap_bilinear_omo_float_to_float_neon.c
//
//    Contents  : Implementation of remapbilinear_omo module with NEON optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_omo_float_to_float.h"
#include "ansi_remap_bilinear_omo_float_to_float.h"

#define integrated
#ifdef integrated
   #include <generated/array2d_float.h>

   #include <baseproc/geometry/point/point2d.h>
   #include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

   #include <inout/system/errors_print.h>
#endif


int rox_ansi_remap_bilinear_omo_float_to_float (
   float ** image_out_data,
   unsigned int ** imask_out_data,
   int rows_out,
   int cols_out,
   float ** image_inp_data,
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
)
{
   int error = 0;

   int cols4 = cols_out / 4;
   if (cols_out % 4) cols4++;

   float32x4_t ssezero = vdupq_n_f32(0);
   float32x4_t sse4 = vdupq_n_f32(4);
   float32x4_t ssecols = vdupq_n_f32(cols_out);
   float32x4_t sseicols = vdupq_n_f32(cols_inp - 1);
   float32x4_t sseirows = vdupq_n_f32(rows_inp - 1);

   Rox_Neon_Uint maskaccess;
   Rox_Neon_Float puaccess, pvaccess;
   Rox_Neon_Float val1, val2, val3, val4, ussej;

   ussej.tab[0] = 0;
   ussej.tab[1] = 1;
   ussej.tab[2] = 2;
   ussej.tab[3] = 3;

   for (int i = 0; i < rows_out; i++)
   {
      float * ptrout = image_out_data[i];
      unsigned int * ptrout_masko = imask_out_data[i];
            
      float * ptr_u = grid_u_data[i];
      float * ptr_v = grid_v_data[i];

      float32x4_t ssej = ussej.ssetype;

      for (Rox_Uint j = 0; j < cols4; j++)
      {
         uint32x4_t ssemask = vcltq_f32(ssej, ssecols);
         vst1q_f32(ptrout, ssezero);

         float32x4_t sseuuuu = vld1q_f32( (float*) ptr_u );
         float32x4_t ssevvvv = vld1q_f32( (float*) ptr_v );

         ssemask = vandq_u32(ssemask, vcgeq_f32(sseuuuu, ssezero));
         ssemask = vandq_u32(ssemask, vcgeq_f32(ssevvvv, ssezero));
         ssemask = vandq_u32(ssemask, vcltq_f32(sseuuuu, sseicols));
         ssemask = vandq_u32(ssemask, vcltq_f32(ssevvvv, sseirows));

         sseuuuu = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(sseuuuu)));
         ssevvvv = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(ssevvvv)));

         int32x4_t sseiu = vcvtq_s32_f32(sseuuuu);
         int32x4_t sseiv = vcvtq_s32_f32(ssevvvv);

         puaccess.ssetype = sseuuuu;
         pvaccess.ssetype = ssevvvv;
         maskaccess.ssetype = ssemask;

         for (Rox_Uint k = 0; k < 4; k++)
         {
            Rox_Sint pu = puaccess.tab[k];
            Rox_Sint pv = pvaccess.tab[k];

            val1.tab[k] = image_inp_data[pv][pu];
            val2.tab[k] = image_inp_data[pv][pu+1];
            val3.tab[k] = image_inp_data[pv+1][pu];
            val4.tab[k] = image_inp_data[pv+1][pu+1];
         }

         float32x4_t ssedu = vsubq_f32(sseuuuu, vcvtq_f32_s32(sseiu));
         float32x4_t ssedv = vsubq_f32(ssevvvv, vcvtq_f32_s32(sseiv));

         float32x4_t sseb1 = val1.ssetype;
         float32x4_t sseb2 = vsubq_f32(val2.ssetype, sseb1);
         float32x4_t sseb3 = vsubq_f32(val3.ssetype, sseb1);
         float32x4_t sseb4 = vaddq_f32(sseb1, vsubq_f32(val4.ssetype, vaddq_f32(val3.ssetype, val2.ssetype)));

         sseb2 = vmulq_f32(sseb2, ssedu);
         sseb3 = vmulq_f32(sseb3, ssedv);
         sseb4 = vmulq_f32(sseb4, vmulq_f32(ssedu, ssedv));

         float32x4_t sseres = vaddq_f32(sseb1, sseb2);
         sseres = vaddq_f32(sseres, sseb3);
         sseres = vaddq_f32(sseres, sseb4);

         vst1q_f32(ptrout, sseres);
         vst1q_u32((Rox_Uint*)ptrout_masko, ssemask);

         ptrout+=4;
         ptrout_masko+=4;
                  
         ptr_u +=4;
         ptr_v +=4;
         
         ssej = vaddq_f32(ssej, sse4);
      }
   }

   return error;
}


Rox_ErrorCode rox_remap_bilinear_omo_float_to_float_old_neon (
   Rox_Image_Float image_out, 
   Rox_Imask imask_out, 
   const Rox_Image_Float image_inp, 
   const Rox_MeshGrid2D_Float grid 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !image_out || !imask_out )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image_inp || !grid ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0 ;
   error = rox_array2d_float_get_size ( &rows, &cols, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_inp, rows_inp;
   error = rox_array2d_float_get_size ( &rows_inp, &cols_inp, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( imask_out, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint rows_grid = 0, cols_grid = 0 ;
   error = rox_meshgrid2d_float_get_size ( &rows_grid, &cols_grid, grid);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( cols_grid != cols )  
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( rows_grid != rows ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** out_masko = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &out_masko, imask_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** out_data = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &out_data, image_out );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** inp_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &inp_data, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef integrated
   Rox_Sint cols4 = cols / 4;
   if (cols % 4) cols4++;

   float32x4_t ssezero = vdupq_n_f32(0);
   float32x4_t sse4 = vdupq_n_f32(4);
   float32x4_t ssecols = vdupq_n_f32(cols);
   float32x4_t sseicols = vdupq_n_f32(cols_inp - 1);
   float32x4_t sseirows = vdupq_n_f32(rows_inp - 1);

   Rox_Neon_Uint maskaccess;
   Rox_Neon_Float puaccess, pvaccess;
   Rox_Neon_Float val1, val2, val3, val4, ussej;

   ussej.tab[0] = 0;
   ussej.tab[1] = 1;
   ussej.tab[2] = 2;
   ussej.tab[3] = 3;

   for (Rox_Uint i = 0; i < rows; i++)
   {
      Rox_Float * ptrout = out_data[i];
      Rox_Uint * ptrout_masko = out_masko[i];
            
      Rox_Float * ptr_u = grid_u_data[i];
      Rox_Float * ptr_v = grid_v_data[i];

      float32x4_t ssej = ussej.ssetype;

      for (Rox_Uint j = 0; j < cols4; j++)
      {
         uint32x4_t ssemask = vcltq_f32(ssej, ssecols);
         vst1q_f32(ptrout, ssezero);

         float32x4_t sseuuuu = vld1q_f32( (float*) ptr_u );
         float32x4_t ssevvvv = vld1q_f32( (float*) ptr_v );

         ssemask = vandq_u32(ssemask, vcgeq_f32(sseuuuu, ssezero));
         ssemask = vandq_u32(ssemask, vcgeq_f32(ssevvvv, ssezero));
         ssemask = vandq_u32(ssemask, vcltq_f32(sseuuuu, sseicols));
         ssemask = vandq_u32(ssemask, vcltq_f32(ssevvvv, sseirows));

         sseuuuu = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(sseuuuu)));
         ssevvvv = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(ssevvvv)));

         int32x4_t sseiu = vcvtq_s32_f32(sseuuuu);
         int32x4_t sseiv = vcvtq_s32_f32(ssevvvv);

         puaccess.ssetype = sseuuuu;
         pvaccess.ssetype = ssevvvv;
         maskaccess.ssetype = ssemask;

         for (Rox_Uint k = 0; k < 4; k++)
         {
            Rox_Sint pu = puaccess.tab[k];
            Rox_Sint pv = pvaccess.tab[k];

            val1.tab[k] = inp_data[pv][pu];
            val2.tab[k] = inp_data[pv][pu+1];
            val3.tab[k] = inp_data[pv+1][pu];
            val4.tab[k] = inp_data[pv+1][pu+1];
         }

         float32x4_t ssedu = vsubq_f32(sseuuuu, vcvtq_f32_s32(sseiu));
         float32x4_t ssedv = vsubq_f32(ssevvvv, vcvtq_f32_s32(sseiv));

         float32x4_t sseb1 = val1.ssetype;
         float32x4_t sseb2 = vsubq_f32(val2.ssetype, sseb1);
         float32x4_t sseb3 = vsubq_f32(val3.ssetype, sseb1);
         float32x4_t sseb4 = vaddq_f32(sseb1, vsubq_f32(val4.ssetype, vaddq_f32(val3.ssetype, val2.ssetype)));

         sseb2 = vmulq_f32(sseb2, ssedu);
         sseb3 = vmulq_f32(sseb3, ssedv);
         sseb4 = vmulq_f32(sseb4, vmulq_f32(ssedu, ssedv));

         float32x4_t sseres = vaddq_f32(sseb1, sseb2);
         sseres = vaddq_f32(sseres, sseb3);
         sseres = vaddq_f32(sseres, sseb4);

         vst1q_f32(ptrout, sseres);
         vst1q_u32((Rox_Uint*)ptrout_masko, ssemask);

         ptrout+=4;
         ptrout_masko+=4;
                  
         ptr_u +=4;
         ptr_v +=4;
         
         ssej = vaddq_f32(ssej, sse4);
      }
   }

#else

#endif

function_terminate:
   return error;
}
