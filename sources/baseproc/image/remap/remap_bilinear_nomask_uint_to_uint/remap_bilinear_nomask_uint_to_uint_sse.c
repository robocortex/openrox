//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_uint_to_uint_sse.c
//
//    Contents  : Implementation of remap_bilinear_nomask_uint_to_uint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_nomask_uint_to_uint.h"
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_bilinear_nomask_uint (
   Rox_Array2D_Uint image_out, 
   Rox_Array2D_Uint image_inp, 
   Rox_MeshGrid2D_Float grid
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Sint cols, rows;
    Rox_Sint incols, inrows;
    Rox_Sint cx, cy;
    Rox_Double ix, iy;
    Rox_Double dx, dy;

    Rox_Double b1, b2, b3, b4;
    
    if (!image_out || !image_inp || !grid) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
    error = rox_array2d_uint_get_size(&rows, &cols, image_out);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_uint_get_size(&inrows, &incols, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );
  
    error = rox_meshgrid2d_float_check_size(grid, rows, cols); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    Rox_Uint ** image_out_data = NULL;
    error = rox_array2d_uint_get_data_pointer_to_pointer ( &image_out_data, image_out);
    ROX_ERROR_CHECK_TERMINATE ( error );

    Rox_Uint ** image_inp_data = NULL;
    error = rox_array2d_uint_get_data_pointer_to_pointer ( &image_inp_data, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );

    Rox_Float ** grid_u_data = NULL;
    error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    Rox_Float ** grid_v_data = NULL;
    error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
    ROX_ERROR_CHECK_TERMINATE ( error );

    for (Rox_Sint i = 0; i < rows; i++)
    {
        for (Rox_Sint j = 0; j < cols; j++)
        {
            image_out_data[i][j] = 0;

            cx = (Rox_Sint) grid_u_data[i][j];
            cy = (Rox_Sint) grid_v_data[i][j];

            ix = cx;
            iy = cy;

            if (cx < 0 || cy < 0) continue;
            if (cx >= incols - 1 || cy >= inrows - 1) continue;

            dx = grid_u_data[i][j] - ix;
            dy = grid_v_data[i][j] - iy;

            b1 = (Rox_Double)image_inp_data[cy][cx];
            b2 = (Rox_Double)image_inp_data[cy][cx+1] - b1;
            b3 = (Rox_Double)image_inp_data[cy+1][cx] - b1;
            b4 = b1 + (Rox_Double)image_inp_data[cy+1][cx+1] - (Rox_Double)image_inp_data[cy+1][cx] - (Rox_Double)image_inp_data[cy][cx+1];

            image_out_data[i][j] = (Rox_Uint) (b1 + b2*dx + b3*dy + b4*dx*dy);
        }
    }

function_terminate:
   return error;
}

Rox_ErrorCode rox_remap_bilinear_nomask_rgba (
   Rox_Array2D_Uint image_out, 
   Rox_Array2D_Uint image_inp, 
   const Rox_MeshGrid2D_Float grid
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Sint cols, rows;
    Rox_Sint incols, inrows;
    Rox_Uint ** image_out_data, **image_inp_data;
    Rox_Sint cx, cy;
    Rox_Double ix, iy;
    Rox_Double dx, dy;

    Rox_Double b1, b2, b3, b4;
    Rox_Double r1, r2, r3, r4;
    Rox_Double g1, g2, g3, g4;
    Rox_Uint r, g, b;
    
    if (!image_out || !image_inp || !grid) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
    
    error = rox_array2d_uint_get_size(&rows, &cols, image_out);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_uint_get_size(&inrows, &incols, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_meshgrid2d_float_check_size (grid, rows, cols); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_uint_get_data_pointer_to_pointer ( &image_out_data, image_out);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_uint_get_data_pointer_to_pointer ( &image_inp_data, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    Rox_Float ** grid_u_data = NULL;
    error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    Rox_Float ** grid_v_data = NULL;
    error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
    ROX_ERROR_CHECK_TERMINATE ( error );

    for (Rox_Sint i = 0; i < rows; i++)
    {
        for (Rox_Sint j = 0; j < cols; j++)
        {
            image_out_data[i][j] = ~0;

            cx = (Rox_Sint) grid_u_data[i][j];
            cy = (Rox_Sint) grid_v_data[i][j];

            ix = cx;
            iy = cy;

            if (cx < 0 || cy < 0) continue;
            if (cx >= incols - 1 || cy >= inrows - 1) continue;

            dx = grid_u_data[i][j] - ix;
            dy = grid_v_data[i][j] - iy;

            // red
            r1 = (Rox_Double)(image_inp_data[cy][cx] & 0x000000ff);
            r2 = (Rox_Double)(image_inp_data[cy][cx+1] & 0x000000ff) - r1;
            r3 = (Rox_Double)(image_inp_data[cy+1][cx] & 0x000000ff) - r1;
            r4 = r1 + (Rox_Double)(image_inp_data[cy+1][cx+1] & 0x000000ff) - (Rox_Double)(image_inp_data[cy+1][cx] & 0x000000ff) - (Rox_Double)(image_inp_data[cy][cx+1] & 0x000000ff);

            r = (Rox_Uint)(r1 + r2*dx + r3*dy + r4*dx*dy);

            // green
            g1 = (Rox_Double)(image_inp_data[cy][cx] >> 8 & 0x000000ff);
            g2 = (Rox_Double)(image_inp_data[cy][cx+1] >> 8 & 0x000000ff) - g1;
            g3 = (Rox_Double)(image_inp_data[cy+1][cx] >> 8 & 0x000000ff) - g1;
            g4 = g1 + (Rox_Double)(image_inp_data[cy+1][cx+1] >> 8 & 0x000000ff) - (Rox_Double)(image_inp_data[cy+1][cx] >> 8 & 0x000000ff) - (Rox_Double)(image_inp_data[cy][cx+1] >> 8 & 0x000000ff);

            g = (Rox_Uint)(g1 + g2*dx + g3*dy + g4*dx*dy);

            // blue
            b1 = (Rox_Double)(image_inp_data[cy][cx] >> 16 & 0x000000ff);
            b2 = (Rox_Double)(image_inp_data[cy][cx+1] >> 16 & 0x000000ff) - b1;
            b3 = (Rox_Double)(image_inp_data[cy+1][cx] >> 16 & 0x000000ff) - b1;
            b4 = b1 + (Rox_Double)(image_inp_data[cy+1][cx+1] >> 16 & 0x000000ff) - (Rox_Double)(image_inp_data[cy+1][cx] >> 16 & 0x000000ff) - (Rox_Double)(image_inp_data[cy][cx+1] >> 16 & 0x000000ff);

            b = (Rox_Uint)(b1 + b2*dx + b3*dy + b4*dx*dy);

            // build output rgba
            image_out_data[i][j] = ((255 << 24) + (b << 16) + (g << 8) + r);
        }
    }

function_terminate:
   return error;
}

int rox_ansi_remap_bilinear_nomask_rgba_fixed (
   unsigned int ** image_out_data,
   unsigned int ** image_inp_data, 
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
)
{
   int error = 0;
   // TODO
   return error;
}

Rox_ErrorCode rox_remap_bilinear_nomask_rgba_fixed (
    Rox_Array2D_Uint image_out, 
    Rox_Array2D_Uint image_inp, 
    Rox_Array2D_Point2D_Sshort grid
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Sint cols, rows;
    Rox_Uint ** image_out_data, **image_inp_data, *ptrout;
    Rox_Point2D_Sshort ptrxy;
    Rox_Sint cx, cy;
    Rox_Sint cols4;

    __m128i ssexy, sse15, sse255, sse0, ssev4u4v3u3, sseu2u2v1u1, ssev4v2u4u2, ssev3v1u3u1, ssev4v3v2v1, sseu4u3u2u1;
    __m128i ssecu, ssecv, ssedu, ssedv, sseb1, sseb2, sseb3, sseb4;
    __m128i sseval, ssev1, ssev2, ssev3, ssev4;

    __m128i ssergba1, ssergba2, ssergba3, ssergba4;
    __m128i ssered1, ssered2, ssered3, ssered4;
    __m128i ssegreen1, ssegreen2, ssegreen3, ssegreen4;
    __m128i sseblue1, sseblue2, sseblue3, sseblue4;

    __m128i sse_red, sse_blue, sse_green, sse_alpha, sse_rgba;

    Rox_Sint vcu[4], vcv[4];
    Rox_Sint rgba1[4], rgba2[4], rgba3[4], rgba4[4];
    
    if (!image_out || !image_inp || !grid) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_array2d_uint_get_size(&rows, &cols, image_out);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_point2d_sshort_check_size(grid, rows, cols); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    cols4 = cols / 4;
    if (cols % 4) cols4++;

    error = rox_array2d_uint_get_data_pointer_to_pointer (&image_out_data, image_out);
    ROX_ERROR_CHECK_TERMINATE ( error );
  
    error = rox_array2d_uint_get_data_pointer_to_pointer (&image_inp_data, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );

    Rox_Point2D_Sshort * mxy = NULL;
    error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &mxy, grid);
    ROX_ERROR_CHECK_TERMINATE ( error );

    sse0 =  _mm_set1_epi16(0);
    sse15 =  _mm_set1_epi32(15);
    sse255 =  _mm_set1_epi32(255);
    sse_alpha = _mm_set1_epi32(255 << 24);

    for (Rox_Sint i = 0; i < rows; i++)
    {
        ptrxy = mxy[i];
        ptrout = image_out_data[i];

        for (Rox_Sint j = 0; j < cols4; j++)
        {

            ssexy = _mm_stream_load_si128((__m128i*) ptrxy);

            ssev4u4v3u3 = _mm_unpackhi_epi16(ssexy, sse0);
            sseu2u2v1u1 = _mm_unpacklo_epi16(ssexy, sse0);
            
            ssev4v2u4u2 = _mm_unpackhi_epi32(ssev4u4v3u3, sseu2u2v1u1);
            ssev3v1u3u1 = _mm_unpacklo_epi32(ssev4u4v3u3, sseu2u2v1u1);
            ssev4v3v2v1 = _mm_unpackhi_epi32(ssev4v2u4u2, ssev3v1u3u1);
            sseu4u3u2u1 = _mm_unpacklo_epi32(ssev4v2u4u2, ssev3v1u3u1);

            ssecu = _mm_srli_epi32(sseu4u3u2u1, 4);
            ssecv = _mm_srli_epi32(ssev4v3v2v1, 4);
            ssedu = _mm_and_si128(sseu4u3u2u1, sse15);
            ssedv = _mm_and_si128(ssev4v3v2v1, sse15);

            _mm_store_si128((__m128i*) vcu, ssecu);
            _mm_store_si128((__m128i*) vcv, ssecv);

            for (Rox_Sint k = 0; k < 4; k++)
            {
                cx = vcu[k];
                cy = vcv[k];

                rgba1[k] = image_inp_data[cy][cx];
                rgba2[k] = image_inp_data[cy][cx + 1];
                rgba3[k] = image_inp_data[cy + 1][cx];
                rgba4[k] = image_inp_data[cy + 1][cx + 1];
            }

            // Set SSE objects
            ssergba1 = _mm_load_si128((__m128i*)rgba1);
            ssergba2 = _mm_load_si128((__m128i*)rgba2);
            ssergba3 = _mm_load_si128((__m128i*)rgba3);
            ssergba4 = _mm_load_si128((__m128i*)rgba4);

            // Extrat r g b
            ssered1 = _mm_and_si128(ssergba1, sse255);
            ssered2 = _mm_and_si128(ssergba2, sse255);
            ssered3 = _mm_and_si128(ssergba3, sse255);
            ssered4 = _mm_and_si128(ssergba4, sse255);

            ssegreen1 = _mm_and_si128(_mm_srli_epi32(ssergba1, 8), sse255);
            ssegreen2 = _mm_and_si128(_mm_srli_epi32(ssergba2, 8), sse255);
            ssegreen3 = _mm_and_si128(_mm_srli_epi32(ssergba3, 8), sse255);
            ssegreen4 = _mm_and_si128(_mm_srli_epi32(ssergba4, 8), sse255);

            sseblue1 = _mm_and_si128(_mm_srli_epi32(ssergba1, 16), sse255);
            sseblue2 = _mm_and_si128(_mm_srli_epi32(ssergba2, 16), sse255);
            sseblue3 = _mm_and_si128(_mm_srli_epi32(ssergba3, 16), sse255);
            sseblue4 = _mm_and_si128(_mm_srli_epi32(ssergba4, 16), sse255);

            // Compute R
            sseb1 = ssered1; //0
            sseb2 = _mm_sub_epi32(ssered2, ssered1);
            sseb3 = _mm_sub_epi32(ssered3, ssered1);
            sseb4 = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(ssered1, ssered4), ssered3), ssered2);

            ssev1 = _mm_slli_epi32(sseb1, 4); //0
            ssev2 = _mm_mullo_epi32(sseb2, ssedu); //00
            ssev3 = _mm_mullo_epi32(sseb3, ssedv); //0
            ssev4 = _mm_slli_epi32(_mm_add_epi32(_mm_add_epi32(ssev1, ssev2), ssev3), 4);
            ssev1 = _mm_mullo_epi32(_mm_mullo_epi32(sseb4, ssedu), ssedv);
            sseval = _mm_add_epi32(ssev4, ssev1);
            sseval = _mm_srli_epi32(sseval, 8);
            sse_red = _mm_min_epi32(sseval, sse255);

            // Compute G
            sseb1 = ssegreen1; //0
            sseb2 = _mm_sub_epi32(ssegreen2, ssegreen1);
            sseb3 = _mm_sub_epi32(ssegreen3, ssegreen1);
            sseb4 = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(ssegreen1, ssegreen4), ssegreen3), ssegreen2);

            ssev1 = _mm_slli_epi32(sseb1, 4); //0
            ssev2 = _mm_mullo_epi32(sseb2, ssedu); //00
            ssev3 = _mm_mullo_epi32(sseb3, ssedv); //0
            ssev4 = _mm_slli_epi32(_mm_add_epi32(_mm_add_epi32(ssev1, ssev2), ssev3), 4);
            ssev1 = _mm_mullo_epi32(_mm_mullo_epi32(sseb4, ssedu), ssedv);
            sseval = _mm_add_epi32(ssev4, ssev1);
            sseval = _mm_srli_epi32(sseval, 8);
            sse_green = _mm_min_epi32(sseval, sse255);

            // Compute B
            sseb1 = sseblue1; //0
            sseb2 = _mm_sub_epi32(sseblue2, sseblue1);
            sseb3 = _mm_sub_epi32(sseblue3, sseblue1);
            sseb4 = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(sseblue1, sseblue4), sseblue3), sseblue2);

            ssev1 = _mm_slli_epi32(sseb1, 4); //0
            ssev2 = _mm_mullo_epi32(sseb2, ssedu); //00
            ssev3 = _mm_mullo_epi32(sseb3, ssedv); //0
            ssev4 = _mm_slli_epi32(_mm_add_epi32(_mm_add_epi32(ssev1, ssev2), ssev3), 4);
            ssev1 = _mm_mullo_epi32(_mm_mullo_epi32(sseb4, ssedu), ssedv);
            sseval = _mm_add_epi32(ssev4, ssev1);
            sseval = _mm_srli_epi32(sseval, 8);
            sse_blue = _mm_min_epi32(sseval, sse255);

            // Compute output
            sse_green = _mm_slli_epi32(sse_green, 8);
            sse_blue  = _mm_slli_epi32(sse_blue, 16);
            sse_rgba  = _mm_add_epi8( _mm_add_epi8( _mm_add_epi8(sse_red, sse_green), sse_blue), sse_alpha);
            sse_rgba = _mm_shuffle_epi32(sse_rgba,_MM_SHUFFLE(0, 1, 2, 3));

            // Set output
            _mm_storeu_si128((__m128i*)ptrout, sse_rgba);

            ptrout+=4;

            ptrxy+=4;
        }
    }

function_terminate:
   return error;
}
