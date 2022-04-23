//==============================================================================
//
//    OPENROX   : File codedframe.c
//
//    Contents  : Implementation of codedframe module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "codedframe.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/image/remap/remap_bilinear_onepixel/remap_bilinear_onepixel.h>
#include <baseproc/tools/encoder/bch.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>

#define TEXTURE_SIZE       128
#define CODES_BLOCKS_SIZE     (TEXTURE_SIZE / 16)

#define WHITE_BORDER_SIZE    16 // old photoframe was 0, new is 16
#define BLACK_BORDER_SIZE    24

// The origin of the codes relative to the origin of the texture
#define CODES_ORIGIN            -CODES_BLOCKS_SIZE
// For exemple to put the code on the border set: #define CODES_ORIGIN -(BLACK_BORDER_SIZE + WHITE_BORDER_SIZE)

// Shift between the code and the texture
#define CODES_TSHIFT            -(CODES_ORIGIN + CODES_BLOCKS_SIZE)

Rox_ErrorCode rox_codedframe_new(Rox_CodedFrame *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CodedFrame ret = NULL;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_CodedFrame) rox_memory_allocate(sizeof(struct Rox_CodedFrame_Struct), 1);
   if (!ret) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->G = NULL;
   ret->cH = NULL;
   ret->value = 0;

   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&ret->G, 3, 3));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&ret->cH, 3, 3));

   // Set the internal homography G such that the origin is the top-left corner of the codes border

   CHECK_ERROR_TERMINATE(rox_array2d_double_fillunit(ret->G));
   CHECK_ERROR_TERMINATE(rox_array2d_double_set_value(ret->G, 0, 2, CODES_ORIGIN));
   CHECK_ERROR_TERMINATE(rox_array2d_double_set_value(ret->G, 1, 2, CODES_ORIGIN));

   error = ROX_ERROR_NONE;
   *obj = ret;

function_terminate:
   if (error) rox_codedframe_del(&ret);
   return error;
}

Rox_ErrorCode rox_codedframe_del(Rox_CodedFrame *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CodedFrame todel = NULL;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->G);
   rox_array2d_double_del(&todel->cH);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_codedframe_make64(Rox_CodedFrame obj, Rox_Image image, Rox_MatSL3 H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar val;
   Rox_Double u, v, nu, nv, nw;
   Rox_Point2D_Float_Struct pts[64];
   Rox_Uchar values[64] = {0};
   Rox_Ulint res = 0;

   if (!obj || !image || !H)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // The homography H contains the position on the texture
   // The homography G contains the shift to trasnform the origin to the top-left corner of the photoframe
   error = rox_array2d_double_mulmatmat(obj->cH, H, obj->G);

   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->cH);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Left codeblocks
   for (Rox_Sint i = 0; i < 16; i++)
   {
      // Get the center of the 8x8 squares (codeblock)
      u = CODES_BLOCKS_SIZE / 2 ;
      v = CODES_BLOCKS_SIZE / 2 + CODES_TSHIFT + CODES_BLOCKS_SIZE + i * CODES_BLOCKS_SIZE;

      nu = dh[0][0] * u + dh[0][1] * v + dh[0][2];
      nv = dh[1][0] * u + dh[1][1] * v + dh[1][2];
      nw = dh[2][0] * u + dh[2][1] * v + dh[2][2];
      pts[0 + i].u = (Rox_Float)(nu / nw);
      pts[0 + i].v = (Rox_Float)(nv / nw);
   }

   // Right codeblocks
   for (Rox_Sint i = 0; i < 16; i++)
   {
      u = CODES_BLOCKS_SIZE / 2 + 2*CODES_TSHIFT + CODES_BLOCKS_SIZE + TEXTURE_SIZE;
      v = CODES_BLOCKS_SIZE / 2 +   CODES_TSHIFT + CODES_BLOCKS_SIZE + i * CODES_BLOCKS_SIZE;

      nu = dh[0][0] * u + dh[0][1] * v + dh[0][2];
      nv = dh[1][0] * u + dh[1][1] * v + dh[1][2];
      nw = dh[2][0] * u + dh[2][1] * v + dh[2][2];
      pts[16 + i].u = (Rox_Float)(nu / nw);
      pts[16 + i].v = (Rox_Float)(nv / nw);
   }

   // Top codeblocks
   for (Rox_Sint i = 0; i < 16; i++)
   {
      u = CODES_BLOCKS_SIZE / 2 + CODES_TSHIFT + CODES_BLOCKS_SIZE + i * CODES_BLOCKS_SIZE;
      v = CODES_BLOCKS_SIZE / 2 ;

      nu = dh[0][0] * u + dh[0][1] * v + dh[0][2];
      nv = dh[1][0] * u + dh[1][1] * v + dh[1][2];
      nw = dh[2][0] * u + dh[2][1] * v + dh[2][2];
      pts[32 + i].u = (Rox_Float)(nu / nw);
      pts[32 + i].v = (Rox_Float)(nv / nw);
   }

   // Bottom codeblocks
   for (Rox_Sint i = 0; i < 16; i++)
   {
      u = CODES_BLOCKS_SIZE / 2 +   CODES_TSHIFT + CODES_BLOCKS_SIZE + i * CODES_BLOCKS_SIZE;
      v = CODES_BLOCKS_SIZE / 2 + 2*CODES_TSHIFT + CODES_BLOCKS_SIZE + TEXTURE_SIZE;

      nu = dh[0][0] * u + dh[0][1] * v + dh[0][2];
      nv = dh[1][0] * u + dh[1][1] * v + dh[1][2];
      nw = dh[2][0] * u + dh[2][1] * v + dh[2][2];
      pts[48 + i].u = (Rox_Float)(nu / nw);
      pts[48 + i].v = (Rox_Float)(nv / nw);
   }

   for (Rox_Sint i = 0; i < 64; i++)
   {
      error = rox_remap_onepixel_bilinear_uchar(&val, image, &pts[i]);
      if (error) val = 0;

      if (val < 128) values[i] = 0;
      else values[i] = 1;
   }

   res = 0;
   for (Rox_Sint i = 0; i < 64; i++)
   {
      if (values[i])
      {
         long bin = i;
         res |= (Rox_Ulint) (1ULL << bin);
      }
   }

   error = rox_bch_c8_e8_decode(&val, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->value = val;

function_terminate:
   return error;
}

Rox_ErrorCode rox_codedframe_make16(Rox_CodedFrame obj, Rox_Image image, Rox_MatSL3 H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar val;
   Rox_Point2D_Float_Struct pts[16];
   Rox_Uchar values[16];
   Rox_Ushort res = 0;

   if (!obj || !image || !H)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_mulmatmat(obj->cH, H, obj->G);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->cH);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Bottom
   for ( Rox_Sint i = 0; i < 16; i++)
   {
      Rox_Double u = CODES_BLOCKS_SIZE / 2 +    CODES_TSHIFT + CODES_BLOCKS_SIZE + i * CODES_BLOCKS_SIZE;
      Rox_Double v = CODES_BLOCKS_SIZE / 2 + 2*CODES_TSHIFT + CODES_BLOCKS_SIZE + TEXTURE_SIZE;

      Rox_Double nu = dh[0][0] * u + dh[0][1] * v + dh[0][2];
      Rox_Double nv = dh[1][0] * u + dh[1][1] * v + dh[1][2];
      Rox_Double nw = dh[2][0] * u + dh[2][1] * v + dh[2][2];

      pts[i].u = (Rox_Float) (nu / nw);
      pts[i].v = (Rox_Float) (nv / nw);
   }

   for ( Rox_Sint i = 0; i < 16; i++)
   {
      error = rox_remap_onepixel_bilinear_uchar(&val, image, &pts[i]);
      if (error) val = 0;

      if (val < 128) values[i] = 0;
      else values[i] = 1;
   }

   res = 0;
   for ( Rox_Sint i = 0; i < 16; i++)
   {
      if (values[i])
      {
         Rox_Sint bin = i;
         res |= (1 << bin);
      }
   }

   error = rox_bch_c6_e2_decode(&val, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->value = val;

function_terminate:
   return error;
}

Rox_ErrorCode rox_codedframe_get_value(Rox_Sint * val, Rox_CodedFrame obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj|| !val) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *val = (Rox_Sint) obj->value ;

function_terminate:
   return error;
}