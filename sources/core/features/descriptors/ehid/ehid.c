//==============================================================================
//
//    OPENROX   : File ehid.c
//
//    Contents  : Implementation of ehid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid.h"
#include "ehid_point_struct.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <generated/dynvec_ehid_point_struct.h>

#include <baseproc/maths/base/basemaths.h>

#include <inout/system/errors_print.h>

//! To be commented
union i64_Union
{
   //! To be commented
   Rox_Int64 big;
   //! To be commented
   Rox_Uint half[2];
};

typedef union i64_Union i64;

Rox_ErrorCode rox_ehid_point_quantizepatch(Rox_Uchar * out_mean, Rox_Uchar output[64], Rox_Double lums[64]);
Rox_ErrorCode rox_ehid_points_computedescriptor(Rox_DynVec_Ehid_Point ptr, Rox_Image source);

Rox_Void rox_ehid_description_from_int_to_bits(Rox_Ehid_Description bits, Rox_Uint ints[64][5])
{
   int id = 0;
   int bin = 0;
   int i,j;

   // Initialize description
   bits[0] = 0;
   bits[1] = 0;
   bits[2] = 0;
   bits[3] = 0;
   bits[4] = 0;

   // Loop over samples
   for (i = 0; i < 64; i++)
   {
      // Loop over histogram
      for (j = 0; j < 5; j++)
      {
         // Int to bit conversion (each int is either 0 or 1)
         if (ints[i][j] == 1) bits[id] |= (((Rox_Int64)1) << bin);

         // Jump to next bit
         bin++;
         if (bin == 64)
         {
            id++;
            bin = 0;
         }
      }

      // The 6th 64-bit pack is here for SIMD computation alignment only
      bits[5] = 0;
   }
}

Rox_Void rox_ehid_description_from_bits_to_int(Rox_Uint ints[64][5], Rox_Ehid_Description bits)
{
   int id = 0;
   int bin = 0;
   int i,j;

   for (i = 0; i < 64; i++)
   {
      for (j = 0; j < 5; j++)
      {
         // Bit to Int conversion (each int is either 0 or 1)
         if ((((Rox_Int64)1) << bin) & bits[id]) ints[i][j] = 1;
         else ints[i][j] = 0;

         // Jump to next bit
         bin++;
         if (bin == 64)
         {
            id++;
            bin = 0;
         }
      }
   }
}

Rox_ErrorCode rox_ehid_points_computeorientation_moments(Rox_DynVec_Ehid_Point ptr, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint u,v,k,l,vk,ul;
   Rox_Ehid_Point  pt;
   Rox_Uchar ** source_data;
   const Rox_Sint radius = 4;
   Rox_Sint m01, m10, pix;
   Rox_Double length, ilength;


   if (!ptr || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Note that we assume the points are not out of the image (the whole patch 17*17)
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&source_data, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // For each point, we estimate the principal orientation as defined in the paper by Drummond
   for (Rox_Uint n = 0; n < ptr->used; n++)
   {
      pt = &ptr->data[n];

      u = (int)(pt->pos.u + 0.5);
      v = (int)(pt->pos.v + 0.5);

      m01 = 0;
      m10 = 0;

      for (k = -radius; k <= radius; k++)
      {
         vk = v + k;

         for (l = -radius; l <= radius; l++)
         {
            ul = u + l;
            pix = source_data[vk][ul];

            m01 += l * pix;
            m10 += k * pix;
         }
      }

      length = sqrt((double)(m01*m01 + m10*m10));
      if (length == 0)
      {
         pt->dir.u = 1;
         pt->dir.v = 0;
      }
      else
      {
         ilength = 1.0 / length;
         pt->dir.u = ilength * (Rox_Double)m01;
         pt->dir.v = ilength * (Rox_Double)m10;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_computeorientation(Rox_DynVec_Ehid_Point ptr, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double dx[8];
   Rox_Double dy[8];
   Rox_Double dl[8];
   Rox_Double magx;
   Rox_Double magy;
   Rox_Double norme, inorme;
   Rox_Sint u,v;
   Rox_Uint n;
   Rox_Ehid_Point  pt;
   Rox_Uchar ** source_data;


   if (!ptr || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Note that we assume the points are not out of the image (the whole patch 17*17)

   error = rox_array2d_uchar_get_data_pointer_to_pointer(&source_data, source);

   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve the 3.4 bresenham circle coefficients for half the circle (16/2)
   dx[0] = 3.0; dy[0] = 0.0;
   dx[1] = 3.0; dy[1] = -1.0;
   dx[2] = 2.0; dy[2] = -2.0;
   dx[3] = 1.0; dy[3] = -3.0;
   dx[4] = 0.0; dy[4] = -3.0;
   dx[5] = -1.0; dy[5] = -3.0;
   dx[6] = -2.0; dy[6] = -2.0;
   dx[7] = 3.0; dy[7] = 1.0;

   // For each point, we estimate the principal orientation as defined in the paper by Drummond
   for (n = 0; n < ptr->used; n++)
   {
      pt = &ptr->data[n];

      u = (int)(pt->pos.u + 0.5);
      v = (int)(pt->pos.v + 0.5);

      // Compute the various gradients along each direction of the half circle (using same circle than fast9)
      dl[0] = (double)source_data[v][u + 3] - (double)source_data[v][u - 3];
      dl[1] = (double)source_data[v - 1][u + 3] - (double)source_data[v + 1][u - 3];
      dl[2] = (double)source_data[v - 2][u + 2] - (double)source_data[v + 2][u - 2];
      dl[3] = (double)source_data[v - 3][u + 1] - (double)source_data[v + 3][u - 1];
      dl[4] = (double)source_data[v - 3][u] - (double)source_data[v + 3][u];
      dl[5] = (double)source_data[v - 3][u - 1] - (double)source_data[v + 3][u + 1];
      dl[6] = (double)source_data[v - 2][u - 2] - (double)source_data[v + 2][u + 2];
      dl[7] = (double)source_data[v + 1][u + 3] - (double)source_data[v - 1][u - 3];

      // Magnitude estimation of the x and y vectors
      magx = dl[0] * dx[0] + dl[1] * dx[1] + dl[2] * dx[2] + dl[3] * dx[3] + dl[4] * dx[4] + dl[5] * dx[5] + dl[6] * dx[6] + dl[7] * dx[7];
      magy = dl[0] * dy[0] + dl[1] * dy[1] + dl[2] * dy[2] + dl[3] * dy[3] + dl[4] * dy[4] + dl[5] * dy[5] + dl[6] * dy[6] + dl[7] * dy[7];

      // The result vector gives a coarse estimation of the main orientation

      // Normalize vector
      norme = sqrt(magx*magx + magy*magy);
      if (fabs(norme) < DBL_EPSILON)
      {
         pt->dir.u = 1;
         pt->dir.v = 0;
      }
      else
      {
         inorme = 1.0 / norme;
         pt->dir.u = magx * inorme;
         pt->dir.v = magy * inorme;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_warp(Rox_DynVec_Ehid_Point ptr, Rox_Array2D_Double H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** hom_data = NULL;
   Rox_Ehid_Point  pt = NULL;

   double w;
   double cu,cv;
   double ou,ov;
   double cx,cy;
   double dx,dy;

   Rox_Double norme, inorme;


   if (!ptr || !H)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&hom_data, H);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // For each point in the db
   for (Rox_Uint n = 0; n < ptr->used; n++)
   {
      pt = &ptr->data[n];

      // Warp center
      cx = pt->pos.u;
      cy = pt->pos.v;
      cu = hom_data[0][0] * cx + hom_data[0][1] * cy + hom_data[0][2];
      cv = hom_data[1][0] * cx + hom_data[1][1] * cy + hom_data[1][2];
      w = hom_data[2][0] * cx + hom_data[2][1] * cy + hom_data[2][2];
      cu = cu / w;
      cv = cv / w;

      // Warp orientated point
      cx = pt->pos.u + pt->dir.u;
      cy = pt->pos.v + pt->dir.v;
      ou = hom_data[0][0] * cx + hom_data[0][1] * cy + hom_data[0][2];
      ov = hom_data[1][0] * cx + hom_data[1][1] * cy + hom_data[1][2];
      w = hom_data[2][0] * cx + hom_data[2][1] * cy + hom_data[2][2];
      ou = ou / w;
      ov = ov / w;


      // Compute orientation vector
      dx = ou - cu;
      dy = ov - cv;

      // Normalize orientation vector
      norme = sqrt(dx*dx + dy*dy);
      if (fabs(norme) < DBL_EPSILON)
      {
         dx = 1;
         dy = 0;
      }
      else
      {
         inorme = 1.0 / norme;
         dx = dx * inorme;
         dy = dy * inorme;
      }

      // Assign new properties
      pt->pos.u = cu;
      pt->pos.v = cv;
      pt->dir.u = dx;
      pt->dir.v = dy;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_compute(Rox_DynVec_Ehid_Point ptr, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ptr || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Computing points description
   error = rox_ehid_points_computeorientation_moments(ptr, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ehid_points_computedescriptor(ptr, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_set_target(Rox_DynVec_Ehid_Point ptr, Rox_Uint targetid)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < ptr->used; id++)
   {
      ptr->data[id].dbid = targetid;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_save(char * filename, Rox_DynVec_Ehid_Point ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!ptr || !filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   out = fopen(filename, "wb");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_ehid_points_save_stream(out, ptr);

   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_ehid_points_save_stream(FILE * output, Rox_DynVec_Ehid_Point ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr || !output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   fwrite(&ptr->used, sizeof(Rox_Uint), 1, output);

   for (Rox_Uint id = 0; id < ptr->used; id++)
   {
      fwrite(&ptr->data[id].uid, sizeof(Rox_Uint), 1, output);
      fwrite(&ptr->data[id].dbid, sizeof(Rox_Uint), 1, output);
      fwrite(&ptr->data[id].pos.u, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].pos.v, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].pos_meters.u, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].pos_meters.v, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].dir.u, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].dir.v, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].scale, sizeof(Rox_Double), 1, output);
      fwrite(&ptr->data[id].Description, sizeof(Rox_Ehid_Description), 1, output);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_load(Rox_DynVec_Ehid_Point ptr, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

   if (!ptr || !filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   in = fopen(filename, "rb");
   if (!in) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_ehid_points_load_stream(ptr, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(in) fclose(in);
   return error;
}

Rox_ErrorCode rox_ehid_points_load_stream ( Rox_DynVec_Ehid_Point ptr, FILE * input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint count, cnt;
   Rox_Ehid_Point_Struct pt;


   if (!ptr || !input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(ptr);

   cnt = (Rox_Uint) fread(&count, sizeof(Rox_Uint), 1, input);
   if (cnt != 1) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   for (Rox_Uint id = 0; id < count; id++)
   {

      cnt = (Rox_Uint) fread(&pt.uid, sizeof(Rox_Uint), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.dbid, sizeof(Rox_Uint), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.pos.u, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.pos.v, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.pos_meters.u, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.pos_meters.v, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
       {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.dir.u, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.dir.v, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.scale, sizeof(Rox_Double), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      cnt = (Rox_Uint) fread(&pt.Description, sizeof(Rox_Ehid_Description), 1, input);
      if (cnt != 1)
         {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

      pt.refcount = 0;
      pt.index = 0;

      rox_dynvec_ehid_point_append(ptr, &pt);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_point_match (
   Rox_Uint * score,
   Rox_Ehid_Description pt1,
   Rox_Ehid_Description pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint cntbits;


   if (!score || !pt1 || !pt2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = 5;

#ifdef ROX_USE_SSE
   i64 tmp;

   #ifdef ROX_IS_32BITS
      tmp.big = pt1[0] & pt2[0];
      cntbits = _mm_popcnt_u32(tmp.half[0]) + _mm_popcnt_u32(tmp.half[1]);
      tmp.big = pt1[1] & pt2[1];
      cntbits += _mm_popcnt_u32(tmp.half[0]) + _mm_popcnt_u32(tmp.half[1]);
      tmp.big = pt1[2] & pt2[2];
      cntbits += _mm_popcnt_u32(tmp.half[0]) + _mm_popcnt_u32(tmp.half[1]);
      tmp.big = pt1[3] & pt2[3];
      cntbits += _mm_popcnt_u32(tmp.half[0]) + _mm_popcnt_u32(tmp.half[1]);
      tmp.big = pt1[4] & pt2[4];
      cntbits += _mm_popcnt_u32(tmp.half[0]) + _mm_popcnt_u32(tmp.half[1]);
   #else
      tmp.big = pt1[0] & pt2[0];
      cntbits = (Rox_Uint) _mm_popcnt_u64(tmp.big);
      tmp.big = pt1[1] & pt2[1];
      cntbits += (Rox_Uint) _mm_popcnt_u64(tmp.big);
      tmp.big = pt1[2] & pt2[2];
      cntbits += (Rox_Uint) _mm_popcnt_u64(tmp.big);
      tmp.big = pt1[3] & pt2[3];
      cntbits += (Rox_Uint) _mm_popcnt_u64(tmp.big);
      tmp.big = pt1[4] & pt2[4];
      cntbits += (Rox_Uint) _mm_popcnt_u64(tmp.big);
   #endif

#else

   #ifdef ROX_USE_NEON
      {
         Rox_Sint pos;
         Rox_Uchar * pdb = (Rox_Uchar*)&pt1[0];
         Rox_Uchar * pdet = (Rox_Uchar*)&pt2[0];

         cntbits = 0;
         for (pos = 0; pos < 48; pos += 16)
         {
            uint8x16_t A_vec = vld1q_u8 (pdb + pos);
            uint8x16_t B_vec = vld1q_u8 (pdet + pos);
            uint8x16_t AxorB = vandq_u8 (A_vec, B_vec);
            uint8x16_t bitsSet = vcntq_u8 (AxorB);
            uint16x8_t bitSet8 = vpaddlq_u8 (bitsSet);
            uint32x4_t bitSet4 = vpaddlq_u16 (bitSet8);
            uint64x2_t bitSet2 = vpaddlq_u32 (bitSet4);
            cntbits += vgetq_lane_u64 (bitSet2,0);
            cntbits += vgetq_lane_u64 (bitSet2,1);
         }
      }

   #else

      i64 tmp;

      tmp.big = pt1[0] & pt2[0];
      cntbits = _rox_popcount_slow(tmp.half[0]) + _rox_popcount_slow(tmp.half[1]);
      tmp.big = pt1[1] & pt2[1];
      cntbits += _rox_popcount_slow(tmp.half[0]) + _rox_popcount_slow(tmp.half[1]);
      tmp.big = pt1[2] & pt2[2];
      cntbits += _rox_popcount_slow(tmp.half[0]) + _rox_popcount_slow(tmp.half[1]);
      tmp.big = pt1[3] & pt2[3];
      cntbits += _rox_popcount_slow(tmp.half[0]) + _rox_popcount_slow(tmp.half[1]);
      tmp.big = pt1[4] & pt2[4];
      cntbits += _rox_popcount_slow(tmp.half[0]) + _rox_popcount_slow(tmp.half[1]);
   #endif
#endif

   *score = cntbits;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_serialize(char* ser, const Rox_DynVec_Ehid_Point ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;

   Rox_Float posu, posv, posmeter_u, posmeter_v, diru, dirv, scale;
   Rox_Ushort dbid;
   Rox_Uint uid;
   Rox_Uchar index;

   if(!ser || !ptr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   memcpy(ser + offset, &ptr->used, sizeof(ptr->used));
   offset += sizeof(ptr->used);

   for (Rox_Uint id = 0; id < ptr->used; id++)
   {
      //  Cast data
      posu = (Rox_Float)ptr->data[id].pos.u;
      posv = (Rox_Float)ptr->data[id].pos.v;
      posmeter_u = (Rox_Float)ptr->data[id].pos_meters.u;
      posmeter_v = (Rox_Float)ptr->data[id].pos_meters.v;
      diru = (Rox_Float)ptr->data[id].dir.u;
      dirv = (Rox_Float)ptr->data[id].dir.v;
      scale = (Rox_Float)ptr->data[id].scale;

      dbid = (Rox_Ushort)ptr->data[id].dbid;

      index = (Rox_Uchar)ptr->data[id].index;
      uid = ptr->data[id].uid;

      memcpy(ser + offset, &uid, sizeof(uid));
      offset += sizeof(uid);

      memcpy(ser + offset, &dbid, sizeof(dbid));
      offset += sizeof(dbid);

      memcpy(ser + offset, &posu, sizeof(posu));
      offset += sizeof(posu);

      memcpy(ser + offset, &posv, sizeof(posv));
      offset += sizeof(posv);

      memcpy(ser + offset, &posmeter_u, sizeof(posmeter_u));
      offset += sizeof(posmeter_u);

      memcpy(ser + offset, &posmeter_v, sizeof(posmeter_v));
      offset += sizeof(posmeter_v);

      memcpy(ser + offset, &diru, sizeof(diru));
      offset += sizeof(diru);

      memcpy(ser + offset, &dirv, sizeof(dirv));
      offset += sizeof(dirv);

      memcpy(ser + offset, &scale, sizeof(scale));
      offset += sizeof(scale);

      memcpy(ser + offset, &ptr->data[id].Description, 5*sizeof(ptr->data[id].Description[0]));
      offset += 5*sizeof(ptr->data[id].Description[0]);

      memcpy(ser + offset, &index, sizeof(index));
      offset += sizeof(index);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_deserialize(Rox_DynVec_Ehid_Point ptr, const char* ser)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0, used = 0;
   Rox_Ehid_Point_Struct pt;

   Rox_Float posu, posv, posmeter_u, posmeter_v, diru, dirv, scale;
   Rox_Ushort dbid;
   Rox_Uint uid;
   Rox_Uchar index;

   if(!ptr || !ser)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   //  Reset ptr
   rox_dynvec_ehid_point_reset(ptr);

   memcpy(&used, ser + offset, sizeof(used));
   offset += sizeof(used);

   for (Rox_Uint i = 0; i < used; i++)
   {
      memcpy(&uid, ser + offset, sizeof(uid));
      offset += sizeof(uid);

      memcpy(&dbid, ser + offset, sizeof(dbid));
      offset += sizeof(dbid);

      memcpy(&posu, ser + offset, sizeof(posu));
      offset += sizeof(posu);

      memcpy(&posv, ser + offset, sizeof(posv));
      offset += sizeof(posv);

      memcpy(&posmeter_u, ser + offset, sizeof(posmeter_u));
      offset += sizeof(posmeter_u);

      memcpy(&posmeter_v, ser + offset, sizeof(posmeter_v));
      offset += sizeof(posmeter_v);

      memcpy(&diru, ser + offset, sizeof(diru));
      offset += sizeof(diru);

      memcpy(&dirv, ser + offset, sizeof(dirv));
      offset += sizeof(dirv);

      memcpy(&scale, ser + offset, sizeof(scale));
      offset += sizeof(scale);

      memcpy(&pt.Description, ser + offset, 5*sizeof(pt.Description[0]));
      offset += 5*sizeof(pt.Description[0]);

      memcpy(&index, ser + offset, sizeof(index));
      offset += sizeof(index);


      //  Fill structure
      pt.uid = (Rox_Uint) uid;
      pt.dbid = (Rox_Uint) dbid;
      pt.pos.u = (Rox_Double)posu;
      pt.pos.v = (Rox_Double)posv;
      pt.pos_meters.u = (Rox_Double)posmeter_u;
      pt.pos_meters.v = (Rox_Double)posmeter_v;
      pt.dir.u = (Rox_Double)diru;
      pt.dir.v = (Rox_Double)dirv;
      pt.scale = (Rox_Double)scale;
      pt.index = (Rox_Uint)index;

      error = rox_dynvec_ehid_point_append(ptr, &pt);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_points_get_octet_size(Rox_Uint *size, Rox_DynVec_Ehid_Point ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ret = 0;

   if(!size || !ptr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *size = 0;

   ret += sizeof(Rox_Uint);  //   uid
   ret += sizeof(Rox_Ushort); //   dbid
   ret += sizeof(Rox_Float); //   posu
   ret += sizeof(Rox_Float); //   posv
   ret += sizeof(Rox_Float); //   diru
   ret += sizeof(Rox_Float); //   dirv
   ret += sizeof(Rox_Float); //   pos_metersu
   ret += sizeof(Rox_Float); //   pos_metersv
   ret += sizeof(Rox_Float); //   scale
   ret += 5*sizeof(Rox_Int64); //  Descriptor
   ret += sizeof(Rox_Uchar); //  index

   ret = ret * ptr->used;
   ret += sizeof(ptr->used);

   *size = ret;

function_terminate:
   return error;
}
