//==============================================================================
//
//    OPENROX   : File nonoverlaperror.c
//
//    Contents  : Implementation of nonoverlaperror module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "nonoverlaperror.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_nonoverlap_geometric_error(Rox_Array2D_Double vecerrors, Rox_Array2D_Double E, Rox_Array2D_Double pose, Rox_DynVec_Point2D_Double ar, Rox_DynVec_Point3D_Double br, Rox_DynVec_Point2D_Double ac, Rox_DynVec_Point3D_Double bc)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double a1, b1, c1, d1;
   Rox_Double a2, b2, c2, d2;
   Rox_Double a3, b3, c3, d3;

   Rox_Double arx, ary, arz;
   Rox_Double acx, acy, acz;
   Rox_Double brx, bry, brz;
   Rox_Double bcx, bcy, bcz;

   if (!vecerrors || !E || !pose || !ar || !br || !ac || !bc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (ar->used != ac->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (br->used != bc->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (ar->used != br->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(E, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(vecerrors, ar->used * 2, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** de = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &de, E );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dve = NULL;
   error = rox_array2d_double_get_data_pointer ( &dve, vecerrors );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < ac->used; i++)
   {
      arx = ar->data[i].u;
      ary = ar->data[i].v;
      arz = 1.0;
      acx = ac->data[i].u;
      acy = ac->data[i].v;
      acz = 1.0;
      brx = br->data[i].X;
      bry = br->data[i].Y;
      brz = br->data[i].Z;
      bcx = bc->data[i].X;
      bcy = bc->data[i].Y;
      bcz = bc->data[i].Z;

      a1 = dt[0][0] * bcx + dt[1][0] * bcy + dt[2][0] * bcz;
      b1 = dt[0][1] * bcx + dt[1][1] * bcy + dt[2][1] * bcz;
      c1 = dt[0][2] * bcx + dt[1][2] * bcy + dt[2][2] * bcz;
      a2 = dt[0][0] * acx + dt[1][0] * acy + dt[2][0] * acz;
      b2 = dt[0][1] * acx + dt[1][1] * acy + dt[2][1] * acz;
      c2 = dt[0][2] * acx + dt[1][2] * acy + dt[2][2] * acz;
      a3 = de[0][0] * acx + de[1][0] * acy + de[2][0] * acz;
      b3 = de[0][1] * acx + de[1][1] * acy + de[2][1] * acz;
      c3 = de[0][2] * acx + de[1][2] * acy + de[2][2] * acz;

      d1 = (a1 * arx + b1 * ary + c1 * arz);
      d2 = (a2 * brx + b2 * bry + c2 * brz);
      d3 = (a3 * arx + b3 * ary + c3 * arz);

      dve[i*2] = d1 + d2 + d3;

      a1 = dt[0][0] * brx + dt[0][1] * bry + dt[0][2] * brz;
      b1 = dt[1][0] * brx + dt[1][1] * bry + dt[1][2] * brz;
      c1 = dt[2][0] * brx + dt[2][1] * bry + dt[2][2] * brz;
      a2 = dt[0][0] * arx + dt[0][1] * ary + dt[0][2] * arz;
      b2 = dt[1][0] * arx + dt[1][1] * ary + dt[1][2] * arz;
      c2 = dt[2][0] * arx + dt[2][1] * ary + dt[2][2] * arz;
      a3 = de[0][0] * arx + de[0][1] * ary + de[0][2] * arz;
      b3 = de[1][0] * arx + de[1][1] * ary + de[1][2] * arz;
      c3 = de[2][0] * arx + de[2][1] * ary + de[2][2] * arz;

      d1 = (a1 * acx + b1 * acy + c1 * acz);
      d2 = (a2 * bcx + b2 * bcy + c2 * bcz);
      d3 = (a3 * acx + b3 * acy + c3 * acz);


      dve[i * 2 + 1] = d1 + d2 + d3;
   }

function_terminate:
   return error;
}

