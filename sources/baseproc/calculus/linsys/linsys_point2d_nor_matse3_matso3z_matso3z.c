//==============================================================================
//
//    OPENROX   : File linsys_point2d_nor_matse3_matso3z_matso3z.c
//
//    Contents  : Implementation of linsys_point2d_nor_matse3_matso3z_matso3z
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_point2d_nor_matse3_matso3z_matso3z.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/point/point3d.h>

#include <generated/dynvec_point3d_float_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_point2d_nor_matse3_matso3z_matso3z(
   Rox_Matrix       LtL,
   Rox_Matrix       Lte,
   const Rox_Array2D_Double       diff_b,
   const Rox_Array2D_Double       weight_b,
   const Rox_DynVec_Point3D_Float mb_c,
   const Rox_Array2D_Double       diff_s,
   const Rox_Array2D_Double       weight_s,
   const Rox_DynVec_Point3D_Float ms_c,
   const Rox_DynVec_Point3D_Float msu_g, 
   const Rox_DynVec_Point3D_Float msu_s 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double    L1[8]      , L2[8];
   Rox_Double    **dLtL=NULL, *dLte=NULL;
   Rox_Double    *ddb=NULL  , *dds=NULL  , *dwb=NULL   , *dws=NULL;

   if ( !LtL || !Lte || !diff_b || !weight_b || !mb_c || !diff_s || !weight_s || !ms_c || !msu_g || !msu_s )
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Sint size_b  = mb_c->used;
   Rox_Sint size_s  = ms_c->used;
   Rox_Sint sizeu_g = msu_g->used;
   Rox_Sint sizeu_s = msu_s->used;

   if ( sizeu_g != size_s )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( sizeu_s != size_s )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size(LtL, 8, 8);                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 8, 1);                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(diff_b   , size_b * 2 , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(weight_b , size_b     , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(diff_s   , size_s * 2 , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(weight_s , size_s     , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point3D_Float dmb_c  = mb_c->data;
   Rox_Point3D_Float dms_c  = ms_c->data;
   Rox_Point3D_Float dmsu_g = msu_g->data;
   Rox_Point3D_Float dmsu_s = msu_s->data;

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dLtL, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer ( &dLte, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer ( &ddb, diff_b );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer ( &dds, diff_s );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer ( &dwb, weight_b );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer ( &dws, weight_s );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < ( size_b + size_s ); i++ )
   {
      Rox_Double x = 0.0, y = 0.0, Z = 0.0, zi = 0.0, w = 0.0, d1 = 0.0, d2 = 0.0;

      if ( i < size_b )
      {
         Rox_Sint ir = i;
         x  = dmb_c[ir].X / dmb_c[ir].Z;
         y  = dmb_c[ir].Y / dmb_c[ir].Z;
         Z  = dmb_c[ir].Z;
         zi = 1.0 / Z;

         w  = dwb[ir];
         d1 = w * ddb[ir * 2];
         d2 = w * ddb[ir * 2 + 1];

         L1[6] = 0.0;
         L2[6] = 0.0;

         L1[7] = 0.0;
         L2[7] = 0.0;
      }
      else
      {
         Rox_Sint il = i - size_b;
         x  = dms_c[il].X / dms_c[il].Z;
         y  = dms_c[il].Y / dms_c[il].Z;
         Z  = dms_c[il].Z;
         zi = 1.0 / Z;

         w  = dws[il];
         d1 = w * dds[il * 2];
         d2 = w * dds[il * 2 + 1];

         L1[6] = zi * ( dmsu_g[il].X - x * dmsu_g[il].Z ); 
         L2[6] = zi * ( dmsu_g[il].Y - y * dmsu_g[il].Z ); 

         L1[7] = zi * ( dmsu_s[il].X - x * dmsu_s[il].Z ); 
         L2[7] = zi * ( dmsu_s[il].Y - y * dmsu_s[il].Z ); 
      }

      // Translation : 1st row 
      L1[0] = -zi * w;
      L1[1] = 0.0;
      L1[2] = +zi * x * w;

      // Rotation : 1st row 
      L1[3] = x * y * w;
      L1[4] = (- x * x - 1.0) * w;
      L1[5] = y * w ;

      // Translation : 2nd row 
      L2[0] =  0.0;
      L2[1] = -zi * w;
      L2[2] =  zi * y * w;

      // Rotation : 2nd row 
      L2[3] = ( y * y + 1.0) * w;
      L2[4] = - x * y * w;
      L2[5] = - x * w;

      for (Rox_Sint k = 0; k < 8; k++)
      {
         for (Rox_Sint l = 0; l <= k; l++)
         {
            dLtL[k][l] += L1[k] * L1[l] + L2[k] * L2[l];
         }
         dLte[k] += L1[k] * d1 + L2[k] * d2;
      }
   }

   for (Rox_Sint k = 0; k < 8; k++)
   {
      for (Rox_Sint l = k; l < 8; l++)
      {
         dLtL[k][l] = dLtL[l][k];
      }
   }

function_terminate:
   return error;
}

