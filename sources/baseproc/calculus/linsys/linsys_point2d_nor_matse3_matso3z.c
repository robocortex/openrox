//==============================================================================
//
//    OPENROX   : File linsys_point2d_nor_matse3_matso3z.c
//
//    Contents  : Implementation of linsys_point2d_nor_matse3_matso3z
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_point2d_nor_matse3_matso3z.h"

#include <generated/dynvec_point3d_float_struct.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_intmat_se3_so3z_weighted_premul_float (
   Rox_Matrix         LtL,
   Rox_Matrix         Lte,
   const Rox_Array2D_Double         diffr,
   const Rox_Array2D_Double         weightr,
   const Rox_DynVec_Point3D_Float   mrc,
   const Rox_Array2D_Double         diffl,
   const Rox_Array2D_Double         weightl,
   const Rox_DynVec_Point3D_Float   mlc,
   const Rox_DynVec_Point3D_Float   mltmp 
)
{
   Rox_ErrorCode     error = ROX_ERROR_NONE;
   Rox_Double          L1[7]      , L2[7];
   Rox_Double          **dLtL=NULL, *dLte=NULL;
   Rox_Point3D_Float dmrc=NULL , dmlc=NULL, dmltmp=NULL;
   Rox_Double          *ddr=NULL  , *ddl=NULL , *dwr=NULL   , *dwl=NULL;
   Rox_Double          x=0.0      , y=0.0     , Z=0.0       , zi=0.0   , w=0.0, d1=0.0 , d2=0.0;
   Rox_Uint          i=0        , ir=0      , il=0        , k=0      , l=0  , sizer=0, sizel=0, sizetmp=0;

   if ( !LtL || !Lte || !diffr || !weightr || !mrc || !diffl || !weightl || !mlc || !mltmp )
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   sizer   = mrc->used;
   sizel   = mlc->used;
   sizetmp = mltmp->used;
   if ( sizetmp != sizel )
   {
      error = ROX_ERROR_ARRAYS_NOT_MATCH;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   
   error = rox_array2d_double_check_size(LtL, 7, 7);               ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(Lte, 7, 1);               ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(diffr   , sizer * 2 , 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(weightr , sizer     , 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(diffl   , sizel * 2 , 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(weightl , sizel     , 1); ROX_ERROR_CHECK_TERMINATE ( error );

   dmrc   = mrc->data;
   dmlc   = mlc->data;
   dmltmp = mltmp->data;

   error = rox_array2d_double_get_data_pointer_to_pointer( &dLtL, LtL);
   error = rox_array2d_double_get_data_pointer ( &dLte, Lte );
   error = rox_array2d_double_get_data_pointer ( &ddr, diffr );
   error = rox_array2d_double_get_data_pointer ( &ddl, diffl );
   error = rox_array2d_double_get_data_pointer ( &dwr, weightr );
   error = rox_array2d_double_get_data_pointer ( &dwl, weightl );

   rox_array2d_double_fillval(LtL, 0);
   rox_array2d_double_fillval(Lte, 0);

   for ( i = 0; i < ( sizer + sizel ); i++ )
   {
      if ( i < sizer )
      {
         ir = i;
         x  = dmrc[ir].X / dmrc[ir].Z;
         y  = dmrc[ir].Y / dmrc[ir].Z;
         Z  = dmrc[ir].Z;
         zi = 1.0 / Z;

         w  = dwr[ir];
         d1 = w * ddr[ir * 2];
         d2 = w * ddr[ir * 2 + 1];

         L1[6] = 0.0;
         L2[6] = 0.0;
      }
      else
      {
         il = i - sizer;
         x  = dmlc[il].X / dmlc[il].Z;
         y  = dmlc[il].Y / dmlc[il].Z;
         Z  = dmlc[il].Z;
         zi = 1.0 / Z;

         w  = dwl[il];
         d1 = w * ddl[il * 2];
         d2 = w * ddl[il * 2 + 1];

         L1[6] = zi * ( dmltmp[il].X - x * dmltmp[il].Z ); 
         L2[6] = zi * ( dmltmp[il].Y - y * dmltmp[il].Z ); 
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

      for (k = 0; k < 7; k++)
      {
         for (l = 0; l <= k; l++)
            dLtL[k][l] += L1[k] * L1[l] + L2[k] * L2[l];

         dLte[k] += L1[k] * d1 + L2[k] * d2;
      }
   }

   for (k = 0; k < 7; k++)
      for (l = k; l < 7; l++)
         dLtL[k][l] = dLtL[l][k];

function_terminate:
   return error;
}

