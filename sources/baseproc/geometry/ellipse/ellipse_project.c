//==============================================================================
//
//    OPENROX   : File ellipse_project.c
//
//    Contents  : Implementation of ellipse_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ellipse_project.h"

#include <baseproc/geometry/ellipse/ellipse2d_struct.h>
#include <baseproc/geometry/ellipse/ellipse3d_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <float.h>
#include <math.h>

Rox_ErrorCode rox_ellipse2d_project_ellipse3d(Rox_Ellipse2D ellipse2d, Rox_Array2D_Double matct2, Rox_Ellipse3D ellipse3d)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d || !ellipse3d || !matct2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_MatSE3 eTc = NULL;
   Rox_MatSE3 cTe = ellipse3d->Te;

   Rox_Double a = ellipse3d->a;
   Rox_Double b = ellipse3d->b;
  
   error = rox_matse3_new(&eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // eTc = inv(cTe);
   error = rox_matse3_inv(eTc, cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T, eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K, matct2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // R = eTc(1:3,1:3);
   // t = eTc(1:3,4:4);

   Rox_Double fui =      1.0/K[0][0];
   Rox_Double fvi =      1.0/K[1][1];
   Rox_Double cui = -K[0][2]/K[0][0];
   Rox_Double cvi = -K[1][2]/K[1][1];

   Rox_Double M[3][3];
   // M = R*inv(Kc);

   M[0][0] = fui*T[0][0]; M[0][1] = fvi*T[0][1]; M[0][2] = T[0][2] + cui*T[0][0] + cvi*T[0][1];
   M[1][0] = fui*T[1][0]; M[1][1] = fvi*T[1][1]; M[1][2] = T[1][2] + cui*T[1][0] + cvi*T[1][1];
   M[2][0] = fui*T[2][0]; M[2][1] = fvi*T[2][1]; M[2][2] = T[2][2] + cui*T[2][0] + cvi*T[2][1];

   Rox_Double c11 = (T[2][3]*M[0][0]-T[0][3]*M[2][0])/a;
   Rox_Double c12 = (T[2][3]*M[0][1]-T[0][3]*M[2][1])/a;
   Rox_Double c13 = (T[2][3]*M[0][2]-T[0][3]*M[2][2])/a;

   Rox_Double c21 = (T[2][3]*M[1][0]-T[1][3]*M[2][0])/b;
   Rox_Double c22 = (T[2][3]*M[1][1]-T[1][3]*M[2][1])/b;
   Rox_Double c23 = (T[2][3]*M[1][2]-T[1][3]*M[2][2])/b;

   Rox_Double c31 = M[2][0];
   Rox_Double c32 = M[2][1];
   Rox_Double c33 = M[2][2];

   Rox_Double sxx = (c11*c11+c21*c21-c31*c31);
   Rox_Double syy = (c12*c12+c22*c22-c32*c32);
   Rox_Double sxy = (c11*c12+c21*c22-c31*c32);

   Rox_Double sx = (c11*c13+c21*c23-c31*c33);
   Rox_Double sy = (c12*c13+c22*c23-c32*c33);
   Rox_Double sp = (c13*c13+c23*c23-c33*c33);

   //Ae = [sxx, sxy; sxy, syy];
   //be = [sx ; sy];
   
   // pc = -inv(Ae)*be;

   Rox_Double d  = ( sxx*syy - sxy*sxy ) ;  
   if(d < DBL_EPSILON)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }
   Rox_Double uc = (sxy * sy - sx * syy) / d;
   Rox_Double vc = (sx * sxy - sxx * sy) / d;

   Rox_Double inv = sxx*uc*uc + syy*vc*vc + 2.0*sxy*uc*vc - sp;
   if (fabs(inv) < DBL_EPSILON)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Double k = 1.0/ inv;

   // ellipse in pixel coordinates

   Rox_Double nxx = sxx*k;
   Rox_Double nyy = syy*k;
   Rox_Double nxy = sxy*k;

   Rox_Double delta = nxx*nxx - 2 * nxx*nyy + 4 * nxy*nxy + nyy*nyy;
   if (delta < 0.0)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if ((nxx / 2 + nyy / 2 - sqrt(delta) / 2 < 0) || (nxx / 2 + nyy / 2 + sqrt(delta) / 2 < 0))
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   ellipse2d->xc = uc;
   ellipse2d->yc = vc;
   ellipse2d->nxx = nxx; 
   ellipse2d->nyy = nyy; 
   ellipse2d->nxy = nxy; 

function_terminate:
   rox_matse3_del(&eTc);
   return error;
}

Rox_ErrorCode rox_ellipse2d_transform_project_ellipse3d(Rox_Ellipse2D ellipse2d, Rox_Array2D_Double matct2, Rox_Array2D_Double matse3, Rox_Ellipse3D ellipse3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d || !matct2 || !matse3 || !ellipse3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_MatSE3 eTc = NULL;
   Rox_MatSE3 cTe = NULL;
   Rox_MatSE3 cTo = matse3;
   Rox_MatSE3 oTe = ellipse3d->Te;

   Rox_Double a = ellipse3d->a;
   Rox_Double b = ellipse3d->b;

   error = rox_matse3_new(&cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_matse3_new(&eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // cTe = cTo * oTe;
   error = rox_matse3_mulmatmat(cTe, cTo, oTe);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // eTc = inv(cTe);
   error = rox_matse3_inv(eTc, cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&T, eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&K, matct2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // R = eTc(1:3,1:3);
   // t = eTc(1:3,4:4);

   Rox_Double fui =      1.0/K[0][0];
   Rox_Double fvi =      1.0/K[1][1];
   Rox_Double cui = -K[0][2]/K[0][0];
   Rox_Double cvi = -K[1][2]/K[1][1];

   Rox_Double M[3][3];
   // M = R*inv(Kc);

   M[0][0] = fui*T[0][0]; M[0][1] = fvi*T[0][1]; M[0][2] = T[0][2] + cui*T[0][0] + cvi*T[0][1];
   M[1][0] = fui*T[1][0]; M[1][1] = fvi*T[1][1]; M[1][2] = T[1][2] + cui*T[1][0] + cvi*T[1][1];
   M[2][0] = fui*T[2][0]; M[2][1] = fvi*T[2][1]; M[2][2] = T[2][2] + cui*T[2][0] + cvi*T[2][1];

   Rox_Double c11 = (T[2][3]*M[0][0]-T[0][3]*M[2][0])/a;
   Rox_Double c12 = (T[2][3]*M[0][1]-T[0][3]*M[2][1])/a;
   Rox_Double c13 = (T[2][3]*M[0][2]-T[0][3]*M[2][2])/a;

   Rox_Double c21 = (T[2][3]*M[1][0]-T[1][3]*M[2][0])/b;
   Rox_Double c22 = (T[2][3]*M[1][1]-T[1][3]*M[2][1])/b;
   Rox_Double c23 = (T[2][3]*M[1][2]-T[1][3]*M[2][2])/b;

   Rox_Double c31 = M[2][0];
   Rox_Double c32 = M[2][1];
   Rox_Double c33 = M[2][2];

   Rox_Double sxx = (c11*c11+c21*c21-c31*c31);
   Rox_Double syy = (c12*c12+c22*c22-c32*c32);
   Rox_Double sxy = (c11*c12+c21*c22-c31*c32);

   Rox_Double sx = (c11*c13+c21*c23-c31*c33);
   Rox_Double sy = (c12*c13+c22*c23-c32*c33);
   Rox_Double sp = (c13*c13+c23*c23-c33*c33);

   //Ae = [sxx, sxy; sxy, syy];
   //be = [sx ; sy];
   
   // pc = -inv(Ae)*be;

   Rox_Double d  = ( sxx*syy - sxy*sxy ) ;  
   Rox_Double uc = (sxy * sy - sx * syy) / d;
   Rox_Double vc = (sx * sxy - sxx * sy) / d;

   Rox_Double k = 1.0/(sxx*uc*uc + syy*vc*vc + 2.0*sxy*uc*vc - sp);

   // ellipse in pixel coordinates

   Rox_Double nxx = sxx*k;
   Rox_Double nyy = syy*k;
   Rox_Double nxy = sxy*k;

   Rox_Double delta = nxx*nxx - 2 * nxx*nyy + 4 * nxy*nxy + nyy*nyy;
   if (delta < 0.0)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if ((nxx / 2 + nyy / 2 - sqrt(delta) / 2 < 0) || (nxx / 2 + nyy / 2 + sqrt(delta) / 2 < 0))
   { 
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   ellipse2d->xc = uc;
   ellipse2d->yc = vc;
   ellipse2d->nxx = nxx; 
   ellipse2d->nyy = nyy; 
   ellipse2d->nxy = nxy; 

function_terminate:

   rox_matse3_del(&cTe);
   rox_matse3_del(&eTc);

   return error;
}