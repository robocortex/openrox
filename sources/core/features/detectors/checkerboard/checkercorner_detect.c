//==============================================================================
//
//    OPENROX   : File checkercorner_detect.c
//
//    Contents  : Implementation of checkercorner_detect module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "checkercorner_detect.h"

#include <generated/objset_dynvec_sparse_value_struct.h>
#include <generated/dynvec_sparse_value_struct.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/image/convolve/sparse_convolve.h>
#include <baseproc/image/image.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/nonlin/polynomials.h>

#include <inout/system/print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

#include <baseproc/maths/maths_macros.h>

Rox_ErrorCode rox_checkercorner_detector_new (
   Rox_CheckerCorner_Detector * checkercorner_detector,
   Rox_Sint                     cols,
   Rox_Sint                     rows,
   Rox_Uint                     kernel_blur_levels,
   Rox_Uint                    score_blur_levels )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerCorner_Detector ret = NULL;

   if ( !checkercorner_detector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   *checkercorner_detector = NULL;

   ret = ( Rox_CheckerCorner_Detector ) rox_memory_allocate( sizeof( *ret ), 1 );
   if ( !ret )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Init all pointers to NULL
   ret->source        = NULL;
   ret->gx            = NULL;
   ret->gy            = NULL;
   ret->kernels       = NULL;
   ret->rep1          = NULL;
   ret->rep2          = NULL;
   ret->rep3          = NULL;
   ret->rep4          = NULL;
   ret->cornerness    = NULL;
   ret->local_corners = NULL;
   ret->corners       = NULL;
   ret->angles        = NULL;
   ret->magnitudes    = NULL;

   error = rox_array2d_float_new( &ret->source, rows, cols );      ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->cornerness, rows, cols );  ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->gx, rows, cols );          ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->gy, rows, cols );          ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_double_new( &ret->angles, rows, cols );     ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_double_new( &ret->magnitudes, rows, cols ); ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->rep1, rows, cols );        ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->rep2, rows, cols );        ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->rep3, rows, cols );        ROX_ERROR_CHECK_TERMINATE( error );
   error = rox_array2d_float_new( &ret->rep4, rows, cols );        ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_objset_dynvec_sparse_value_new( &ret->kernels, 16 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_dynvec_checkercorner_new( &ret->local_corners, 10 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_dynvec_checkercorner_new( &ret->corners, 10 );
   ROX_ERROR_CHECK_TERMINATE( error );

   ret->kernel_blur_levels = 1; // Default value
   if ( ( kernel_blur_levels >= 1 ) && ( kernel_blur_levels <= 8 ) )
   {
      // The higher is the max the slower is the algorithm but more robust to corners which are not sharp
      ret->kernel_blur_levels = kernel_blur_levels;
   }

   ret->score_blur_levels = 1; // Default value
   if ( ( score_blur_levels >= 1 ) && ( score_blur_levels <= 8 ) )
   {
      // The higher is the max the slower is the algorithm but more robust to corners which are not sharp
      ret->score_blur_levels = score_blur_levels;
   }

   error = rox_checkercorner_build_kernels(ret);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *checkercorner_detector = ret;

function_terminate:
   if (error) rox_checkercorner_detector_del(&ret);
   return error;
}

Rox_ErrorCode rox_checkercorner_detector_del( Rox_CheckerCorner_Detector * checkercorner_detector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CheckerCorner_Detector todel = NULL;


   if (!checkercorner_detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *checkercorner_detector;
   *checkercorner_detector = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_checkercorner_del( &todel->local_corners );
   rox_dynvec_checkercorner_del( &todel->corners );
   rox_objset_dynvec_sparse_value_del( &todel->kernels );

   rox_array2d_float_del( &todel->rep1 );
   rox_array2d_float_del( &todel->rep2 );
   rox_array2d_float_del( &todel->rep3 );
   rox_array2d_float_del( &todel->rep4 );
   rox_array2d_float_del( &todel->source );
   rox_array2d_double_del( &todel->angles );
   rox_array2d_double_del( &todel->magnitudes );
   rox_array2d_float_del( &todel->cornerness );
   rox_array2d_float_del( &todel->gx );
   rox_array2d_float_del( &todel->gy );

   rox_memory_delete( todel );

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkercorner_build_kernels( Rox_CheckerCorner_Detector checkercorner_detector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Sparse_Value kernel1, kernel2, kernel3, kernel4;
   Rox_Uint levels;
   Rox_Sparse_Value_Struct sval;
   Rox_Double n1u, n1v, n2u, n2v, sum1, sum2, sum3, sum4;
   Rox_Double posu, posv, dist, dp1, dp2,sigma, var,pdf;
   Rox_Sint middle, size;
   Rox_Uint id;

   const Rox_Float radiuses[] = {4, 8, 12, 16, 20, 24, 28, 32};

   const Rox_Float vec1[] = {0, ( Rox_Float )ROX_PI_4};
   const Rox_Float vec2[] = { (Rox_Float) ROX_PI_2, (Rox_Float) - ROX_PI_4};

   if (!checkercorner_detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   levels = checkercorner_detector->kernel_blur_levels;

   for ( Rox_Uint curradius = 0; curradius < levels; curradius++ )
   {
      size   = (Rox_Sint) (radiuses[curradius] * 2 + 1);
      middle = (Rox_Sint) radiuses[curradius];
      sigma  = (Rox_Double) (radiuses[curradius] / 2.0);
      var    = sigma * sigma;

      for ( Rox_Uint curvec = 0; curvec < 2; curvec++ )
      {
         kernel1 = NULL;
         kernel2 = NULL;
         kernel3 = NULL;
         kernel4 = NULL;

         error = rox_dynvec_sparse_value_new( &kernel1, 10 );
         ROX_ERROR_CHECK_TERMINATE( error )

         error = rox_dynvec_sparse_value_new( &kernel2, 10 );
         ROX_ERROR_CHECK_TERMINATE( error )

         error = rox_dynvec_sparse_value_new( &kernel3, 10 );
         ROX_ERROR_CHECK_TERMINATE( error )

         error = rox_dynvec_sparse_value_new( &kernel4, 10 );
         ROX_ERROR_CHECK_TERMINATE( error )

         n1u = -sin( vec1[curvec] ); n1v = cos( vec1[curvec] );
         n2u = -sin( vec2[curvec] ); n2v = cos( vec2[curvec] );

         sum1 = 0;
         sum2 = 0;
         sum3 = 0;
         sum4 = 0;

         for ( Rox_Sint v = 0; v < size; v++ )
         {
            for ( Rox_Sint u = 0; u < size; u++ )
            {
               posu = u - middle;
               posv = v - middle;
               dist = sqrt( posu * posu + posv * posv );

               dp1 = posu * n1u + posv * n1v;
               dp2 = posu * n2u + posv * n2v;

               pdf = ( 1.0 / sqrt( 2.0*ROX_PI*var ) )* exp( -( dist * dist ) / ( 2.0 * var ) );

               // Fill kernel
               sval.u = (Rox_Uint) posu;
               sval.v = (Rox_Uint) posv;

               if ( dp1 <= -0.1 && dp2 <= -0.1 )
               {
                  sval.value = pdf;
                  sum1 += sval.value;
                  rox_dynvec_sparse_value_append( kernel1, &sval );
               }
               else if ( dp1 >= 0.1 && dp2 >= 0.1 )
               {
                  sval.value = pdf;
                  sum2 += sval.value;
                  rox_dynvec_sparse_value_append( kernel2, &sval );
               }
               else if ( dp1 <= -0.1 && dp2 >= 0.1 )
               {
                  sval.value = pdf;
                  sum3 += sval.value;
                  rox_dynvec_sparse_value_append( kernel3, &sval );
               }
               else if ( dp1 >= 0.1 && dp2 <= -0.1 )
               {
                  sval.value = pdf;
                  sum4 += sval.value;
                  rox_dynvec_sparse_value_append( kernel4, &sval );
               }
            }
         }

         for ( id = 0; id < kernel1->used; id++ )
         {
            kernel1->data[id].value /= sum1;
         }

         for ( id = 0; id < kernel2->used; id++ )
         {
            kernel2->data[id].value /= sum1;
         }

         for ( id = 0; id < kernel3->used; id++ )
         {
            kernel3->data[id].value /= sum1;
         }

         for ( id = 0; id < kernel4->used; id++ )
         {
            kernel4->data[id].value /= sum1;
         }

         rox_objset_dynvec_sparse_value_append( checkercorner_detector->kernels, kernel1 );
         rox_objset_dynvec_sparse_value_append( checkercorner_detector->kernels, kernel2 );
         rox_objset_dynvec_sparse_value_append( checkercorner_detector->kernels, kernel3 );
         rox_objset_dynvec_sparse_value_append( checkercorner_detector->kernels, kernel4 );
      }
   }

function_terminate:
   if ( error )
   {
      rox_dynvec_sparse_value_del( &kernel1 );
      rox_dynvec_sparse_value_del( &kernel2 );
      rox_dynvec_sparse_value_del( &kernel3 );
      rox_dynvec_sparse_value_del( &kernel4 );
   }

   return error;
}

Rox_ErrorCode rox_checkercorner_detector_refine_position (
   Rox_CheckerCorner_Detector checkercorner_detector,
   Rox_CheckerCorner corner
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint size_u = 40;
   Rox_Uint size_v = 40;
   Rox_Double A[2][2], Ainv[2][2], B[2];
   Rox_Double norm, gx, gy, w1, w2, v1, v2, d1, d2, det, nx, ny, dx, dy, dist, t1,t2,t3,t4;

   if ( !checkercorner_detector || !corner )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Float ** du = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &du, checkercorner_detector->gx );
   ROX_ERROR_CHECK_TERMINATE( error );
   Rox_Float ** dv = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dv, checkercorner_detector->gy );
   ROX_ERROR_CHECK_TERMINATE( error );

   A[0][0] = 0;
   A[0][1] = 0;
   A[1][0] = 0;
   A[1][1] = 0;

   B[0] = 0;
   B[1] = 0;

   for (Rox_Uint i = (Rox_Uint)(corner->coords.v - size_v); i <= (Rox_Uint) corner->coords.v + size_v; i++ )
   {
      for (Rox_Uint j = (Rox_Uint) (corner->coords.u - size_u); j <= (Rox_Uint) corner->coords.u + size_u; j++ )
      {
         if ( i == corner->coords.v && j == corner->coords.u ) continue;

         gx = du[i][j];
         gy = dv[i][j];
         norm = sqrt( gx*gx+gy*gy );

         if ( norm < 0.1 ) continue;
         gx /= norm;
         gy /= norm;

         w1 = j - corner->coords.u;
         w2 = i - corner->coords.v;
         t1 = corner->edge1.u*corner->edge1.u; t2 = corner->edge1.u*corner->edge1.v;
         t3 = corner->edge1.v*corner->edge1.u; t4 = corner->edge1.v*corner->edge1.v;
         v1 = w1 - ( w1 * t1 + w2 * t3 );
         v2 = w2 - ( w1 * t2 + w2 * t4 );
         d1 = sqrt( v1*v1+v2*v2 );

         w1 = j - corner->coords.u;
         w2 = i - corner->coords.v;
         t1 = corner->edge2.u*corner->edge2.u; t2 = corner->edge2.u*corner->edge2.v;
         t3 = corner->edge2.v*corner->edge2.u; t4 = corner->edge2.v*corner->edge2.v;
         v1 = w1 - ( w1 * t1 + w2 * t3 );
         v2 = w2 - ( w1 * t2 + w2 * t4 );
         d2 = sqrt( v1*v1+v2*v2 );

         if ( ( d1 < 3.0 && fabs( corner->edge1.u * gx + corner->edge1.v * gy ) < 0.25 ) || ( d2 < 3.0 && fabs( corner->edge2.u * gx + corner->edge2.v * gy ) < 0.25 ) )
         {
            gx = du[i][j];
            gy = dv[i][j];

            A[0][0] += gx*gx; A[0][1] += gx*gy;
            A[1][0] += gy*gx; A[1][1] += gy*gy;
            B[0] += gx*gx * ( Rox_Double )j + gx*gy * ( Rox_Double )i;
            B[1] += gx*gy * ( Rox_Double )j + gy*gy * ( Rox_Double )i;
         }
      }
   }

   det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
   if ( fabs( det ) < DBL_EPSILON ) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; if ( error ) goto function_terminate;} // ROX_ERROR_CHECK_TERMINATE( error )}

   Ainv[0][0] = A[1][1] / det;
   Ainv[0][1] = -A[1][0] / det;
   Ainv[1][0] = -A[0][1] / det;
   Ainv[1][1] = A[0][0] / det;

   nx = Ainv[0][0] * B[0] + Ainv[0][1] * B[1];
   ny = Ainv[1][0] * B[0] + Ainv[1][1] * B[1];
   dx = nx - corner->coords.u;
   dy = ny - corner->coords.v;
   dist = sqrt( dx*dx+dy*dy );
   if ( dist >= 4.0 ) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; if ( error ) goto function_terminate;} // ROX_ERROR_CHECK_TERMINATE( error )}

   corner->coords.u = nx;
   corner->coords.v = ny;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkercorner_detector_refine_orientation (
   Rox_CheckerCorner_Detector checkercorner_detector,
   Rox_CheckerCorner corner )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint size_u = 40;
   Rox_Uint size_v = 40;
   Rox_Uint i,j, smallest;
   Rox_Double norm, gx,gy,a,b,c,trace,det,nv,t11,t12,t21,t22;
   Rox_Double coeffs[3];
   Rox_Complex_Struct roots[2];
   Rox_Double A1[2][2], A2[2][2];


   if (!checkercorner_detector || !corner) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float ** du = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&du, checkercorner_detector->gx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dv = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dv, checkercorner_detector->gy);
   ROX_ERROR_CHECK_TERMINATE ( error );

   A1[0][0] = 0;
   A1[0][1] = 0;
   A1[1][0] = 0;
   A1[1][1] = 0;

   A2[0][0] = 0;
   A2[0][1] = 0;
   A2[1][0] = 0;
   A2[1][1] = 0;

   // Refine corner orientation
   for ( i = (Rox_Uint) corner->coords.v - size_v; i <= (Rox_Uint) corner->coords.v + size_v; i++ )
   {
      for ( j = (Rox_Uint) corner->coords.u - size_u; j <= (Rox_Uint) corner->coords.u + size_u; j++ )
      {
         gx = du[i][j];
         gy = dv[i][j];
         norm = sqrt( gx*gx+gy*gy );
         if ( norm < DBL_EPSILON ) continue;
         gx /= norm;
         gy /= norm;

         if ( fabs( corner->edge1.u * gx + corner->edge1.v * gy ) < 0.25 )
         {
            gx = du[i][j];
            gy = dv[i][j];

            A1[0][0] += gx * gx;
            A1[0][1] += gx * gy;
            A1[1][0] += gy * gx;
            A1[1][1] += gy * gy;
         }

         if ( fabs( corner->edge2.u * gx + corner->edge2.v * gy ) < 0.25 )
         {
            gx = du[i][j];
            gy = dv[i][j];

            A2[0][0] += gx * gx;
            A2[0][1] += gx * gy;
            A2[1][0] += gy * gx;
            A2[1][1] += gy * gy;
         }
      }
   }

   // Update First orientation
   a = A1[0][0];
   b = A1[0][1];
   c = A1[1][1];

   trace = a+c;
   det = a*c - b*b;

   coeffs[0] = 1.0;
   coeffs[1] = -trace;
   coeffs[2] = det;

   error = rox_quadratic_roots(roots, coeffs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   smallest = 0;
   if ( fabs( roots[0].imag ) > DBL_EPSILON )
   {

      if ( fabs( roots[1].imag ) > DBL_EPSILON ) 
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      smallest = 1;
   }
   else
   {

      if ( fabs( roots[1].imag ) > DBL_EPSILON ) 
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      if ( roots[1].real < roots[0].real ) smallest = 1;
   }

   if ( fabs( b ) < DBL_EPSILON )
   {
      nv = 0;
   }
   else
   {
      nv = -( c - roots[smallest].real ) / b;
   }

   norm = sqrt( 1.0 + nv * nv );

   if ( norm < DBL_EPSILON ) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   corner->edge1.u = 1.0 / norm;
   corner->edge1.v = nv / norm;
   t11 = roots[smallest].real * corner->edge1.u;
   t21 = roots[smallest].real * corner->edge1.v;
   t12 = a * corner->edge1.u + b * corner->edge1.v;
   t22 = b * corner->edge1.u + c * corner->edge1.v;
   if  ( ( t11 / t12 ) != ( t21 / t22 ) )
   {
      corner->edge1.u = nv / norm;
      corner->edge1.v = 1.0 / norm;
   }

   if  ( fabs( t11 - t12 ) > 1e-6 || fabs( t21 - t22 ) > 1e-6 )
   {
         corner->edge1.u = nv / norm;
         corner->edge1.v = 1.0 / norm;
   }

   // Update second orientation
   a = A2[0][0];
   b = A2[0][1];
   c = A2[1][1];

   trace = a+c;
   det = a*c - b*b;

   coeffs[0] = 1.0;
   coeffs[1] = -trace;
   coeffs[2] = det;

   error = rox_quadratic_roots( roots, coeffs );
   ROX_ERROR_CHECK_TERMINATE( error );

   smallest = 0;
   if ( fabs( roots[0].imag ) > DBL_EPSILON )
   {
      if ( fabs( roots[1].imag ) > DBL_EPSILON ) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE( error )}

      smallest = 1;
   }
   else
   {
      if ( fabs( roots[1].imag ) > DBL_EPSILON ) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE( error )}

      if ( roots[1].real < roots[0].real ) smallest = 1;
   }

   if ( fabs( b ) < DBL_EPSILON )
   {
      nv = 0;
   }
   else
   {
      nv = -( c - roots[smallest].real ) / b;
   }

   norm = sqrt( 1.0 + nv * nv );
   if ( norm < DBL_EPSILON )
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE( error ); }

   corner->edge2.u = 1.0 / norm;
   corner->edge2.v = nv / norm;
   t11 = roots[smallest].real * corner->edge2.u;
   t21 = roots[smallest].real * corner->edge2.v;
   t12 = a * corner->edge2.u + b * corner->edge2.v;
   t22 = b * corner->edge2.u + c * corner->edge2.v;

   if  ( fabs( t11 - t12 ) > 1e-6 || fabs( t21 - t22 ) > 1e-6 )
   {
         corner->edge2.u = nv / norm;
         corner->edge2.v = 1.0 / norm;
   }

   //t11 = roots[smallest].real * corner->edge2.u;
   //t21 = roots[smallest].real * corner->edge2.v;
   //t12 = a * corner->edge2.u + b * corner->edge2.v;
   //t22 = b * corner->edge2.u + c * corner->edge2.v;

function_terminate:
   return error;
}


Rox_ErrorCode rox_checkercorder_detector_score (
   Rox_Double * score,
   Rox_CheckerCorner_Detector checkercorner_detector,
   Rox_CheckerCorner corner
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint curradius;
   Rox_Sint size, middle, sigma, var;
   Rox_Sint cols, rows, posu, posv;
   Rox_Double sum1, sum2, sum3, sum4, dist, dp1, dp2, pdf, n1u, n1v, n2u, n2v;
   Rox_Double s1,s2,s3, s4,sa,sb,r1,r2,t1,t2,t3,t4,v1,v2, n1,n2,du,dv;
   Rox_Double score1, score2, scoreradius, sum_filter, sum_w, mean_filter, mean_w, stdf, stdw;
   Rox_Sint v, u;
   Rox_Double a1, a2, mean;
   Rox_Float ** dsrc, **df;
   Rox_Double  **dw;
   Rox_Array2D_Float filter = NULL;
   Rox_Uint levels = 0;

   Rox_Sint cu, cv, pu, pv;

   double sum = 0.0;

   const Rox_Float radiuses[] = {4, 8, 12, 16, 20, 24, 28, 32};

   if ( !score || !checkercorner_detector || !corner )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   cv = (Rox_Sint) (corner->coords.v + 0.5);
   cu = (Rox_Sint) (corner->coords.u + 0.5);

   error = rox_array2d_float_get_size( &rows, &cols, checkercorner_detector->source );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dsrc, checkercorner_detector->source );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dw, checkercorner_detector->magnitudes );
   ROX_ERROR_CHECK_TERMINATE( error );

   a1 = atan2( corner->edge1.v, corner->edge1.u );
   a2 = atan2( corner->edge2.v, corner->edge2.u );
   n1u = -sin( a1 );
   n1v = cos( a1 );
   n2u = -sin( a2 );
   n2v = cos( a2 );

   *score = 0;

   levels = checkercorner_detector->score_blur_levels;

   for ( curradius = 0; curradius < levels; curradius++ )
   {
      size = (Rox_Sint)(radiuses[curradius] * 2 + 1);
      middle = (Rox_Sint)radiuses[curradius];
      sigma = (Rox_Sint)(radiuses[curradius] / 2.0);
      var = sigma * sigma;

      error = rox_array2d_float_new( &filter, size, size );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_array2d_float_get_data_pointer_to_pointer( &df, filter );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_array2d_float_fillval( filter, -1 );
      ROX_ERROR_CHECK_TERMINATE( error );

      sum1 = 0;
      sum2 = 0;
      sum3 = 0;
      sum4 = 0;

      s1 = 0;
      s2 = 0;
      s3 = 0;
      s4 = 0;

      if ( cv < middle ) continue;
      if ( cu < middle ) continue;

      if ( cv >= rows-middle ) continue;
      if ( cu >= cols-middle ) continue;

      // Compute the specific response to the corner filter
      sum = 0.0;
      for ( v = 0; v < size; v++ )
      {
         for ( u = 0; u < size; u++ )
         {
            posu = u - middle;
            posv = v - middle;

            pu = posu + cu;
            pv = posv + cv;

            sum += dw[( int )pv][( int )pu];

            dist = sqrt( posu * posu + posv * posv );
            dp1 = posu * n1u + posv * n1v;
            dp2 = posu * n2u + posv * n2v;

            pdf = ( 1.0 / sqrt( 2.0*ROX_PI*var ) )*exp( - ( dist * dist ) / ( 2.0 * var ) );

            // Fill kernel
            if ( dp1 <= -0.1 && dp2 <= -0.1 )
            {
               s1 += pdf * dsrc[pv][pu];
               sum1 += pdf;
            }
            else if ( dp1 >= 0.1 && dp2 >= 0.1 )
            {
               s2 += pdf * dsrc[pv][pu];
               sum2 += pdf;
            }
            else if ( dp1 <= -0.1 && dp2 >= 0.1 )
            {
               s3 += pdf * dsrc[pv][pu];
               sum3 += pdf;
            }
            else if ( dp1 >= 0.1 && dp2 <= -0.1 )
            {
               s4 += pdf * dsrc[pv][pu];
               sum4 += pdf;
            }

            t1 = corner->edge1.u*corner->edge1.u; t2 = corner->edge1.u*corner->edge1.v;
            t3 = corner->edge1.v*corner->edge1.u; t4 = corner->edge1.v*corner->edge1.v;
            v1 = ( posu * t1 + posv * t3 );
            v2 = ( posu * t2 + posv * t4 );
            du = posu - v1;
            dv = posv - v2;
            n1 = sqrt( du*du+dv*dv );

            t1 = corner->edge2.u*corner->edge2.u; t2 = corner->edge2.u*corner->edge2.v;
            t3 = corner->edge2.v*corner->edge2.u; t4 = corner->edge2.v*corner->edge2.v;
            v1 = ( posu * t1 + posv * t3 );
            v2 = ( posu * t2 + posv * t4 );
            du = posu - v1;
            dv = posv - v2;
            n2 = sqrt( du*du+dv*dv );

            if ( n1 <= 1.5 || n2 <= 1.5 )
            {
               df[v][u] += 1.0;
            }
         }
      }

      sum_filter = 0;
      sum_w = 0;
      for ( v = 0; v < size; v++ )
      {
         for ( u = 0; u < size; u++ )
         {
            posu = u - middle;
            posv = v - middle;

            pu = posu + cu;
            pv = posv + cv;

            sum_filter += df[v][u];
            sum_w += dw[pv][pu];
         }
      }

      mean_filter = sum_filter / ( size*size );
      mean_w = sum_w / ( size*size );

      sum_filter = 0;
      sum_w = 0;
      for ( v = 0; v < size; v++ )
      {
         for ( u = 0; u < size; u++ )
         {
            posu = u - middle;
            posv = v - middle;
            pu = posu + cu;
            pv = posv + cv;

            sum_filter += ( df[v][u] - mean_filter )*( df[v][u] - mean_filter );
            sum_w += ( dw[pv][pu] - mean_w ) * ( dw[pv][pu] - mean_w );
         }
      }

      stdf = sqrt( sum_filter / ( size*size - 1 ) );
      stdw = sqrt( sum_w / ( size*size - 1 ) );

      score2 = 0;
      for ( v = 0; v < size; v++ )
      {
         for ( u = 0; u < size; u++ )
         {
            posu = u - middle;
            posv = v - middle;
            pu = posu + cu;
            pv = posv + cv;

            score2 += ( ( df[v][u] - mean_filter )/stdf ) * ( ( dw[pv][pu] - mean_w )/stdw ) / ( size*size - 1 );
         }
      }

      score2 = ROX_MAX( score2, 0 );

      s1 /= sum1;
      s2 /= sum2;
      s3 /= sum3;
      s4 /= sum4;

      mean = ( s1+s2+s3+s4 )/4.0;
      sa = ROX_MIN( s1-mean, s2-mean );
      sb = ROX_MIN( mean-s3, mean-s4 );
      r1 = ROX_MIN( sa, sb );
      sa = ROX_MIN( mean - s1, mean - s2 );
      sb = ROX_MIN( s3 - mean, s4 - mean );
      r2 = ROX_MIN( sa, sb );
      score1 = ROX_MAX( 0, ROX_MAX( r1, r2 ) );

      scoreradius = score1 * score2;
      if ( scoreradius > *score )
      {
         *score = scoreradius;
      }

      rox_array2d_float_del( &filter );
   }

   error = ROX_ERROR_NONE;

function_terminate:
   if ( filter ) rox_array2d_float_del( &filter );
   return error;
}

Rox_ErrorCode rox_checkercorner_detector_processcorners ( Rox_CheckerCorner_Detector checkercorner_detector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idc;
   Rox_Uint cu, cv;
   Rox_Sint ids, idh, index, prev, next;
   Rox_Double histo[32], histosmooth[32], pdf;
   Rox_Sint mode1, mode2;
   Rox_Double maxmode1, maxmode2, angle1, angle2, deltaangle, sign;
   Rox_Sint bin;
   Rox_CheckerCorner_Struct corner;
   Rox_Point2D_Double_Struct tmp;
   Rox_Double score, swapangle;
   Rox_Uint border_u = 42;
   Rox_Uint border_v = 42;
   Rox_Uint size_u = 40;
   Rox_Uint size_v = 40;

   if ( !checkercorner_detector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size( &rows, &cols, checkercorner_detector->source );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** da = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &da, checkercorner_detector->angles );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** dm = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dm, checkercorner_detector->magnitudes );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dgx = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgx, checkercorner_detector->gx );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dgy = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgy, checkercorner_detector->gy );
   ROX_ERROR_CHECK_TERMINATE( error );

   rox_dynvec_checkercorner_reset( checkercorner_detector->corners );

   // Compute gradient vectors polar
   for ( Rox_Sint i = 0; i < rows; i++ )
   {
      for ( Rox_Sint j = 0; j < cols; j++ )
      {
         dm[i][j] = sqrt( dgx[i][j] * dgx[i][j] + dgy[i][j] * dgy[i][j] );
         da[i][j] = atan2( dgy[i][j], dgx[i][j] );

         // Constrain the angle to lie between 0 and Pi
         if ( da[i][j] < 0 ) da[i][j] += ROX_PI;
         if ( da[i][j] > ROX_PI ) da[i][j] -= ROX_PI;

         // Convert to edge direction
         da[i][j] += ROX_PI_2;
         if ( da[i][j] > ROX_PI ) da[i][j] -= ROX_PI;
      }
   }

   // Loop through edges
   for ( idc = 0; idc < checkercorner_detector->local_corners->used; idc++ )
   {
      corner = checkercorner_detector->local_corners->data[idc];
      cu = (Rox_Uint) checkercorner_detector->local_corners->data[idc].coords.u;
      cv = (Rox_Uint) checkercorner_detector->local_corners->data[idc].coords.v;

      // Check that edges are not on borders
      if ( cu < border_u || cv < border_v ) continue;
      if ( cu >= cols - border_u || cv >= rows - border_v ) continue;

      // Reset histogram
      for ( idh = 0; idh < 32; idh++ ) histo[idh] = 0;

      // Assign gradients to histogram
      for (Rox_Uint i = cv - size_v; i <= cv + size_v; i++ )
      {
         for ( Rox_Uint j = cu - size_u; j <= cu + size_u; j++ )
         {
            bin = (Rox_Sint) ROX_MIN( ROX_MAX( floor( ( da[i][j] / ROX_PI ) * 32 ), 0 ), 31 );

            histo[bin] += dm[i][j];
         }
      }

      // Smooth histogram
      for ( idh = 0; idh < 32; idh++ )
      {
         histosmooth[idh] = 0;

         for ( ids = -2; ids <= 2; ids++ )
         {
            index = idh + ids;
            if ( index < 0 ) index = 32 + index;
            if ( index >= 32 ) index = index - 32;

            pdf = ( 1.0 / sqrt( 2.0*ROX_PI ) )*exp( -( ids*ids ) / ( 2.0 ) );

            histosmooth[idh] += histo[index] * pdf;
         }
      }

      maxmode1 = 0;
      maxmode2 = 0;
      mode1 = -1;
      mode2 = -1;

      // Find the two most important peaks in histogram
      for ( idh = 0; idh < 32; idh++ )
      {
         prev = idh - 1;
         if ( prev < 0 ) prev = 32 + prev;
         next = idh + 1;
         if ( next >= 32 ) next = next - 32;

         // Is it a peak ?
         if ( histosmooth[idh] > histosmooth[prev] && histosmooth[idh] > histosmooth[next] )
         {
            // Is it an important peak ?
            if ( histosmooth[idh] > maxmode2 )
            {
               if ( histosmooth[idh] > maxmode1 )
               {
                  maxmode2 = maxmode1;
                  maxmode1 = histosmooth[idh];
                  mode2 = mode1;
                  mode1 = idh;
               }
               else
               {
                  maxmode2 = histosmooth[idh];
                  mode2 = idh;
               }
            }
         }
      }

      if ( mode1 < 0 || mode2 < 0 ) continue;

      angle1 = mode1 * ROX_PI / 32.0;
      angle2 = mode2 * ROX_PI / 32.0;
      if ( angle1 > angle2 )
      {
         swapangle = angle1;
         angle1 = angle2;
         angle2 = swapangle;
      }

      deltaangle = fabs( angle2 - angle1 );
      deltaangle = atan2( sin( deltaangle ), cos( deltaangle ) );

      // threshold on angle
      if ( deltaangle < 0.3 ) continue;

      corner.edge1.u = cos( angle1 );
      corner.edge1.v = sin( angle1 );
      corner.edge2.u = cos( angle2 );
      corner.edge2.v = sin( angle2 );

      error = rox_checkercorner_detector_refine_orientation( checkercorner_detector, &corner );
      if ( error ) continue;

      error = rox_checkercorner_detector_refine_position( checkercorner_detector, &corner );
      if ( error ) continue;

      error = rox_checkercorder_detector_score( &score, checkercorner_detector, &corner );
      if ( error ) continue;

      // threshold on score
      if ( score < 0.01 ) continue;

      if ( corner.edge1.u + corner.edge1.v < 0 )
      {
         corner.edge1.u = -corner.edge1.u;
         corner.edge1.v = -corner.edge1.v;
      }

      // Make sure right handed edges pair
      tmp.u = corner.edge1.v;
      tmp.v = -corner.edge1.u;
      sign = -ROX_SIGN( tmp.u * corner.edge2.u + tmp.v * corner.edge2.v );
      corner.edge2.u = corner.edge2.u * sign;
      corner.edge2.v = corner.edge2.v * sign;

      rox_dynvec_checkercorner_append( checkercorner_detector->corners, &corner );
   }

   // Do not delete the following error set unless you know what you are doing
   error = ROX_ERROR_NONE;

function_terminate:
   return error;
}

Rox_ErrorCode rox_checkercorner_detector_process (
   Rox_CheckerCorner_Detector checkercorner_detector,
   Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Sparse_Value * kernels = NULL;
   Rox_Float mean = 0.0, val1 = 0.0, val2 = 0.0, valc = 0.0, maxc = 0.0;
   Rox_Uint maxi = 0, maxj = 0;

   // Window size for local maxima was 3
   const Rox_Sint sizen = 10;

   if ( !checkercorner_detector || !image )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   rox_dynvec_checkercorner_reset( checkercorner_detector->local_corners );

   // Convert uchar image to float in order to compute the gradient
   error = rox_array2d_float_from_uchar( checkercorner_detector->source, image );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_float_gradientsobel_nomask( checkercorner_detector->gx, checkercorner_detector->gy, checkercorner_detector->source );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_float_from_uchar_normalize_minmax( checkercorner_detector->source, image );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dc = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dc, checkercorner_detector->cornerness );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dr1 = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dr1, checkercorner_detector->rep1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dr2 = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dr2, checkercorner_detector->rep2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dr3 = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dr3, checkercorner_detector->rep3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float ** dr4 = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dr4, checkercorner_detector->rep4 );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size( &rows, &cols, checkercorner_detector->source );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_float_fillval( checkercorner_detector->cornerness, 0 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Compute cornerness for each pixels
   for (Rox_Uint idkernel = 0; idkernel < checkercorner_detector->kernels->used; idkernel+=4 )
   {
      kernels = &checkercorner_detector->kernels->data[idkernel];
      error = rox_array2d_float_convolve_with_sparse_kernel( checkercorner_detector->rep1, checkercorner_detector->source, kernels[0] );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_array2d_float_convolve_with_sparse_kernel( checkercorner_detector->rep2, checkercorner_detector->source, kernels[1] );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_array2d_float_convolve_with_sparse_kernel( checkercorner_detector->rep3, checkercorner_detector->source, kernels[2] );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_array2d_float_convolve_with_sparse_kernel( checkercorner_detector->rep4, checkercorner_detector->source, kernels[3] );
      ROX_ERROR_CHECK_TERMINATE( error );

      for ( Rox_Sint i = 0; i < rows; i++ )
      {
         for ( Rox_Sint j = 0; j < cols; j++ )
         {
            mean = ( dr1[i][j] + dr2[i][j] + dr3[i][j] + dr4[i][j] ) / 4;

            valc = dc[i][j];

            val1 = ROX_MIN( dr1[i][j] - mean, dr2[i][j] - mean );
            val2 = ROX_MIN( mean - dr3[i][j], mean - dr4[i][j] );
            valc = ROX_MAX( valc, ROX_MIN( val1, val2 ) );
            val1 = ROX_MIN( mean - dr1[i][j], mean - dr2[i][j] );
            val2 = ROX_MIN( dr3[i][j] - mean, dr4[i][j] - mean );
            valc = ROX_MAX( valc, ROX_MIN( val1, val2 ) );

            dc[i][j] = valc;
         }
      }
   }

   // Extract local maximas
   for ( Rox_Sint i = sizen; i < rows - sizen; i++ )
   {
      for ( Rox_Sint j = sizen; j < cols - sizen; j++ )
      {
         maxi = i;
         maxj = j;
         maxc = dc[i][j];

         for ( Rox_Sint k = i - sizen; k <= i + sizen; k++ )
         {
            for ( Rox_Sint l = j - sizen; l <= j + sizen; l++ )
            {
               valc = dc[k][l];
               // Test if valc is greater OR equal to maxc
               // Test if equal avoids to duplicate points when equal
               if ( valc >= maxc )
               {
                  maxi = k;
                  maxj = l;
                  maxc = valc;
               }
            }
         }

         if ( maxi != i || maxj != j ) continue;

         // Maximum cornerness
         if ( maxc > 0.025 )
         {
            Rox_CheckerCorner_Struct corner;
            corner.coords.u = j;
            corner.coords.v = i;

            rox_dynvec_checkercorner_append( checkercorner_detector->local_corners, &corner );
         }
      }
   }

   error = rox_checkercorner_detector_processcorners ( checkercorner_detector );
   ROX_ERROR_CHECK_TERMINATE( error )

function_terminate:
   return error;
}
