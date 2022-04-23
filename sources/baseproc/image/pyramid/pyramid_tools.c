//============================================================================
//
//    OPENROX   : File pyramid_tools.c
//
//    Contents  : Implementation of pyramid_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "pyramid_tools.h"

#include <baseproc/image/convolve/basic_convolve.h>
#include <baseproc/image/convolve/array2d_float_symmetric_separable_convolve.h>
#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pyramid_compute_optimal_level_count(
  Rox_Uint * bestsize,
  const Rox_Sint  originalcols,
  const Rox_Sint  originalrows,
  const Rox_Uint  minimalsize )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint minsizeori = ROX_MIN( originalcols, originalrows );

   // Here we consider a pyramid where each level size is half the size of the previous level
   // size( l ) = size( l-1 ) / 2

   // If the pyramid has k levels the last level size is :
   // size( k ) = size( k-1 ) / 2
   // size( k ) = size( 0 ) / ( 2*2*2*...*2 [k times] )
   // size( k ) = size( 0 ) / ( 2^k )
   // size( k ) = size( 0 ) * 2^( -k )

   // if size( k ) and size( 0 ) are given, what is k ?
   // 2^( -k ) = size( k ) / size( 0 )
   // k = -ln( size( k )/size( 0 ) )/ln( 2 )
   // k = -( ln( size( k ) )-ln( size( 0 ) )/ln( 2 )
   // k = ( ln( size( 0 ) - ln( size( k ) )/ln( 2 )

   Rox_Double optimk = log( ( Rox_Double )minsizeori ) - log( ( Rox_Double )minimalsize );
   optimk = optimk / log( 2.0 );
   
   if ( optimk < DBL_EPSILON ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // We aim to find an integer value ! if minsizeori and minimalsize are not both power of 2, optimk will be a real value
   // Truncate optimk toward zero, the true size of the last level will be more than the requested minimal size, but at least not less

   *bestsize = ( Rox_Uint ) optimk;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_build_scale_space_new(
  Rox_Array2D_Float_Collection *output_scalespace,
  Rox_Array2D_Float             source,
  Rox_Uint                      nblevels,
  Rox_Float                     sigma,
  Rox_Float                     cutoff )
{
   Rox_ErrorCode                error = ROX_ERROR_NONE;
   Rox_Array2D_Float_Collection scale_space;
   Rox_Sint                     cols,       rows;
   Rox_Array2D_Float            one,         two;
   Rox_Array2D_Float            hfilter,     vfilter;

   Rox_Float k, sigma_prev, sigma_total, cur_sigma;

   if ( !output_scalespace )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !source )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   hfilter = NULL;
   vfilter = NULL;

   error = rox_array2d_float_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_collection_new( &scale_space, nblevels + 3, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   one   = rox_array2d_float_collection_get( scale_space, 0 );
   error = rox_array2d_float_copy( one, source );
   ROX_ERROR_CHECK_TERMINATE(error)

   two = one;
   k   = (Rox_Float) pow( 2.0, 1.0 / nblevels );

   for (Rox_Uint i = 1; i < nblevels + 3; i++ )
   {
      one = two;
      two = rox_array2d_float_collection_get( scale_space, i );

      // See original lindeberg article for details on scale space computation
      sigma_prev  = (Rox_Float) (pow( ( Rox_Double )k, ( int )i - 1 ) * sigma);
      sigma_total = sigma_prev * k;
      cur_sigma   = (Rox_Float) sqrt( sigma_total * sigma_total - sigma_prev * sigma_prev );

      // Generate kernels
      error = rox_kernelgen_gaussian2d_separable_float_new( &hfilter, &vfilter, cur_sigma, cutoff );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Convolve both directions using the symmetric horizontal kernel, assuming the vertical is its transposed
      error = rox_array2d_float_symmetric_seperable_convolve( two, one, hfilter );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Delete kernels
      error = rox_array2d_float_del(&hfilter);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_del(&vfilter);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *output_scalespace = scale_space;

function_terminate:
   if (error)
   {
      ROX_ERROR_WARNING( rox_array2d_float_collection_del( &scale_space ) );
      ROX_ERROR_WARNING( rox_array2d_float_del( &hfilter )                );
      ROX_ERROR_WARNING( rox_array2d_float_del( &vfilter )                );
   }
   return error;
}
