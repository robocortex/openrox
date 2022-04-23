//==============================================================================
//
//    OPENROX   : File quaternion.c
//
//    Summary   : Implementation of quaternion module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quaternion.h"
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_quaternion_new( Rox_Quaternion * q )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Quaternion ret = NULL;

   // A quaternion is a 1 x 4 vector of doubles
   error = rox_array2d_double_new( &ret, 1, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Init the quaternion

   *q = ret;

function_terminate:

   return error;
}


Rox_ErrorCode rox_quaternion_copy( Rox_Quaternion q_out, const Rox_Quaternion q_inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_copy( q_out, q_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_del(Rox_Quaternion * q )
{
    return rox_array2d_double_del(q);
}

Rox_ErrorCode rox_quaternion_set_data( Rox_Quaternion q, const Rox_Double data[4] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !q || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** q_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &q_data, q );
   ROX_ERROR_CHECK_TERMINATE( error );

   q_data[0][0] = data[0]; q_data[0][1] = data[1]; q_data[0][2] = data[2]; q_data[0][3] = data[3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_set_data_float( Rox_Quaternion q, const Rox_Float data[4] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !q || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** q_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &q_data, q );
   ROX_ERROR_CHECK_TERMINATE( error );

   q_data[0][0] = data[0]; q_data[0][1] = data[1]; q_data[0][2] = data[2]; q_data[0][3] = data[3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_new_copy( Rox_Quaternion * q_out, const Rox_Quaternion q_inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_quaternion_new( q_out );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_quaternion_copy( * q_out, q_inp );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_quaternion_get_data( Rox_Double data[4], Rox_Quaternion q )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !q || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** q_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &q_data, q );
   ROX_ERROR_CHECK_TERMINATE( error );

   data[0] = q_data[0][0]; data[1] = q_data[0][1]; data[2] = q_data[0][2]; data[3] = q_data[0][3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_set_unit( Rox_Quaternion q )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !q )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** q_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &q_data, q );
   ROX_ERROR_CHECK_TERMINATE( error );

   for ( Rox_Sint i = 0; i < 3; i++ )
   {
       q_data[0][i] = 0.0;
   }
   q_data[0][3] = 1.0;  // The norm should be 1

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_print( const Rox_Quaternion q )
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!q)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

    error = rox_quaternion_print_stream( q, stdout, 3 );
    ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_print_stream( const Rox_Quaternion q, FILE* stream, Rox_Uint precision )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!q)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if ( !stream )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   const Rox_Uint size = 4;
   char format[16];

   if ( precision > 64 ) precision = 64;

#if Rox_Double == Rox_Double80
   sprintf( format,"%%.%d%c%c ",precision,'L','f' );
#else
   sprintf( format,"%%.%d%c ",precision,'f' );
#endif
    Rox_Double * q_data = NULL;
    error = rox_array2d_double_get_data_pointer( &q_data, q );

   fprintf( stream, "Quaternion ( %u ):\r\n", size );
   for ( Rox_Sint i=0 ; i < size ; ++i)
   {
      fprintf( stream, format, q_data[i] );
   }
   fprintf( stream, "\r\n\n" );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaternion_sumsquares( Rox_Double * sumsquares, const Rox_Quaternion q )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!q | !sumsquares)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double * data = NULL;
   error = rox_array2d_double_get_data_pointer( &data, q );

   Rox_Double s = ( *data ) * ( *data );
   Rox_Uint size = 4;

   while ( --size )
   {
      data++;
      s += ( *data ) * ( *data );
   }

   *sumsquares = s;

function_terminate:
    return ( error );
}

Rox_ErrorCode rox_quaternion_norm( Rox_Double * norm, const Rox_Quaternion q )
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!q | !norm)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

    Rox_Double ssq = 0.0;

    error = rox_quaternion_sumsquares(&ssq, q );
    ROX_ERROR_CHECK_TERMINATE( error );

    * norm = sqrt( ssq );

function_terminate:
    return(error);
}

Rox_ErrorCode rox_quaternion_normalize( Rox_Quaternion q )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!q)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double * q_data = NULL;
   error = rox_array2d_double_get_data_pointer( &q_data, q );

   Rox_Double s = 0.0;
   error = rox_quaternion_sumsquares(&s, q);
   ROX_ERROR_CHECK_TERMINATE( error );

   s = sqrt(s);
   for ( Rox_Sint i=0; i<4; i++)
   {
      q_data[i] /= s;
   }

function_terminate:
   return(error);
}

Rox_ErrorCode rox_quaternion_save( const Rox_Char * filename, const Rox_Quaternion q )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * stream = fopen( filename,"w" );

   if (!filename || !q)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if (!stream)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double * q_data = NULL;
   error = rox_array2d_double_get_data_pointer( &q_data, q );
   ROX_ERROR_CHECK_TERMINATE( error );

   for ( Rox_Sint  k=0 ; k < 4 ; ++k )
   {
      fprintf( stream, "%32.32f  ", q_data[k] );
   }
  fprintf( stream, "\r\n" );

function_terminate:
   if(stream) fclose( stream );
   return(error);
}

Rox_ErrorCode rox_quaternion_mul( Rox_Quaternion q_out, const Rox_Quaternion q_in1, const Rox_Quaternion q_in2 )
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!q_out || !q_in1 || !q_in2)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

    Rox_Double * qo = NULL;
    error = rox_array2d_double_get_data_pointer( &qo, q_out );
    ROX_ERROR_CHECK_TERMINATE( error );

    Rox_Double * q1 = NULL;
    error = rox_array2d_double_get_data_pointer( &q1, q_in1 );
    ROX_ERROR_CHECK_TERMINATE( error );

    Rox_Double * q2 = NULL;
    error = rox_array2d_double_get_data_pointer( &q2, q_in2 );
    ROX_ERROR_CHECK_TERMINATE( error );

    qo[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    qo[1] = q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] - q1[3]*q2[2];
    qo[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
    qo[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];

function_terminate:
    return(error);
}

Rox_ErrorCode rox_quaternion_from_matso3( Rox_Quaternion q_out, Rox_MatSO3 R_inp )
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    Rox_Double scale = 1.0, n4 = 1.0;

    if(!q_out || !R_inp)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

    Rox_Double * q_data = NULL;
    error = rox_array2d_double_get_data_pointer ( &q_data, q_out );

    Rox_Double ** R = NULL;
    error = rox_array2d_double_get_data_pointer_to_pointer( &R, R_inp );

    Rox_Double tr = R[0][0] + R[1][1] + R[2][2];

    if ( tr > 0.0f )
    {
        q_data[0] = R[1][2] - R[2][1];
        q_data[1] = R[2][0] - R[0][2];
        q_data[2] = R[0][1] - R[1][0];
        q_data[3] = tr + 1.0f;
        n4 = q_data[3];
    }
    else if ( ( R[0][0] > R[1][1] ) && ( R[0][0] > R[2][2] ) )
    {
        q_data[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
        q_data[1] = R[1][0] + R[0][1];
        q_data[2] = R[2][0] + R[0][2];
        q_data[3] = R[1][2] - R[2][1];
        n4 = q_data[0];
    }
    else if ( R[1][1] > R[2][2] )
    {
        q_data[0] = R[1][0] + R[0][1];
        q_data[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
        q_data[2] = R[2][1] + R[1][2];
        q_data[3] = R[2][0] - R[0][2];
        n4 = q_data[1];
    }
    else
    {
        q_data[0] = R[2][0] + R[0][2];
        q_data[1] = R[2][1] + R[1][2];
        q_data[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
        q_data[3] = R[0][1] - R[1][0];
        n4 = q_data[2];
    }

    scale = 0.5f / ( Rox_Double )( sqrt( n4 ) );

    q_data[0] *= scale;
    q_data[1] *= scale;
    q_data[2] *= scale;
    q_data[3] *= scale;

function_terminate:
    return(error);

}
