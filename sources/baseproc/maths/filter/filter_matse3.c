//==============================================================================
//
//    OPENROX   : File filter_matse3.c
//
//    Contents  : Implementation of filter matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
// 
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

//====== INCLUDED HEADERS   ===================================================

#include "filter_matse3.h"
#include "math.h"
#include <generated/array2d_double.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/transpose/transpose.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

//====== INTERNAL MACROS    ===================================================

//====== INTERNAL TYPESDEFS ===================================================

//====== INTERNAL DATATYPES ===================================================

//====== INTERNAL VARIABLES ===================================================

//====== INTERNAL FUNCTDEFS ===================================================

Rox_ErrorCode rox_filter_matse3_projector ( Rox_Matrix out, Rox_Matrix inp );

//====== INTERNAL FUNCTIONS ===================================================

Rox_ErrorCode rox_filter_matse3_projector ( Rox_Matrix out, Rox_Matrix inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Matrix Tt  = NULL;
   Rox_Matrix buf = NULL;

   Rox_Double fB1[16] = {0.0,  0.0, 0.0, 0, 0, 0, -1.0/sqrt(2.0), 0,  0, 1.0/sqrt(2.0), 0, 0, 0, 0, 0, 0};
   Rox_Double fB2[16] = {0.0,  0.0, 1.0/sqrt(2.0), 0, 0, 0,  0, 0, -1.0/sqrt(2.0), 0, 0, 0, 0, 0, 0, 0};
   Rox_Double fB3[16] = {0.0, -1.0/sqrt(2.0), 0, 0, 1.0/sqrt(2.0), 0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0};
   Rox_Double fB4[16] = {0.0,  0.0, 0.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
   Rox_Double fB5[16] = {0.0,  0.0, 0.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0};
   Rox_Double fB6[16] = {0.0,  0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0};
   
   Rox_Matrix B[6] = {NULL};
   
   error = rox_matrix_new ( &Tt, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &buf, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &B[0], 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_buffer_no_stride ( B[0], fB1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &B[1], 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_buffer_no_stride ( B[1], fB2 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &B[2], 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_buffer_no_stride ( B[2], fB3 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &B[3], 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_buffer_no_stride ( B[3], fB4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &B[4], 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_set_buffer_no_stride ( B[4], fB5 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &B[5], 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_buffer_no_stride ( B[5], fB6 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_zero ( out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < 6; i++ )
   {
      Rox_Double trace;
       
      // P = P + trace(transpose(T)*B(:,:,i))*B(:,:,i); 
      error = rox_array2d_double_transpose ( Tt, inp );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat ( buf, Tt, B[i] );
	    ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matrix_trace ( &trace, buf );
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_scale_inplace ( B[i], trace );
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_add ( out, out, B[i] );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   rox_matrix_del ( &Tt   );
   rox_matrix_del ( &buf  );
   rox_matrix_del ( &B[0] );
   rox_matrix_del ( &B[1] );
   rox_matrix_del ( &B[2] );
   rox_matrix_del ( &B[3] );
   rox_matrix_del ( &B[4] );
   rox_matrix_del ( &B[5] );

function_terminate:
   return error;
}

//====== EXPORTED FUNCTIONS ===================================================

Rox_ErrorCode rox_filter_matse3_new ( Rox_Filter_MatSE3 * filter_matse3, Rox_Double dt )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Filter_MatSE3 ret = NULL;

   if (!(dt > 0.0) )
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!filter_matse3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Filter_MatSE3) malloc(sizeof(*ret));
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_new ( &ret->Th );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matse3_new ( &ret->Te );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matse3_new ( &ret->Tp );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matse3_new ( &ret->Tc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->Te_inv );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matse3_new ( &ret->Tp_inv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->Q, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_new ( &ret->P, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_new ( &ret->T, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_new ( &ret->M, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->Wa, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_new ( &ret->Wt, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->Ah, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_set_zero(ret->Ah);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->Ap, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_set_zero(ret->Ap);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->At, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->dt = dt;

   // Default Gains
   ret->Ka = 0.01/ret->dt;
   ret->Kg = 0.1/ret->dt;

   *filter_matse3 = ret;

function_terminate:
   if (error) rox_filter_matse3_del ( &ret );
   return error ;
}

Rox_ErrorCode rox_filter_matse3_del ( Rox_Filter_MatSE3 * filter_matse3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Filter_MatSE3 todel = NULL;

   if ( !filter_matse3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *filter_matse3;
   *filter_matse3 = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del ( &todel->Th );
   rox_matse3_del ( &todel->Te );
   rox_matse3_del ( &todel->Tp );
   rox_matse3_del ( &todel->Tc );
 
   rox_matse3_del ( &todel->Te_inv );
   rox_matse3_del ( &todel->Tp_inv );
  
   rox_matrix_del ( &todel->Q );
   rox_matrix_del ( &todel->P );
   rox_matrix_del ( &todel->T );
   rox_matrix_del ( &todel->M );
  
   rox_matrix_del ( &todel->Wt );
   rox_matrix_del ( &todel->Wa );
  
   rox_matrix_del ( &todel->Ah );
   rox_matrix_del ( &todel->Ap );
   rox_matrix_del ( &todel->At );

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_filter_matse3_init_pose ( Rox_Filter_MatSE3 filter_matse3, const Rox_MatSE3 T0 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Set the current estimation
   error = rox_matse3_copy ( filter_matse3->Th, T0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make the predicition
   error = rox_matse3_copy ( filter_matse3->Tp, T0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_filter_matse3_init_velocity ( Rox_Filter_MatSE3 filter_matse3, const Rox_Matrix A0 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Set the current estimation
   error = rox_matrix_copy ( filter_matse3->Ah, A0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make the predicition

   error = rox_matrix_copy ( filter_matse3->Ap, A0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_filter_matse3_reset ( Rox_Filter_MatSE3 filter_matse3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   error = rox_matse3_set_unit ( filter_matse3->Tp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_set_unit ( filter_matse3->Th );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_set_zero ( filter_matse3->Ah );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_zero(filter_matse3->Ap);
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:
   return error;
}

Rox_ErrorCode rox_filter_matse3_update ( Rox_Filter_MatSE3 filter, const Rox_MatSE3 Tm )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Compute prediction error 
   error = rox_matse3_inv ( filter->Tp_inv, filter->Tp );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Te = inv(Tp) * Tm
   error = rox_array2d_double_mulmatmat ( filter->Te, filter->Tp_inv, Tm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Change of variable : Q = Te'*(Te - I)

   // M = I
   error = rox_matrix_set_unit ( filter->M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // M = Te - I
   error = rox_array2d_double_substract ( filter->M, filter->Te, filter->M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // T = Te'
   error = rox_array2d_double_transpose ( filter->T, filter->Te );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Q = T * M = Te' * (Te - I)
   error = rox_array2d_double_mulmatmat ( filter->Q, filter->T, filter->M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Projector : P = -projSE3(Q)
   error = rox_filter_matse3_projector ( filter->P, filter->Q );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale_inplace ( filter->P, -1.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute control gains 

   // Wa = -Ka * P
   error = rox_array2d_double_scale ( filter->Wa, filter->P, -filter->Ka); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Wt = - Ah(:,:,k-1) + Te(:,:,k) * ( Ah(:,:,k-1) - Kg * P )  * inv(Te(:,:,k));
   
   // Temporary matrix : M = -Kg * P
   error = rox_array2d_double_scale ( filter->M, filter->P, -filter->Kg);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Temporary matrix : M = -Kg * P + Ah(:,:,k-1)
   error = rox_array2d_double_add ( filter->M, filter->M, filter->Ah );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Temporary matrix : T = Te * ( M ) * inv ( Te )
   error = rox_algse3_adjoint_matse3 ( filter->T, filter->Te, filter->M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Temporary matrix : T = Te * ( M ) * inv ( Te ) - Ah(:,:,k-1)
   error = rox_array2d_double_substract ( filter->Wt, filter->T, filter->Ah );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Integration / Correction 
   error = rox_array2d_double_add ( filter->Wt, filter->Wt, filter->Ah );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale_inplace ( filter->Wt, filter->dt );
   ROX_ERROR_CHECK_TERMINATE ( error );   

   error = rox_matse3_exponential_algse3 ( filter->Tc, filter->Wt );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add ( filter->Ah, filter->Ap, filter->Wa );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Th(:,:,k) = Th(:,:,k-1) * Tc
   error = rox_array2d_double_copy ( filter->T, filter->Th );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat ( filter->Th, filter->T, filter->Tc );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Prediction

   // Ap = Ah
   error = rox_matrix_copy ( filter->Ap, filter->Ah );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // At = Ah * dt
   error = rox_array2d_double_scale ( filter->At, filter->Ah, filter->dt );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Tc = expm ( At )
   error = rox_matse3_exponential_algse3 ( filter->Tc, filter->At );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Tp = Th * Tc
   error = rox_array2d_double_mulmatmat ( filter->Tp, filter->Th, filter->Tc );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
} 

Rox_ErrorCode rox_filter_matse3_get_estimated_pose ( Rox_MatSE3 Th, const Rox_Filter_MatSE3 filter_matse3 )
{
   return rox_matse3_copy ( Th, filter_matse3->Th );
}

Rox_ErrorCode rox_filter_matse3_get_predicted_pose ( Rox_MatSE3 Tp, const Rox_Filter_MatSE3 filter_matse3 )
{
   return rox_matse3_copy ( Tp, filter_matse3->Tp );
}

Rox_ErrorCode rox_filter_matse3_get_estimated_velocity ( Rox_Matrix Ah, const Rox_Filter_MatSE3 filter_matse3 )
{
   return rox_matrix_copy(Ah, filter_matse3->Ah);
}

Rox_ErrorCode rox_filter_matse3_get_predicted_velocity ( Rox_Matrix Ap, const Rox_Filter_MatSE3 filter_matse3 )
{
   return rox_matrix_copy(Ap, filter_matse3->Ap);
}

Rox_ErrorCode rox_filter_matse3_set_gain_pose ( Rox_Filter_MatSE3 filter_matse3, const Rox_Double Kg )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filter_matse3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   filter_matse3->Kg = Kg;

function_terminate:
   return error;
}

Rox_ErrorCode rox_filter_matse3_set_gain_velocity ( Rox_Filter_MatSE3 filter_matse3, const Rox_Double Ka )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filter_matse3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   filter_matse3->Ka = Ka;

function_terminate:
   return error;
}
