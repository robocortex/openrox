//==============================================================================
//
//    OPENROX   : File matlt3.h
//
//    Contents  : API of matlt3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MATLT3__
#define __OPENROX_MATLT3__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Group
//! \addtogroup MatLT3
//! \brief Matrix Lie Group LT3 : invertible lower triangular matrices with positive diagonal entries of size 3x3 
//! @{

//! Define the Rox_MatLT3 object 
typedef struct _Rox_Array2D_Double * Rox_MatLT3;

//! Create a Rox_MatLT3 object and initialize it to the identity
//! \param  [out]  matlt3         The object to create
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matlt3_new(Rox_MatLT3 * matlt3);

//! Delete a Rox_MatLT3 object
//! \param  [out]  matlt3 		    The object to delete
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matlt3_del(Rox_MatLT3 * matlt3);

//! Copy a Rox_MatLT3 object
//! \param  [out]  result 		    The matlt3 in which copy the input
//! \param  [out]  input 			 The matlt3 to be copied
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matlt3_copy(Rox_MatLT3 result, const Rox_MatLT3 input);

//! Set a Rox_MatLT3 object to the identity
//! \param  [in ]  matlt3         The object to set
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matlt3_set_unit(Rox_MatLT3 matlt3);

//! Multiply two Rox_MatLT3 matrices
//! \param  [out]  result         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matlt3_mulmatmat(Rox_MatLT3 result, const Rox_MatLT3 input_1, const Rox_MatLT3 input_2);

//! Error between two Rox_MatLT3 matrices : result = input_1 * inv(input_2)
//! \param  [out]  matlt3         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_mulmatinv(Rox_MatLT3 matlt3, const Rox_MatLT3 input_1, const Rox_MatLT3 input_2);

//! Compute the inverse of a Rox_MatLT3 matrix
//! \param  [out]  result         The inversion result
//! \param  [in ]  input          The Rox_MatLT3 to inverse
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_inv(Rox_MatLT3 result, const Rox_MatLT3 input);

//! Set matrix data organized row by row
//! \param  [out]  matlt3         The object to set
//! \param  [in ]  data           Data organized by rows
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_set_data(Rox_MatLT3 matlt3, const Rox_Double data[9]);

//! Get matrix data organized row by row
//! \param  [out]  data           Data organized by rows
//! \param  [in ]  matlt3         The object to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_get_data(Rox_Double data[9], const Rox_MatLT3 matlt3);

//! Update the matlt3 to the right given a 8 rows vector
//! \param  [out]  matlt3         The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 8x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_update_right(Rox_MatLT3 matlt3, const Rox_Array2D_Double vector);

//! Update the matlt3 to the left given a 8 rows vector in algsl3
//! \param  [out]  matlt3         The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 8x1 vector in algsl3
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_update_left(Rox_MatLT3 matlt3, const Rox_Array2D_Double vector);

//! Get the pointers to internal data (need kwoledge of internal data storage organization, be careful)
//! \param  [out]  vector         The input 9x1 vector
//! \param  [in ]  matlt3         The input matlt3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_get_data_pointer_to_pointer (
   Rox_Double *** rowsptr, 
   const Rox_MatLT3 matlt3
);

//! Display the matlt3 on the stdout stream
//! \param  [in ]  matlt3         The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matlt3_print ( const Rox_MatLT3 matlt3 );

//! @}

#endif // __OPENROX_MATLT3__
