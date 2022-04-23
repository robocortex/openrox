//==============================================================================
//
//    OPENROX   : File matsl3.h
//
//    Contents  : API of matsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MATSL3__
#define __OPENROX_MATSL3__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Group
//! \addtogroup MatSL3
//! \brief Matrix Lie Group SL3 : unit determinant matrices of size 3x3 such that det(H) = 1
//! @{

//! Define the Rox_MatSL3 object 
typedef struct _Rox_Array2D_Double * Rox_MatSL3;

//! Create a Rox_MatSL3 object and initialize it to the identity
//! \param  [out]  matsl3         The object to create
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matsl3_new (
   Rox_MatSL3 * matsl3 
);

//! Delete a Rox_MatSL3 object
//! \param  [out]  matsl3 		    The object to delete
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matsl3_del ( 
   Rox_MatSL3 * matsl3 
);

//! Copy a Rox_MatSL3 object
//! \param  [out]  result 		    The matsl3 in which copy the input
//! \param  [in ]  input 			 The matsl3 to be copied
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matsl3_copy ( 
   Rox_MatSL3 result, 
   const Rox_MatSL3 input
);

//! Set a Rox_MatSL3 object to the identity
//! \param  [out]  matsl3         The object to set
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matsl3_set_unit (
   Rox_MatSL3 matsl3
);

//! Multiply two Rox_MatSL3 matrices
//! \param  [out]  result         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matsl3_mulmatmat (
   Rox_MatSL3 result, 
   const Rox_MatSL3 input_1, 
   const Rox_MatSL3 input_2
);

//! Error between two Rox_MatSL3 matrices : result = input_1 * inv(input_2)
//! \param  [out]  matsl3         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_mulmatinv (
   Rox_MatSL3 matsl3, 
   const Rox_MatSL3 input_1, 
   const Rox_MatSL3 input_2
);

//! Compute the inverse of a Rox_MatSL3 matrix
//! \param  [out] result          The inversion result
//! \param  [in ]  input          The Rox_MatSL3 to inverse
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_inv (
   Rox_MatSL3 result, 
   const Rox_MatSL3 input
);

//! Set matrix data organized row by row
//! \param  [out]  matsl3         The object to set
//! \param  [in ]  data           Data organized by rows
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_set_data ( 
   Rox_MatSL3 matsl3,
   const Rox_Double data[9]
);

//! Get matrix data organized row by row
//! \param  [out] data            Data organized by rows
//! \param  [in ]  matsl3         The object to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_get_data(
   Rox_Double data[9], 
   const Rox_MatSL3 matsl3
);

//! Update the matsl3 to the right given a 8 rows vector
//! \param  [out]  matsl3         The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 8x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_update_right(
   Rox_MatSL3 matsl3, 
   const Rox_Array2D_Double vector
);

//! Update the matsl3 to the left given a 8 rows vector in algsl3
//! \param  [out]  matsl3         The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 8x1 vector in algsl3
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_update_left(
   Rox_MatSL3 matsl3, 
   const Rox_Array2D_Double vector
);

//! Get the pointers to internal data (need kwoledge of internal data storage organization, be careful)
//! \param  [out]  vector         The input 9x1 vector
//! \param  [in ]  matsl3         The input matsl3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_get_data_pointer_to_pointer(
   Rox_Double *** rowsptr, 
   const Rox_MatSL3 matsl3
);

//! Display the matsl3 on the stdout stream
//! \param  [in ]  matsl3         The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_print(
   const Rox_MatSL3 matsl3
);

//! Compute the SL3 matrix exponential
//! \param [out]  dest            The exponential
//! \param [in ]  input           The input array
//! \return An error code
//! \todo   To be tested, move in matsl3 module
ROX_API Rox_ErrorCode rox_array2d_double_expmat_sl3(
   Rox_Array2D_Double dest, 
   const Rox_Array2D_Double input
);


//! Check that the size of the input MatSL3 matrix is 3x3
//! \param  [in ]  rotation         The input MatSL3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_check_size ( 
   const Rox_MatSL3 matsl3 
);


//! @}

#endif // __OPENROX_MATSL3__
