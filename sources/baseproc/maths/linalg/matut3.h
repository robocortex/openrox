//==============================================================================
//
//    OPENROX   : File matut3.h
//
//    Contents  : API of matut3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MATUT3__
#define __OPENROX_MATUT3__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Group
//! \addtogroup MatSL3
//! \brief Matrix Lie Group UT3 : invertible upper triangular matrices with positive diagonal entries of size 3x3 
//! @{

//! Define the Rox_MatUT3 object 
typedef struct _Rox_Array2D_Double * Rox_MatUT3;

//! Create a Rox_MatUT3 object and initialize it to the identity
//! \param  [out]  matut3         The object to create
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matut3_new ( Rox_MatUT3 * matut3);

//! Delete a Rox_MatUT3 object
//! \param  [out]  matut3 		    The object to delete
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matut3_del ( Rox_MatUT3 * matut3);

//! Copy a Rox_MatUT3 object
//! \param  [out]  result 		    The matut3 in which copy the input
//! \param  [out]  input 			 The matut3 to be copied
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matut3_copy ( Rox_MatUT3 result, const Rox_MatUT3 input );

//! Set a Rox_MatUT3 object to the identity
//! \param  [in ]  matut3         The object to set
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matut3_set_unit ( Rox_MatUT3 matut3 );

//! Multiply two Rox_MatUT3 matrices
//! \param  [out]  result         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_matut3_mulmatmat ( Rox_MatUT3 result, const Rox_MatUT3 input_1, const Rox_MatUT3 input_2 );

//! Error between two Rox_MatUT3 matrices : result = input_1 * inv(input_2)
//! \param  [out]  matut3         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_mulmatinv ( Rox_MatUT3 matut3, const Rox_MatUT3 input_1, const Rox_MatUT3 input_2 );

//! Compute the inverse of a Rox_MatUT3 matrix
//! \param  [out]  result        The inversion result
//! \param  [in ]  input         The Rox_MatUT3 to inverse
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_inv(Rox_MatUT3 result, const Rox_MatUT3 input);

//! Build a calibration matrix using 4 classic parameters
//! \param  [out] matrix          The calibration matrix
//! \param  [in]  fu              The fu parameter
//! \param  [in]  fv              The fv parameter
//! \param  [in]  cu              The cu parameter
//! \param  [in]  cv              The cv parameter
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_build_calibration_matrix(Rox_MatUT3 matut3, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv);

//! Set matrix data organized row by row
//! \param  [in ]  matut3         The object to set
//! \param  [in ]  data           Data organized by rows
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_set_data ( Rox_MatUT3 matut3, const Rox_Double data[9] );
ROX_API Rox_ErrorCode rox_matut3_set_data_float ( Rox_MatUT3 matut3, const Rox_Float data[9] );

//! Get matrix data organized row by row
//! \param  [out]  data           Data organized by rows
//! \param  [in ]  matut3         The object to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_get_data(Rox_Double data[9], const Rox_MatUT3 matut3);

//! Update the matut3 to the right given a 8 rows vector
//! \param  [out]  matut3         The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 8x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_update_right(Rox_MatUT3 matut3, const Rox_Array2D_Double vector);

//! Update the matut3 to the left given a 6 rows vector in algsl3
//! \param  [out]  matut3         The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 5x1 vector in algsl3
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_update_left ( Rox_MatUT3 matut3, const Rox_Array2D_Double vector );

//! Get the pointers to internal data (need kwoledge of internal data storage organization, be careful)
//! \param  [out]  vector         The input 9x1 vector
//! \param  [in ]  matut3         The input matut3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_get_data_pointer_to_pointer (Rox_Double *** rowsptr, const Rox_MatUT3 matut3);

//! Display the matut3 on the stdout stream
//! \param  [in ]  matut3         The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_print(Rox_MatUT3 matut3);

//! Compute the UT3 matrix exponential
//! \param  [out]  dest           The exponential
//! \param  [in ]  input          The input array
//! \return An error code, rename the function rox_matut3_exp 
ROX_API Rox_ErrorCode rox_array2d_double_expmat_ut3(Rox_MatUT3 dest, const Rox_Array2D_Double input);

//! Check that the size of the input MatUT3 matrix is 3x3
//! \param  [in ]  rotation         The input MatSO3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matut3_check_size ( const Rox_MatUT3  matut3 );

//! @}

#endif // __OPENROX_MATUT3__
