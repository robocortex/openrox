//==============================================================================
//
//    OPENROX   : File matrix.h
//
//    Contents  : API of matrix module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATRIX__
#define __OPENROX_MATRIX__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \addtogroup Matrix
//! @{

//! Define the Rox_Matrix object
typedef struct _Rox_Array2D_Double * Rox_Matrix;

//! Create and allocate memory for a matrix object.
//! \param  [out]  matrix         The object to create
//! \param  [in ]  rows           The matrix number of rows
//! \param  [in ]  cols           The matrix number of columns
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_new (
   Rox_Matrix * matrix, 
   const Rox_Sint rows, 
   const Rox_Sint cols
);

//! Delete a matrix object
//! \param  [in ]  matrix         The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_del(Rox_Matrix * matrix);

//! Get the matrix width
//! \param  [out]  cols           The matrix width
//! \param  [in ]  matrix         The matrix object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_get_cols (
   Rox_Sint * cols, 
   const Rox_Matrix matrix
);

//! Get the matrix height
//! \param  [out]  rows           The matrix height
//! \param  [in ]  matrix         The matrix object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_get_rows (
   Rox_Sint * rows, 
   const Rox_Matrix matrix
);

//! Get a matrix value M(i,j)
//! \param  [out]  value          The returned value
//! \param  [in ]  matrix         The matrix object
//! \param  [in ]  i              The row index
//! \param  [in ]  j              The column index
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_get_value (
   Rox_Double * value, 
   const Rox_Matrix matrix, 
   const Rox_Sint i, 
   const Rox_Sint j
);

//! Set a matrix value M(i,j)
//! \param  [in ]  matrix         The matrix object
//! \param  [in ]  i              The row index
//! \param  [in ]  j              The column index
//! \param  [in ]  value          The value to set
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_matrix_set_value (
   Rox_Matrix matrix, 
   const Rox_Sint i, 
   const Rox_Sint j, 
   const Rox_Double value
);

//! Set a matrix object to the identity
//! \param  [in ]  matrix         The object to set
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matrix_set_unit (
   Rox_Matrix matrix
);

//! Set each matrix element to zero
//! \param  [in ]  matrix         The object to set
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matrix_set_zero (
   Rox_Matrix matrix
);

//! Display the matrix on the stdout stream
//! \param  [in ]  matrix         The object to print
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matrix_print (
   const Rox_Matrix matrix
);

//! Build a calibration matrix using 4 classic parameters
//! \param  [out] matrix          The calibration matrix
//! \param  [in]  fu              The fu parameter
//! \param  [in]  fv              The fv parameter
//! \param  [in]  cu              The cu parameter
//! \param  [in]  cv              The cv parameter
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_build_calibration_matrix (
   Rox_Matrix matrix, 
   const Rox_Double fu, 
   const Rox_Double fv, 
   const Rox_Double cu, 
   const Rox_Double cv
);

//! Compute matrix_out = matrix_in1 + matrix_in2'
//! \param  [out] matrix_out        The matrix output
//! \param  [in]  matrix_in1        The matrix input 1
//! \param  [in]  matrix_in2        The object input 2
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_add_matmattrans (
   Rox_Matrix matrix_out, 
   const Rox_Matrix matrix_in1, 
   const Rox_Matrix matrix_in2
);

//! Compute matrix_power = matrix_base^exponent 
//! \param  [out]  matrix_power   The matrix power
//! \param  [in ]  matrix_base    The matrix base
//! \param  [in ]  exponent       The exponent
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_power (
   Rox_Matrix matrix_power, 
   const Rox_Matrix matrix_base, 
   const Rox_Sint exponent
);

//! Compute matrix_out = matrix_inp 
//! \param  [out]  matrix_out     The matrix power
//! \param  [in ]  matrix_inp     The matrix base
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_copy (
   Rox_Matrix matrix_out, 
   const Rox_Matrix matrix_inp
);

//! Trace of a square matrix
//! \param  [out]  trace          The trace of the matrix
//! \param  [in ]  matrix         The square matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matrix_trace (
   Rox_Double * trace, 
   const Rox_Matrix matrix
);

ROX_API Rox_ErrorCode rox_matrix_get_data_pointer_to_pointer ( 
   Rox_Double *** rowsptr, 
   const Rox_Matrix matrix 
);

ROX_API Rox_ErrorCode rox_matrix_set_data ( 
   Rox_Matrix matrix, 
   const Rox_Double * data 
);

ROX_API Rox_ErrorCode rox_matrix_get_data ( 
   Rox_Double * data, 
   const Rox_Matrix matrix 
);

//! @}

#endif // __OPENROX_MATRIX__
