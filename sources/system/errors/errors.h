//==============================================================================
//
//    OPENROX   : File errors.h
//
//    Contents  : API of errors module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ERRORS__
#define __OPENROX_ERRORS__

// Definition of errors which will be returned by functions upon malfunction

// No error
#define ROX_ERROR_NONE                          0

// A null pointer was passed to the function
#define ROX_ERROR_NULL_POINTER                  1

// I/O bad descriptor
#define ROX_ERROR_BAD_IOSTREAM                  2

// The data type is not valid in this context
#define ROX_ERROR_BAD_TYPE                      3

// Size (Dimension) of parameters is invalid
#define ROX_ERROR_BAD_SIZE                      4

// Two arrays are not of the same size (but they should)
#define ROX_ERROR_ARRAYS_NOT_MATCH              5

// An algorithm-specific error (e.g. a divide by zero)
#define ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE   6

// A parameter value is not in the restricted range
#define ROX_ERROR_INVALID_VALUE                 7

// The parameter value is too large for the current context
#define ROX_ERROR_TOO_LARGE_VALUE               8

// An error occured in an external library
#define ROX_ERROR_EXTERNAL                      9

// A value should be even and is not
#define ROX_ERROR_VALUE_NOT_EVEN                10

// A value should be odd and is not
#define ROX_ERROR_VALUE_NOT_ODD                 11

// A structure is not valid
#define ROX_ERROR_INVALID                       12

// Available error code
#define ROX_ERROR_TO_BE_DEFINED                 13

// Undocumented error
#define ROX_ERROR_INTERNAL                      14

// The filename is not valid or the file does not exist
#define ROX_ERROR_FILE_NOT_FOUND                15

// The template is not found
#define ROX_ERROR_TEMPLATE_NOT_FOUND            16

// An algorithm-specific error
#define ROX_ERROR_ALGORITHM_FAILURE             17

// A generic error
#define ROX_ERROR_PROCESS_FAILED                18

// The buffer is full
#define ROX_ERROR_FULL_BUFFER                   19

// The buffer is empty
#define ROX_ERROR_EMPTY_BUFFER                  20

// ZNCC undefined
#define ROX_ERROR_ZNCC_UNDEFINED                21

// Matrix with null determinant
#define ROX_ERROR_DETERMINANT_NULL              22

// Collection of points all set to the origin
#define ROX_ERROR_ALL_POINTS_NULL               23

// Not enough data to compute am algorithm
#define ROX_ERROR_INSUFFICIENT_DATA             24

// This feature or case is not implemented yet
#define ROX_ERROR_NOT_IMPLEMENTED               25

// Max number of available instances reached
#define ROX_ERROR_MAX_INSTANCES_REACHED         26

// Macro for checking errors and return
#define CHECK_ERROR_RETURN(A)    error = (A); if (error) return error;
#define CHECK_ERROR_CONTINUE(A)  error = (A); if (error) continue;
#define CHECK_ERROR_TERMINATE(A) error = (A); if (error) goto function_terminate;

#ifdef __cplusplus
   #define CHECK_ERROR_THROW(A) error = (A); if (error) throw error;
#endif

// Macro for avoiding unused variables
#define ROX_UNUSED(var) ((void)var)

#endif
