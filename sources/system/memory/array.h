//============================================================================
//
//    OPENROX   : File array.h
//
//    Contents  : API of array module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ARRAY__
#define __OPENROX_ARRAY__

#ifdef __cplusplus
extern "C" {
#endif

#include "datatypes.h"
#include "memory.h"
#include <system/errors/errors.h>

//! \ingroup System
//! @defgroup Array Array
//! @{

//! Pointer to structure 
typedef struct Rox_Array_Struct * Rox_Array;

//! Create a new array
//! \param  [out]  array          The pointer to the newly created array
//! \param  [in ]  datatype       Type of elements
//! \param  [in ]  length         Number of elements
//! \return Error code
ROX_API Rox_ErrorCode rox_array_new (
   Rox_Array * array, 
   const Rox_Datatype_Description datatype, 
   const Rox_Uint length
);

//! Create a new array containing a pre-allocated buffer
//! \remarks buffer must be aligned on ROX_DEFAULT_ALIGNMENT
//! \param  [out]  array            The pointer to the newly created array
//! \param  [in ]  datatype       Type of elements
//! \param  [in ]  length         Number of elements
//! \param  [in ]  buffer         Pre-allocated aligned buffer
//! \return Error code
ROX_API Rox_ErrorCode rox_array_new_frombuffer (
   Rox_Array *array, 
   Rox_Datatype_Description datatype, 
   const Rox_Uint length, 
   Rox_Void * buffer
);

//! Delete an array
//! \param  [out]  ptr            Pointer to the Array to delete
//! \return Error code
ROX_API Rox_ErrorCode rox_array_del (
   Rox_Array *ptr
);

//! Increment the reference counter
//! \param  [out]  ptr            Array to update
//! \return Error code
ROX_API Rox_ErrorCode rox_array_reference (
   Rox_Array *ptr
);

//! Decrement the reference counter, delete if unreferenced
//! \param  [out]  ptr            Array to update
//! \return Error code
ROX_API Rox_ErrorCode rox_array_dereference (
   Rox_Array *ptr
);

//! Copy an array (array should be of same size and type)
//! \param  [out]  dest           The destination array
//! \param  [in ]  src            The source array
//! \return An Error code 
ROX_API Rox_ErrorCode rox_array_copy (
   Rox_Array dest, const Rox_Array src
);

//! @} 

#ifdef __cplusplus
}
#endif

#endif //__OPENROX_ARRAY__
