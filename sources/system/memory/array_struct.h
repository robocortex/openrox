//==============================================================================
//
//    OPENROX   : File array_struct.h
//
//    Contents  : Structure of array module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY_STRUCT__
#define __OPENROX_ARRAY_STRUCT__

#ifdef __cplusplus
extern "C" {
#endif

#include "datatypes.h"
#include "memory.h"
#include <system/errors/errors.h>

//! \ingroup System
//! @defgroup Array Array
//! @{

//! Structure of array 
struct Rox_Array_Struct
{
   //! Allocation pointer
   Rox_Void * system_ptr;

   //! Data start pointer
   Rox_Void * base_ptr;

   //! Data type for elements
   Rox_Datatype_Description datatype;

   //! Number of elements
   Rox_Size length;

   //! How many times is this array used in other structures ?
   Rox_Uint reference_count;

   //! Does this array own the data memory
   Rox_Bool owndata;
};

//! @} 

#ifdef __cplusplus
}
#endif

#endif
