//==============================================================================
//
//    OPENROX   : File objset_@LOBJSETTYPE@.h
//
//    Contents  : API of objset_@LOBJSETTYPE@ module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OBJSET_@OBJSETTYPE@__
#define __OPENROX_OBJSET_@OBJSETTYPE@__

#include <system/memory/datatypes.h>
#include <system/memory/memory.h>
#include <system/errors/errors.h>

#include @OBJSETTYPE_INCLUDE@
#include @OBJSETTYPE_INCLUDE_STRUCT@

//! \ingroup ObjSet
//! @defgroup ObjSet_@OBJSETTYPE@ objset_@OBJSETTYPE@
//! @{

//! A Objset pointer
typedef struct Rox_ObjSet_@OBJSETTYPE@_Struct * Rox_ObjSet_@OBJSETTYPE@;

//! Create a new Object set structure (a set of managed pointer to objects)
//! \param  [out]  obj            Address of the pointer for the newly created object
//! \param  [in ]  allocblocks    How many cells are allocated
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_new (
   Rox_ObjSet_@OBJSETTYPE@ * obj, 
   Rox_Uint allocblocks
);

//! Delete a objset structure (Will delete all attached objects)
//! \param  [out]     ptr         The pointer to the dynamic array to be deleted
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_del (
   Rox_ObjSet_@OBJSETTYPE@ * ptr
);

//! Retrieve the list of  managed pointers of an objset
//! \param  [out]  ptr            The objset array to get data from
//! \return the pointer
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_get_data_pointer (
   Rox_@OBJSETTYPE@ ** data_pointer,
   Rox_ObjSet_@OBJSETTYPE@ ptr
);

//! Retrieve the number of blocks used in objset
//! \param  [out]  used           the size of the objset
//! \param  [in ]  ptr            the objset array to get data from
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_get_used (
   Rox_Uint* used, Rox_ObjSet_@OBJSETTYPE@ ptr
);

//! Reset a objset structure
//! \param  [out] ptr             The dynamic array to be reseted
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_reset (
   Rox_ObjSet_@OBJSETTYPE@ ptr
);

//! Stack two objset together
//! \param  [out]  ptr            The dynamic array to update
//! \param  [in ]  other          The dynamic array to add after
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_stack (
   Rox_ObjSet_@OBJSETTYPE@ ptr, 
   Rox_ObjSet_@OBJSETTYPE@ other
);

//! Clone one objset
//! \param  [out]  ptr            The dynamic array to update with other array
//! \param  [in ]  source         The dynamic array to clone
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_clone (
   Rox_ObjSet_@OBJSETTYPE@ ptr, 
   Rox_ObjSet_@OBJSETTYPE@ source
);

//! Add an element to the objset
//! \param  [out]  ptr            The object pointer to add
//! \param  [in ]  data           The value to add (MUST BE A REAL POINTER, NOT A POINTER TO A LOCAL VARIABLE)
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_@LOBJSETTYPE@_append (
   Rox_ObjSet_@OBJSETTYPE@ ptr, 
   Rox_@OBJSETTYPE@ data
);

//! @}

#endif
