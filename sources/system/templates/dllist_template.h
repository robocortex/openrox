//==============================================================================
//
//    OPENROX   : File dllist_template.h
//
//    Contents  : API of dllist_template module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DLLIST_TYPED_@DLLISTTYPENAME@__
#define __OPENROX_DLLIST_TYPED_@DLLISTTYPENAME@__

#include <system/memory/datatypes.h>
#include <system/memory/memory.h>
#include <system/errors/errors.h>
#include @DLLISTTYPE_INCLUDE@
#include @DLLIST_STRUCT_INCLUDE@

//! \ingroup DlList
//! @defgroup DlList_@DLLISTTYPENAME@ DlList_@DLLISTTYPENAME@
//! \brief DlList with type @DLLISTTYPENAME@.
//! @{

//! A DlList Node pointer
typedef struct Rox_Dllist_@DLLISTTYPE@_Node_Struct * Rox_Dllist_@DLLISTTYPE@_Node;

//! A DllList pointer
typedef struct Rox_Dllist_@DLLISTTYPE@_Struct * Rox_Dllist_@DLLISTTYPE@;

//! Create a new double linked list
//! \param  [out]  obj            A pointer to the newly created list
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_new(Rox_Dllist_@DLLISTTYPE@ * obj);

//! Delete a double linked list
//! \param  [in]   obj            A pointer to the list to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_del(Rox_Dllist_@DLLISTTYPE@ * obj);

//! Append a node to the end of the list
//! \param  [out]  obj            A pointer to the list to modify
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_append(Rox_Dllist_@DLLISTTYPE@ obj);

//! Delete the last node of the list
//! \param  [out]  obj            A pointer to the list to modify
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_removelast(Rox_Dllist_@DLLISTTYPE@ obj);

//! Delete the first node of the list
//! \param  [out]  obj            A pointer to the list to modify
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_removefirst(Rox_Dllist_@DLLISTTYPE@ obj);

//! Reset the list
//! \param  [out]  obj            A pointer to the list to modify
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_reset(Rox_Dllist_@DLLISTTYPE@ obj);

//! Add value to the list
//! \param  [out] obj A pointer to the list to modify
//! \param  [in]     val The value to add
//! \return An error code
ROX_API Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_add(Rox_Dllist_@DLLISTTYPE@ obj, Rox_@DLLISTTYPE@_Struct * val);

//! @}

#endif
