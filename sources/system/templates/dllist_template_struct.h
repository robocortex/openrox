//==============================================================================
//
//    OPENROX   : File dllist_struct_template.h
//
//    Contents  : Structure of dllist_template module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DLLIST_STRUCT_TYPED_@DLLISTTYPENAME@__
#define __OPENROX_DLLIST_STRUCT_TYPED_@DLLISTTYPENAME@__

#include <system/memory/datatypes.h>
#include <system/memory/memory.h>
#include <system/errors/errors.h>
#include @DLLIST_STRUCT_INCLUDE@

//! \ingroup DlList
//! @defgroup DlList_@DLLISTTYPENAME@ DlList_@DLLISTTYPENAME@
//! \brief DlList with type @DLLISTTYPENAME@.
//! @{

//! A DLList Node
struct Rox_Dllist_@DLLISTTYPE@_Node_Struct
{
   //!Pointer to previous node
   struct Rox_Dllist_@DLLISTTYPE@_Node_Struct * previous;

   //!Pointer to next node
   struct Rox_Dllist_@DLLISTTYPE@_Node_Struct * next;

   //!Data container for this node
   Rox_@DLLISTTYPE@_Struct data;
};

//! A DlList Node pointer
typedef struct Rox_Dllist_@DLLISTTYPE@_Node_Struct Rox_Dllist_@DLLISTTYPE@_Node_Struct ;

//! A structure for basic double linked list
struct Rox_Dllist_@DLLISTTYPE@_Struct
{
   //! How many nodes are used
   Rox_Uint used;

   //! How manu nodes are allocated
   Rox_Uint allocated;

   //! First node of the list
   Rox_Dllist_@DLLISTTYPE@_Node_Struct * first;

   //! Last USED node of the list.
   //! A node may be allocated previously but not needed for the moment
   Rox_Dllist_@DLLISTTYPE@_Node_Struct * last_used;

   //! Last node of the list
   Rox_Dllist_@DLLISTTYPE@_Node_Struct * last;
};

//! A DllList pointer
typedef struct Rox_Dllist_@DLLISTTYPE@_Struct Rox_Dllist_@DLLISTTYPE@_Struct;

//! @}

#endif
