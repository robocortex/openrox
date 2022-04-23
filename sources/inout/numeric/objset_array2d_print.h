//============================================================================
//
//    OPENROX   : File objset_array2d_print.h
//
//    Contents  : API of array2d_serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_OBJSET_ARRAY2D_PRINT__
#define __OPENROX_OBJSET_ARRAY2D_PRINT__

#include <stdio.h>
#include <generated/objset_array2d_double.h>

//! Display of an objset of array2D.
//! \param  [in]  input          The objset of array2d to print
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_double_print(Rox_ObjSet_Array2D_Double input);

#endif   //__objset_array2d_print__
