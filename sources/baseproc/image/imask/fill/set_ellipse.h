//============================================================================
//
//    OPENROX   : File set_ellipse.h
//
//    Contents  : API of set_ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_MASK_SET_ELLIPSE__
#define __OPENROX_MASK_SET_ELLIPSE__

#include <generated/array2d_uint.h>
#include "baseproc/geometry/ellipse/ellipse2d.h"

//! \ingroup  Mask
//! \addtogroup MaskSetEllipse
//! @{

//! Set the mask with an elliptic shape
//! \param  [in ]  dest  The mask to set
//! \return An error code
//! \todo  To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_set_centered_ellipse ( Rox_Array2D_Uint mask );
ROX_API Rox_ErrorCode rox_array2d_uint_new_ellipse ( Rox_Array2D_Uint * mask, const Rox_Ellipse2D ellipse2d );

//! @}

#endif // __OPENROX_MASK_SET_ELLIPSE__
