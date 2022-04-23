//============================================================================
//
//    OPENROX   : File gaussian2d.h
//
//    Contents  : API of gaussian2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_GAUSSIAN2D__
#define __OPENROX_GAUSSIAN2D__

#include <generated/array2d_float.h>

//! \ingroup Image
//! \addtogroup gaussian2d
//! @{

//! Create a separated gaussian smoothing kernel
//! \param  [out]  hfilter        The horizontal gaussian filter (allocated by function)
//! \param  [out]  vfilter        The vertical gaussian filter (allocated by function)
//! \param  [in ]  sigma          The deviation of the gaussian
//! \param  [in ]  cutoff         How many times sigma is multiplied to get the limits
//! \return An error code
//! \remarks hfilter and vfilter should be freed by function user afterwards
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_kernelgen_gaussian2d_separable_float_new(Rox_Array2D_Float * hfilter, Rox_Array2D_Float * vfilter, Rox_Float sigma, Rox_Float cutoff);

//! @}

#endif
