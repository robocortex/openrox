//==============================================================================
//
//    OPENROX   : File sl3virtualview.h
//
//    Contents  : API of sl3virtualview module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SL3_VIRTUALVIEW__
#define __OPENROX_SL3_VIRTUALVIEW__

#include <generated/array2d_double.h>

//! \ingroup MatSL3
//! \addtogroup virtualview
//! @{

//! Generate a homography given needed x and y scaling
//! \param  []  new_width         computed new width
//! \param  []  new_height        computed new height
//! \param  []  homography        computed homography
//! \param  []  scalex            the x scaling needed
//! \param  []  scaley            the y scaling needed
//! \param  []  source_width      original width
//! \param  []  source_height     original height
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_virtualview_anisotropic_scaling(Rox_Sint *new_width, Rox_Sint *new_height, Rox_Array2D_Double homography, Rox_Double scalex, Rox_Double scaley, Rox_Sint source_width, Rox_Sint source_height);

//! Generate a homography given needed skew
//! \param  []  new_width         computed new width
//! \param  []  new_height        computed new height
//! \param  []  homography        computed homography
//! \param  []  skewx             the x skew needed
//! \param  []  source_width      original width
//! \param  []  source_height     original height
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_virtualview_skew(Rox_Sint *new_width, Rox_Sint *new_height, Rox_Array2D_Double homography, Rox_Double skewx, Rox_Sint source_width, Rox_Sint source_height);

//! @}

#endif
