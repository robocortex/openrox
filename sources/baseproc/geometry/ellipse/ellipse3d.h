//==============================================================================
//
//    OPENROX   : File ellipse3d.h
//
//    Contents  : API of ellipse_3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE3D__
#define __OPENROX_ELLIPSE3D__

#include <baseproc/maths/linalg/matse3.h>

//! \ingroup  Geometry
//! \addtogroup Ellipse
//! \brief Ellipse
//! @{

//! Pointer to the Rox_Ellipse3D structure
typedef struct Rox_Ellipse3D_Double_Struct * Rox_Ellipse3D;

//! Create a Rox_Ellipse3D object and initialize it to a = 1.0, b = 1.0, Te = I
//! \param [out]  ellipse3d      The object to create
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_ellipse3d_new(Rox_Ellipse3D * ellipse3d);

//! Set the ellipse parameters
//! \param  [out]  ellipse3d      The object to set
//! \param  [in ]	 a 				 The scale factor along the x->axis 
//! \param  [in ]	 b 				 The scale factor along the y->axis 
//! \param  [in ]	 T 			    The pose of the ellipse relative to a reference frame
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_ellipse3d_set(Rox_Ellipse3D ellipse3d, Rox_Double a, Rox_Double b, Rox_MatSE3 T);

//! Delete a Rox_MatSL3 object
//! \param  [out]  matsl3         The object to delete
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_ellipse3d_del(Rox_Ellipse3D * ellipse3d);

//! Copy a Rox_Ellipse3D object
//! \param  [out]  ellipse3d_out  The object to copy into
//! \param  [out]  ellipse3d_inp  The object to be copied
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse3d_copy(Rox_Ellipse3D ellipse3d_out, Rox_Ellipse3D ellipse3d_inp);

//! Display the parameters of a Rox_Ellipse3D object
//! \param  [out]  ellipse3d      The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse3d_print(Rox_Ellipse3D ellipse3d);

//! @}

#endif // __OPENROX_ELLIPSE_3D__
