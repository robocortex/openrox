//==============================================================================
//
//    OPENROX   : File gradientsobel.h
//
//    Contents  : API of gradientsobel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_GRADIENT_SOBEL__
#define __OPENROX_GRADIENT_SOBEL__

#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_sint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_sshort.h>
#include <generated/array2d_point2d_sshort.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Gradient
//! @{

//! Compute gradient using sobel kernel without masks
//! \param   [out]  gu            The result horizontal gradient using kernel [ 1; 2; 1]*[-1, 0, 1];
//! \param   [out]  gv            The result vertical   gradient using kernel [-1; 0; 1]*[ 1, 2, 1]; 
//! \param   [in ]  image_gray    The source image
//! \warning The border are not considered 
//! \warning The gradient is not normalized : it is scale by a factor of 8
//! \return  An error code
ROX_API Rox_ErrorCode rox_array2d_float_gradientsobel_nomask(Rox_Array2D_Float gu, Rox_Array2D_Float gv, Rox_Array2D_Float image_gray);

//! Compute gradient using sobel kernel without masks
//! \param   [out]  gu            The result horizontal gradient using kernel [ 1; 2; 1]*[-1, 0, 1];
//! \param   [out]  gv            The result vertical   gradient using kernel [-1; 0; 1]*[ 1, 2, 1]; 
//! \param   [in ]  image_gray    The source image
//! \warning The border are not considered 
//! \warning The gradient is not normalized : it is scale by a factor of 8
//! \return  An error code
ROX_API Rox_ErrorCode rox_array2d_sint_gradientsobel_nomask(Rox_Array2D_Sint gu, Rox_Array2D_Sint gv, Rox_Image image_gray);

//! Compute gradient using sobel kernel without masks
//! \param   [out]  gradient       The gradient
//! \param   [in ]  image_gray     The source image
//! \warning The border are not considered 
//! \warning The gradient is not normalized : it is scale by a factor of 8
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_gradientsobel_nomask(Rox_Array2D_Point2D_Sshort gradient, Rox_Image image_gray);

//! Compute gradient at a single image point using sobel kernel without mask 
//! \param   [out]  gradient      The result gradient along the u and v axis
//! \param   [in ]  image_gray    The source image
//! \param   [in ]  u,v           Coordinates
//! \warning The gradient is not normalized 
//! \warning The gradient is not normalized : it is scale by a factor of 8
//! \return  An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_gradientsobel_singlepoint_nomask(Rox_Point2D_Sshort gradient, Rox_Image image_gray, Rox_Sint u, Rox_Sint v);

//! @} 

#endif // __OPENROX_GRADIENT_SOBEL__
