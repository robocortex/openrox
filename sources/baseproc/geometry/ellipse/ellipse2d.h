//==============================================================================
//
//    OPENROX   : File ellipse2d.h
//
//    Contents  : API of ellipse2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE2D__
#define __OPENROX_ELLIPSE2D__

#include <generated/dynvec_point2d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/rectangle/rectangle.h>

//! \ingroup  Geometry
//! \addtogroup Ellipse
//! \brief Ellipse
//! @{

//! Pointer to the Rox_Ellipse2D structure
typedef struct Rox_Ellipse2D_Double_Struct * Rox_Ellipse2D;

typedef struct Rox_Ellipse2D_Double_Struct * Rox_Ellipse2D_Parametric;

//! Create a Rox_Ellipse2D object and initialize it to xc = 1.0, yc = 1.0, nxx = 1.0, nyy = 1.0, nxy = 1.0 
//! \param  [out]  ellipse2d      The object to create
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse2d_new ( Rox_Ellipse2D * ellipse2d );

//! Set the ellipse parameters
//! \param  [out]  ellipse2d      The object to set
//! \param  [in ]  xc
//! \param  [in ]  yc
//! \param  [in ]  nxx
//! \param  [in ]  nyy
//! \param  [in ]  nxy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse2d_set ( Rox_Ellipse2D ellipse2d, Rox_Double xc, Rox_Double yc, Rox_Double nxx, Rox_Double nyy, Rox_Double nxy );

//! Delete a Rox_Ellipse2D object
//! \param  [out]  ellipse2d         The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse2d_del ( Rox_Ellipse2D * ellipse2d );

//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse2d_sample ( Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Ellipse2D ellipse2d, Rox_Double sampling_angle );

//! \return An error code
//! \todo   To be tested
//! \todo   Should xe, ye be a point2d_double
ROX_API Rox_ErrorCode rox_ellipse2d_get_tangent_angle ( Rox_Double * angle, Rox_Ellipse2D ellipse2d, const Rox_Double xe, const Rox_Double ye );

//! \return An error code
//! \todo   To be tested
//! \todo   Should xe, ye be a point2d_double
ROX_API Rox_ErrorCode rox_ellipse2d_get_normal_angle ( Rox_Double * angle, Rox_Ellipse2D ellipse2d, const Rox_Double xe, const Rox_Double ye);

//! Display the parameters of a Rox_Ellipse2D object
//! \param  [out]  ellipse2d        The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ellipse2d_print(Rox_Ellipse2D ellipse2d);

//! \return An error code
//! \param  [out]  dynvec_point2d    The sampled points
//! \param  [in ]  ellipse2d         The ellipse to sample
//! \param  [in ]  sampling_step     The sampling step 
//! \todo   To be tested, move the function in baseproc/geometry/measures/distance_point_to_ellipse.h
ROX_API Rox_ErrorCode rox_ellipse2d_sample_perimeter(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Ellipse2D ellipse2d, Rox_Double sampling_step);

ROX_API Rox_ErrorCode rox_ellipse2d_convert_canonical_to_parametric ( Rox_Double * xc, Rox_Double * yc, Rox_Double * a, Rox_Double * b, Rox_Double * phi, Rox_Ellipse2D ellipse2d);
ROX_API Rox_ErrorCode rox_ellipse2d_convert_parametric_to_canonical ( Rox_Ellipse2D ellipse2d, Rox_Double xc, Rox_Double yc, Rox_Double a, Rox_Double b, Rox_Double phi );

ROX_API Rox_ErrorCode rox_ellipse2d_get_rect_sint ( 
   Rox_Rect_Sint window, 
   Rox_Ellipse2D ellipse2d 
);

//! @}

#endif // __OPENROX_ELLIPSE_3D__
