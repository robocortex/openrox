//==============================================================================
//
//    OPENROX   : File calibration_generalized.h
//
//    Contents  : API of calibration_generalized module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_GENERALIZED__
#define __OPENROX_CALIBRATION_GENERALIZED__

#include <generated/objset_dynvec_point_double.h>
#include <generated/objset_array2d_double.h>
#include <generated/dynvec_double.h>

//! \ingroup  Camera
//! \defgroup Camera_Calibration Camera Calibration 
//! \brief Camera calibration Structure and methods for intrisic parameters camera calibration.

//! \ingroup Camera_Calibration
//! \defgroup Calibration_Mono_Generalized Generalized Monocular Calibration

//! \addtogroup Calibration_Mono_Generalized
//! @{

//! Define the pointer of the Rox_Calibration_Mono_Perspective_Struct
typedef struct Rox_Calibration_Mono_Generalized_Struct * Rox_Calibration_Mono_Generalized;

//! \brief Create a new calibration object
//!  \param  []  obj is the newly created calibration object
//!  \param polynomial_order the polynom order for the calibration model
//!  \return An error code
ROX_API Rox_ErrorCode rox_calibration_mono_generalized_new (
   Rox_Calibration_Mono_Generalized * obj, 
   Rox_Uint polynomial_order
);

//!  \brief Delete a calibration object
//!  \param  []  obj is the created calibration object to delete
//!  \return An error code
ROX_API Rox_ErrorCode rox_calibration_mono_generalized_del (
   Rox_Calibration_Mono_Generalized *obj
);

//! \brief Set the image size in pixels
//! \param  []  obj the calibration to use
//! \param  []  width the image width
//! \param  []  height the image height
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_mono_generalized_set_imagesize (
   Rox_Calibration_Mono_Generalized obj, 
   Rox_Sint width, 
   Rox_Sint height
);


//! \brief Add a view to the calibration object
//! \param  []  obj the calibration to use
//! \param  []  points the list of points with 2D and 3D information filled (2D in pixels)
//! \param  []  count the number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_mono_generalized_append_view (
   Rox_Calibration_Mono_Generalized obj, 
   Rox_Point_Double points, 
   Rox_Uint count
);

//! \brief Compute calibration given preset input
//! \param  []  obj the calibration to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_mono_generalized_process (
   Rox_Calibration_Mono_Generalized obj
);

//! @}

#endif // __OPENROX_CALIBRATION_GENERALIZED__
