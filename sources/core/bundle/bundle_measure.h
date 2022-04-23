//==============================================================================
//
//    OPENROX   : File bundle_measure.h
//
//    Contents  : API of bundle_measure module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_MEASURE__
#define __OPENROX_BUNDLE_MEASURE__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! Bundle Camera structure
struct Rox_Bundle_Camera_Struct;

//! Bundle Frame structure
struct Rox_Bundle_Frame_Struct;

//! Bundle point structure
struct Rox_Bundle_Point_Struct;

//! Bundle Measure structure
struct Rox_Bundle_Measure_Struct
{
   //! Validity flag
   Rox_Uint is_invalid;

   //! Associated camera shallow pointer
   struct Rox_Bundle_Camera_Struct * camera;

   //! Associated frame shallow pointer
   struct Rox_Bundle_Frame_Struct * frame;

   //! Associated point shallow pointer
   struct Rox_Bundle_Point_Struct * point;

   //! Covariance for this measure
   Rox_Double weight;

   //! Jacobian wrt point
   Rox_Double jacPoint[2][3];

   //! Jacobian wrt frame
   Rox_Double jacFrame[2][6];

   //! Jpoint^t * Jframe
   Rox_Matrix JpointTJframe;

   //! Transformed coordinates
   Rox_Point3D_Double_Struct transformed_point;

   //! Coordinates in pixels of the prediction
   Rox_Point2D_Double_Struct coords_estimated_pixels;

   //! Normalized coordinates in meters of the prediction
   Rox_Point2D_Double_Struct coords_estimated_meters;

   //! Coordinates in pixels of the measure
   Rox_Point2D_Double_Struct coords_pixels;

   //! Normalized coordinates in meters of the measure
   Rox_Point2D_Double_Struct coords_meters;

   //! Error in pixels between measure and estimation
   Rox_Point2D_Double_Struct error_pixels;

   //! Distance in pixels between measure and estimation
   Rox_Double distance_pixels;

   //! Intermediate bundle buffer
   Rox_MatSE3 Tpc;
};

//! Bundle measure object
typedef struct Rox_Bundle_Measure_Struct * Rox_Bundle_Measure;

//! Create a container object for a bundle measure
//! \param  []  obj the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_measure_new(Rox_Bundle_Measure * obj);

//! Delete a container object for a bundle measure
//! \param  []  obj the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_measure_del(Rox_Bundle_Measure * obj);

//! Predict a bundle measure
//! \param  []  obj the measure object
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_measure_predict(Rox_Bundle_Measure obj);

//! Compute jacobians for this measure (after prediction)
//! \param  []  obj the measure object
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_measure_build_jacobians(Rox_Bundle_Measure obj);

//! @}

#endif
