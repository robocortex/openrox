//============================================================================
//
//    OPENROX   : File model_single_plane.h
//
//    Contents  : API of model_single_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_MODEL_ONEPLANE__
#define __OPENROX_MODEL_ONEPLANE__

#include <generated/array2d_uint.h>
#include <generated/array2d_double.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/image/image.h>

//! \addtogroup Model_Single_Plane
//! @{

//! model_single_planeect
typedef struct Rox_Model_Single_Plane_Struct * Rox_Model_Single_Plane;

//! Create a new empty model for a planar template
//! \param  [out]  model_single_plane           The pointer to the newly created model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_new (
   Rox_Model_Single_Plane * model_single_plane
);

//! Delete a model for a planar template
//! \param  [out]  model_single_plane           The pointer to the model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_del (
   Rox_Model_Single_Plane * model_single_plane
);

//! Assign a given template to the model.
//! \param  [in ]  model_single_plane           The model to modify
//! \param  [in ]  source        The luminosity to use
//! \param  [in ]  vertices      The 4 coplanar (important) vertices of the template
//! \param  [in ]  basesize
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_set_3d_template (
   Rox_Model_Single_Plane model_single_plane, 
   const Rox_Image source, 
   const Rox_Point3D_Double vertices, 
   const Rox_Double basesize
);

//! Set the model image and its dimensions
//! \param  [out]  model         The pointer to the model object
//! \param  [in ]  image         The model image
//! \param  [in ]  sizex         The model size along the x axis in meters
//! \param  [in ]  sizey         The model size along the y axis in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_set_template_xright_ydown (
   Rox_Model_Single_Plane model, 
   const Rox_Image image, 
   const Rox_Double sizex, 
   const Rox_Double sizey
);

ROX_API Rox_ErrorCode rox_model_single_plane_set_template_xright_yup (
   Rox_Model_Single_Plane model, 
   const Rox_Image image, 
   const Rox_Double sizex, 
   const Rox_Double sizey
);

//! Set the current pose.
//! \param  [out]  model_single_plane   The model to modify
//! \param  [in ]  pose                 The input pose matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_set_pose ( 
   Rox_Model_Single_Plane model_single_plane, 
   const Rox_MatSE3 pose 
);

//! Get the current pose.
//! \param  [out]  pose                 The input pose matrix
//! \param  [in ]  model_single_plane   The model to modify
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_get_pose (
   Rox_MatSE3 pose,
   const Rox_Model_Single_Plane model_single_plane
);

//! Transform a model given the current pose.
//! \param  [in ]  model_single_plane           The model to modify
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_transform (
   Rox_Model_Single_Plane model_single_plane
);

ROX_API Rox_ErrorCode rox_model_single_plane_check_visibility (
   Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint image_rows,
   const Rox_Sint image_cols,
   const Rox_MatUT3 Kc,
   const Rox_MatUT3 cTo
);

//! Retrieve transformed model vertices
//! \param  [out]  vertices               The result transformed vertices
//! \param  [in ]  model_single_plane         The model to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_get_vertices_cur (
   Rox_Point3D_Double vertices, 
   const Rox_Model_Single_Plane model_single_plane
);

//! Retrieve transformed model vertices
//! \param  [out]  vertices               The result transformed vertices
//! \param  [in ]  model_single_plane         The model to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_get_vertices_ref (
   Rox_Point3D_Double vertices, 
   const Rox_Model_Single_Plane model_single_plane
);

//! Copy a model onto another.
//! \param  [out]  dst           The destination model
//! \param  [in ]  src           The source model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_single_plane_copy (
   Rox_Model_Single_Plane dst, 
   const Rox_Model_Single_Plane src
);

//! @}

#endif
