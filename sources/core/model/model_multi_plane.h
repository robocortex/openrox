//==============================================================================
//
//    OPENROX   : File model_multi_plane.h
//
//    Contents  : API of model_multi_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MODEL_MULTI_PLANE__
#define __OPENROX_MODEL_MULTI_PLANE__

#include <generated/objset_model_single_plane.h>
#include <generated/objset_model_single_plane_struct.h>

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/point/point3d.h>

#include <core/model/model_single_plane_struct.h>
#include <core/model/model_multi_plane_struct.h>

//! \addtogroup Model_Multi_Plane
//! @{
//! Pointer ti the structure
typedef struct Rox_Model_Multi_Plane_Struct * Rox_Model_Multi_Plane;


//! Create a new multiplane model object
//! \param  [out]  model_multi_plane         The pointer to the object created
//! \return An error code
ROX_API Rox_ErrorCode rox_model_multi_plane_new ( 
   Rox_Model_Multi_Plane * model_multi_plane 
);


//! Delete multiplane model object
//! \param  [out]  model_multi_plane         The pointer to the object to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_model_multi_plane_del ( 
   Rox_Model_Multi_Plane * model_multi_plane 
);


//! Add a new plane to the object
//! \param  [out]  model_multi_plane    The pointer to the object 
//! \param  [in ]  source               The luminosity to use
//! \param  [in ]  vertices             The 4 coplanar ( important ) vertices of the template
//! \param  [in ]  basesize             The minimal size of the unwrapped template to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_multi_plane_append_3d_template ( 
   Rox_Model_Multi_Plane model_multi_plane, 
   const Rox_Image image_template, 
   const Rox_Point3D_Double vertices, 
   const Rox_Sint basesize 
);


ROX_API Rox_ErrorCode rox_model_multi_plane_append_plane (
   Rox_Model_Multi_Plane    model_multi_plane,
   const Rox_Image          image_template,
   const Rox_Point3D_Double vertices_cur 
);


//! Set the current pose.
//! \param  [out]  model_multi_plane         The model to modify
//! \param  [in ]  pose                 The input current pose matrix
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_model_multi_plane_set_currentpose ( 
   Rox_Model_Multi_Plane model_multi_plane, 
   const Rox_MatSE3 pose 
);


//! Transform a model given the current pose.
//! \param  [out]  model_multi_plane         The the model to modify
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_multi_plane_transform ( 
   Rox_Model_Multi_Plane model_multi_plane 
);


//! Retrieve transformed model vertices for a given plane id
//! \param  [out]  vertices             The 4 result transformed vertices
//! \param  [in ]  model_multi_plane     The model to use
//! \param  [in ]  id                   The plane id
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_multi_plane_get_vertices_cur ( 
   Rox_Point3D_Double vertices, 
   const Rox_Model_Multi_Plane model_multi_plane, 
   const Rox_Sint id 
);


//! Copy a model onto another.
//! \param  [out]  dst                  The destination model
//! \param  [in ]  src                  The source model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_model_multi_plane_copy ( 
   Rox_Model_Multi_Plane dst, 
   const Rox_Model_Multi_Plane src 
);


ROX_API Rox_ErrorCode rox_model_multi_plane_get_number (
   Rox_Sint * nbp,
   const Rox_Model_Multi_Plane model_multi_plane
);


ROX_API Rox_ErrorCode rox_model_multi_plane_check_visibility (
   Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Sint image_rows,
   const Rox_Sint image_cols,   
   const Rox_MatUT3 Kc,
   const Rox_MatSE3 cTo
);


//! @}

#endif
