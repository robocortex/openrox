//==============================================================================
//
//    OPENROX   : File planar_view_generator.h
//
//    Contents  : API of planar_view_generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VIEWGENERATOR_PLANAR__
#define __OPENROX_VIEWGENERATOR_PLANAR__

#include <generated/array2d_double.h>
#include <generated/array2d_uint.h>

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Vision
//! \addtogroup ViewGenerator

//! Planar view generator struct 
struct Rox_ViewGenerator_Planar_Struct
{
   //! Internal Transformations 
   Rox_Array2D_Double homography;

   //! Internal Transformations 
   Rox_Array2D_Double invhomography;

   //! Internal Transformations 
   Rox_Array2D_Double refhomography;

   //! Internal Transformations 
   Rox_Array2D_Double warphomography;

   //! Internal Transformations 
   Rox_Array2D_Double pose;

   //! Internal calibration 
   Rox_Array2D_Double calib_input;

   //! Internal calibration 
   Rox_Array2D_Double calib_output;

   //! Generated output
   Rox_Image generated;

   //! Generated output mask
   Rox_Imask generated_mask;

   //! Generated output warping grid
   Rox_MeshGrid2D_Float generated_grid;

   //! Source width 
   Rox_Uint image_width;

   //! Source height 
   Rox_Uint image_height;
};

//! Planar view generator struct 
typedef struct Rox_ViewGenerator_Planar_Struct * Rox_ViewGenerator_Planar;

//! Create a new generator of virtual views of planar templates object.
//! Viewpoint bin cluster many viewpoints to learn features
//! \param  [out]  obj            The pointer to the generator created.
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_viewgenerator_planar_new (
   Rox_ViewGenerator_Planar * obj
);

//! Delete a generator of virtual views of planar templates object.
//! \param  [out]  obj            The pointer to the generator to delete.
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_viewgenerator_planar_del (
   Rox_ViewGenerator_Planar * obj
);

//! Set the generator source image size.
//! \param  [out]  obj            The object to use
//! \param  [in ]  width          The source image width
//! \param  [in ]  height         The source image height
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_viewgenerator_planar_set_source_size (
   Rox_ViewGenerator_Planar obj, 
   Rox_Sint width, 
   Rox_Sint height
);

//! Perform generation.
//! \param  [out]  obj the object to use
//! \param  [in ]  source the image to use as source (must be the size set by rox_viewgenerator_planar_set_source_size)
//! \param  [in ]  scale the scale of the camera used for generation
//! \param  [in ]  scale the rotation (image plane) of the camera used for generation
//! \param  [in ]  origin the 3d point which defines the optical center of the camera (0,0,0 is the plane center, 0,0,1 is the typical front camera)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_viewgenerator_planar_generate (
   Rox_ViewGenerator_Planar obj, 
   Rox_Image source, 
   Rox_Double scale, 
   Rox_Double inplane_rot, 
   Rox_Point3D_Double origin
);

//! @} 

#endif
