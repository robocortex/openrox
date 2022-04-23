//==============================================================================
//
//    OPENROX   : File StereoRectifier.h
//
//    Contents  : API of StereoRectifier module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_STEREO_RECTIFIER__
#define __OPENROX_STEREO_RECTIFIER__

#include <generated/array2d_uint.h>
#include <generated/array2d_double.h>
#include <generated/array2d_point2d_sshort.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Stereo_Vision
//! \addtogroup Stereo_Rectification
//! \brief Structure and functions of the perspective stereo rectification
//! @{

//! Stereo rectification structure 
struct Rox_StereoRectifier_Struct
{
    //! To be commented  
   Rox_Array2D_Double rectified_calibration;

   //! To be commented  
   Rox_Array2D_Double rectified_poseleft;

    //! To be commented  
   Rox_Array2D_Double rectified_poseright;
   
   //! To be commented  
   Rox_Array2D_Double rectified_relativepose;

    //! To be commented  
   Rox_Array2D_Double rectified_homographyleft;

   //! To be commented  
   Rox_Array2D_Double rectified_homographyright;

   //! To be commented  
   Rox_Image rectified_imageleft;

   //! To be commented  
   Rox_Image rectified_imageright;

    //! To be commented  
   Rox_Array2D_Uint  rectified_rgba_imageleft;

    //! To be commented  
   Rox_Array2D_Uint  rectified_rgba_imageright;

    //! To be commented  
   Rox_Imask  rectified_leftmask;

    //! To be commergba_nted  
   Rox_Imask  rectified_rightmask;

    //! To be commented  
   Rox_MeshGrid2D_Float rectified_grid_left;

    //! To be commented  
   Rox_MeshGrid2D_Float rectified_grid_right;

    //! To be commented  
   Rox_Array2D_Point2D_Sshort rectified_grid_left_fixed;

   //! To be commented  
   Rox_Array2D_Point2D_Sshort rectified_grid_right_fixed;

   //! To be commented  
   Rox_Double rectified_baseline;
};

//! Slam Stereo rectification structure 
typedef struct Rox_StereoRectifier_Struct * Rox_StereoRectifier;

//! Create a new rectification structure for stereo
//! \param  [out]  obj            The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_stereorectifier_new ( Rox_StereoRectifier * obj );

//! Delete a rectification structure
//! \param  [out]  obj            The pointer to the object
//! \return An error code
ROX_API Rox_ErrorCode rox_stereorectifier_del ( Rox_StereoRectifier * obj );

//! Prepare a rectification structure
//! \param  [out]  obj            The object to set
//! \param  [in ]  Kl 
//! \param  [in ]  Dl 
//! \param  [in ]  Kr 
//! \param  [in ]  Dr
//! \param  [in ]  rTl 
//! \param  [in ]  width
//! \param  [in ]  height
//! \return An error code
ROX_API Rox_ErrorCode rox_stereorectifier_prepare ( 
   Rox_StereoRectifier obj, 
   Rox_Array2D_Double Kl, 
   Rox_Array2D_Double Dl, 
   Rox_Array2D_Double Kr, 
   Rox_Array2D_Double Dr, 
   Rox_Array2D_Double rTl, 
   Rox_Sint width, 
   Rox_Sint height
);

//! Apply a rectification structure to an input image pair
//! \param  [out]  obj            The object to set
//! \param  [in ]  left           The input image left
//! \param  [in ]  right          The input image right
//! \return An error code
ROX_API Rox_ErrorCode rox_stereorectifier_apply ( Rox_StereoRectifier obj, Rox_Image left, Rox_Image right );

//! Apply a rectification structure to an input color image pair
//! \param  [out]  obj            The object to set
//! \param  [in ]  rgba_left      The input image left
//! \param  [in ]  rgba_right     The input image right
//! \return An error code
ROX_API Rox_ErrorCode rox_stereorectifier_rgba_apply ( Rox_StereoRectifier obj, Rox_Array2D_Uint rgba_left, Rox_Array2D_Uint rgba_right );

//! @}

#endif

