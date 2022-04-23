//==============================================================================
//
//    OPENROX   : File odometry_dense_depthmap.h
//
//    Contents  : API of odometry_dense_depthmap module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_DENSE_DEPTHMAP__
#define __OPENROX_ODOMETRY_DENSE_DEPTHMAP__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

#define IMAGE_FLOAT
// With normalize the image float is between 0 and 1
//#define NORMALIZE

//! \ingroup Odometry
//! \defgroup Odometry_Dense_Depthmap Odometry Dense Depthmap 

//! \addtogroup Odometry_Dense_Depthmap 
//! @{

//! Structure 
struct Rox_Odometry_Dense_DepthMap_Struct
{
   //! Geometric info
   Rox_Array2D_Double calibration;
   
   //! The pose 
   Rox_Array2D_Double pose;

   //! Information of the reference view
   Rox_Array2D_Float image_ref;

   // The inverse depth map (in the reference frame) of each pixel of the reference image 
   Rox_Array2D_Float Zir;

   // The gradient of the inverse depth map along the u axis
   Rox_Array2D_Float Ziur;

   // The gradient of the inverse depth map along the v axis
   Rox_Array2D_Float Zivr;
   
   //! The depth_map Zr (in the reference frame) of each pixel of the reference image 
   Rox_Array2D_Float depthmap;

   //! The depth_map mask
   Rox_Imask depthmap_validity;

   //! Warped grid 
   Rox_MeshGrid2D_Float warped_grid;

   //! The warped mask
   Rox_Imask warped_grid_mask;
   
   //! To be commented  
   Rox_Imask image_cur_mask;
   
   //! Iw = Ic(warp(K, cTr, Zir , pr))
   Rox_Array2D_Float warped_image;
   
   //! The warped image mask
   Rox_Imask warped_image_validity;

   //! Intermediate buffers 
   Rox_Array2D_Float difference;
   
   //! To be commented  
   Rox_Array2D_Float weights;
   
   //! Not use, should be deleted
   Rox_Array2D_Float mean;
   
   //! mean_lum = (alpha * Iw + beta + Ir) / 2
   Rox_Array2D_Float mean_lum;
   
   //! To be commented  
   Rox_Array2D_Float Iu;
   
   //! To be commented  
   Rox_Array2D_Float Iv;
   
   //! To be commented  
   Rox_Imask gradient_mask;

   //! The linear system A matrix 
   Rox_Array2D_Double JtJ;
   
   //! The linear system inverse A matrix 
   Rox_Array2D_Double iJtJ;
   
   //! The linear system b vector
   Rox_Array2D_Double Jtf;
   
   //! To be commented  
   Rox_Array2D_Double solution;
   
   //! To be commented  
   Rox_Array2D_Double solution_pose;

   //! Parameters
   Rox_Uint max_iters;

   //! Affine illumination model: parameter alpha
   Rox_Double alpha;
   
   //! Affine illumination model: parameter alpha
   Rox_Double beta;
   
   //! To be commented  
   Rox_Uint count_pixels;
   
   //! To be commented  
   Rox_Double sum_square;
   
   //! To be commented  
   Rox_Double median;

   // The score 
   Rox_Double score;
};

//! To be commented  
typedef struct Rox_Odometry_Dense_DepthMap_Struct * Rox_Odometry_Dense_DepthMap;

//! Create a new depthmap base odometry object
//! \param  [out]  obj            the pointer to the newly created object
//! \param  [in ]  image_cols
//! \param  [in ]  image_rows
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_new (
   Rox_Odometry_Dense_DepthMap * obj, 
   Rox_Sint image_cols, 
   Rox_Sint image_rows
);

//! Delete a depthmap base odometry object
//! \param  [out]  obj the pointer to the object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_del (
   Rox_Odometry_Dense_DepthMap * obj
);

//! Set the calibration matrix for this odometry
//! \param  [out]  obj a pointer to odometry object
//! \param  [in ]  calib a 3*3 matrix with the intrinsic parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_set_calibration (
   Rox_Odometry_Dense_DepthMap obj, 
   const Rox_MatUT3 calib
);

//! Set the pose matrix cTr for this odometry
//! \param  [out]  obj a pointer to odometry object
//! \param  [in ]  pose a 4*4 matrix with the extrinsics parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_set_pose (
   Rox_Odometry_Dense_DepthMap obj, 
   const Rox_MatSE3 pose
);
//! Set the pose matrix for this odometry

ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_set_predicition (
   Rox_Odometry_Dense_DepthMap obj, 
   const Rox_MatSE3 cTc
);

//! Set the reference information for this odometry
//! \param  [out]  obj a pointer to odometry object
//! \param  [in ]  image the reference image image
//! \param  [in ]  depth the reference depth per pixel
//! \param  [in ]  mask the depth & image validity
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_set_reference_depth (
   Rox_Odometry_Dense_DepthMap obj, 
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float image, 
#else
   const Rox_Image image, 
#endif
   const Rox_Array2D_Float depth, 
   const Rox_Imask mask
);

//! Set the reference information for this odometry
//! \param  [out]  obj a pointer to odometry object
//! \param  [in ]  image the reference image image
//! \param  [in ]  depth the reference depth per pixel
//! \param  [in ]  mask the depth & image validity
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_set_reference (
   Rox_Odometry_Dense_DepthMap obj, 
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float Ir, 
#else
   const Rox_Image Ir, 
#endif   
   const Rox_Array2D_Float Zir, 
   const Rox_Array2D_Float Ziur, 
   const Rox_Array2D_Float Zivr, 
   const Rox_Imask mask
);

//! Compute Linear System for odometry
//! \param  [out]  obj            The pointer to odometry object
//! \param  [in ]  image          The current image
//! \return An error code
//! \tot should be integrated in rox_odometry_dense_depthmap_make ?
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_prepare (
   Rox_Odometry_Dense_DepthMap obj, 
#ifdef IMAGE_FLOAT 
   const Rox_Array2D_Float image
#else
   #ifdef NORMALIZE
      const Rox_Array2D_Float image
   #else
      const Rox_Image image
   #endif
#endif
);


//! Compute odometry
//! \param  [out]  obj            The pointer to odometry object
//! \param  [in ]  image          The current image 
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_make (
   Rox_Odometry_Dense_DepthMap obj, 
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float image
#else
   const Rox_Image image 
#endif
);


//! Get the pose matrix for this odometry
//! \param  [out]  pose           The 4*4 matrix with the extrinsics parameters
//! \param  [in ]  obj            The pointer to odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_get_pose (
   Rox_MatSE3 pose, 
   const Rox_Odometry_Dense_DepthMap obj
);


//! Get the results for this odometry
//! \param  [out]  success        The flag to know if the odometry was successfull (0: not success, 1: success)
//! \param  [out]  score          The score (0: very bad, 1: perfect)
//! \param  [out]  pose           The 4 x 4 matrix with the extrinsics parameters cTo
//! \param  [in ]  obj            The pointer to odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_get_results (
   Rox_Sint * success,
   Rox_Double * score,
   Rox_MatSE3 pose, 
   const Rox_Odometry_Dense_DepthMap obj
);

//! Compute odometry in both images for a stereo system
//! \param  [out]  obj_left       The pointer to left odometry object
//! \param  [in ]  image_left     The current left image image
//! \param  [in ]  obj_right      The pointer to right odometry object
//! \param  [in ]  image_right    The current right image image
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_make_both (
   Rox_Odometry_Dense_DepthMap obj_left, 
   Rox_Odometry_Dense_DepthMap obj_right, 
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float image_left, 
   const Rox_Array2D_Float image_right
#else
   const Rox_Image image_left, 
   const Rox_Image image_right
#endif
   );

//! @} 

#endif
