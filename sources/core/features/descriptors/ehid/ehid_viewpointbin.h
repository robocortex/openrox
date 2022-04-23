//==============================================================================
//
//    OPENROX   : File ehid_viewpointbin.h
//
//    Contents  : API of ehid_viewpointbin module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_VIEWPOINTBIN__
#define __OPENROX_EHID_VIEWPOINTBIN__

#include <core/features/descriptors/ehid/ehid.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_segment_point.h>
#include <generated/objset_ehid_window.h>
#include <generated/dynvec_ehid_dbindex.h>

//! \addtogroup EHID
//! @{

//! Ehid object 
typedef struct Rox_Ehid_ViewpointBin_Struct * Rox_Ehid_ViewpointBin;

//! Create a new Ehid viewpoint bin object.
//! Viewpoint bin cluster many viewpoints to learn features
//! \param  [out] obj 				A pointer to the viewpoint bin created.
//! \param  [in]  image_width    	Image width
//! \param  [in]  image_height   	Image height
//! \param  [in]  minscale       	Min scale allowed
//! \param  [in]  maxscale       	Max scale allowed
//! \param  [in]  minaffine      	Min affine (out of plane) rotation
//! \param  [in]  maxaffine      	Max affine (out of plane) rotation
//! \param  [in]  sigma				The standard deviation of the gaussian noise
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_viewpointbin_new (
   Rox_Ehid_ViewpointBin * obj, 
   const Rox_Sint image_width, 
   const Rox_Sint image_height, 
   const Rox_Double minscale, 
   const Rox_Double maxscale, 
   const Rox_Double minaffine, 
   const Rox_Double maxaffine, 
   const Rox_Double sigma);

//! Delete an Ehid viewpoint bin object.
//! \param 	[out] obj 				A pointer to the viewpoint bin to delete.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_viewpointbin_del(Rox_Ehid_ViewpointBin * obj);

//! Process an image using an Ehid viewpoint bin object.
//! \param  [in] obj 				The viewpoint bin to use.
//! \param  [in] source 			The image to process (must be the size specified in the vpbin constructor)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_viewpointbin_process(Rox_Ehid_ViewpointBin obj, Rox_Image source);

//! Test an image using an Ehid viewpoint bin object.
//! \param  [out] count          	The number of blocks with enought points
//! \param  [out] count_total 		The number of blocks
//! \param  [in]  obj            	The viewpoint bin to use.
//! \param  [in]  image          	The image to test (must be the size specified in the vpbin constructor)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ehid_viewpointbin_test(Rox_Sint * count, Rox_Sint * count_total, Rox_Ehid_ViewpointBin obj, Rox_Image image);

//! @} 

#endif
