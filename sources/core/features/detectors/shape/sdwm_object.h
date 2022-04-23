//==============================================================================
//
//    OPENROX   : File sdwm_object.h
//
//    Contents  : API of sdwm_object module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SDWM_OBJECT__
#define __OPENROX_SDWM_OBJECT__

#include <system/memory/datatypes.h>
#include <generated/array2d_uchar.h>
#include <generated/objset_array2d_uchar.h>
#include <generated/objset_array2d_uchar_struct.h>
#include <generated/objset_array2d_sshort.h>
#include <generated/objset_array2d_sshort_struct.h>
#include <generated/objset_dynvec_point2d_sshort.h>
#include <generated/objset_dynvec_point2d_sshort_struct.h>
#include <generated/dynvec_fpsm_feature.h>
#include <generated/dynvec_fpsm_feature_struct.h>
#include <stdio.h>
#include <baseproc/image/image.h>

//! \ingroup Detectors
//! \addtogroup SDWM
//!   @{

//! The Rox_Sdwm_Object_Struct object 
struct Rox_Sdwm_Object_Struct
{
   //! The width of the template model 
   Rox_Sint width;
   //! The height of the template model 
   Rox_Sint height;

   //! The set of templates 
   Rox_ObjSet_Array2D_Uchar templates;
   //! The set of edge maps 
   Rox_ObjSet_Array2D_Uchar edgemaps;
   //! The set of angle maps 
   Rox_ObjSet_Array2D_Sshort anglemaps;
   //! The set of 2D points 
   Rox_ObjSet_DynVec_Point2D_Sshort pointsset;
   //! The list of fpsm features  
   Rox_DynVec_Fpsm_Feature features;
};

//! Define the pointer of the Rox_Sdwm_Object_Struct 
typedef struct Rox_Sdwm_Object_Struct * Rox_Sdwm_Object;

// Should the rox_sdwm_object be renamed rox_sdwm_item ? 
// Indeed a rox_sdwm_item is on object containing the information relative to a given template 

//! Create sdwm_object descriptor object
//! \param  [out] obj               The pointer to the object
//! \param  [in]  width_model       The width of the template model
//! \param  [in]  height_model      The height of the template model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_sdwm_object_new(Rox_Sdwm_Object * obj, Rox_Sint width_model, Rox_Sint height_model);

//! Delete an sdwm_object 
//! \param  [in ]  obj               The pointer to the object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_sdwm_object_del(Rox_Sdwm_Object * obj);

//! Add template to list
//! \param  [out]  obj               The object
//! \param  [in ]  image_template    The image template to add
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_sdwm_object_add_template ( Rox_Sdwm_Object obj, Rox_Image image_template );

//! @} 

#endif // __OPENROX_SDWM_OBJECT__

