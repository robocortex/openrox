//==============================================================================
//
//    OPENROX   : File edgepreproc_rgb.h
//
//    Contents  : API of edgepreproc_rgb module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGEPREPROC_RGB__
#define __OPENROX_EDGEPREPROC_RGB__

#include <system/memory/datatypes.h>

#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_point2d_sshort.h>

#include <baseproc/image/image.h>
#include <baseproc/image/image_rgba.h>


//! \ingroup Edges
//! \addtogroup EdgesPreproc
//! @{

//! The Rox_EdgePreproc_Rgb_Struct object 
struct Rox_EdgePreproc_Rgb_Struct
{
   //! The source width 
   Rox_Sint width;

   //! The source height 
   Rox_Sint height;

   //! Filter kernel
   Rox_Array2D_Float hkernel;

   //! Filter kernel
   Rox_Array2D_Float vkernel;

   //! Color plane image
   Rox_Image red;

   //! Color plane image
   Rox_Image green;

   //! Color plane image
   Rox_Image blue;

   //! Filtered image
   Rox_Image filtered;

   //! Gradients 
   Rox_Array2D_Point2D_Sshort gradients;

   //! Gradients 
   Rox_Array2D_Point2D_Sshort gradients_red;

   //! Gradients 
   Rox_Array2D_Point2D_Sshort gradients_green;

   //! Gradients 
   Rox_Array2D_Point2D_Sshort gradients_blue;
};

//! Define the pointer of the Rox_EdgePreproc_Rgb_Struct 
typedef struct Rox_EdgePreproc_Rgb_Struct * Rox_EdgePreproc_Rgb;

//! Create edge preprocessor object
//! \param  [out] obj the pointer to the object
//! \param  [in] width the width
//! \param  [in] height the height
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_edgepreproc_rgb_new(Rox_EdgePreproc_Rgb * obj, Rox_Sint width, Rox_Sint height);

//! Delete edge preprocessor  object
//! \param  [in] obj the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_edgepreproc_rgb_del(Rox_EdgePreproc_Rgb * obj);

//! Process edge preprocessir on image
//! \param  [in] obj the pointer to the object
//! \param  [in] source the image  to process (with dimensions equals to those set in object constructor)
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_edgepreproc_rgb_process(Rox_EdgePreproc_Rgb obj, Rox_Image_RGBA source);

//! @} 

#endif // __OPENROX_EDGEPREPROC_RGB__

