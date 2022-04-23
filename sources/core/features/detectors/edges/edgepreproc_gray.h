//==============================================================================
//
//    OPENROX   : File edgepreproc_gray.h
//
//    Contents  : API of edgepreproc_gray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGEPREPROC_GRAY__
#define __OPENROX_EDGEPREPROC_GRAY__

#include <system/memory/datatypes.h>

#include <generated/array2d_float.h>
#include <generated/array2d_point2d_sshort.h>

#include <baseproc/image/image.h>

//! \ingroup Edges
//! \addtogroup EdgesPreproc
//! @{

//! The Rox_EdgePreproc_Gray_Struct object
struct Rox_EdgePreproc_Gray_Struct
{
   //! The source width 
   Rox_Sint width;

   //! The source height 
   Rox_Sint height;

   //! Filter kernel
   Rox_Array2D_Float hkernel;

   //! Filter kernel
   Rox_Array2D_Float vkernel;

   //! Filtered image
   Rox_Image filtered;

   //! Gradients 
   Rox_Array2D_Point2D_Sshort gradients;
};

//! Define the pointer of the Rox_EdgePreproc_Gray_Struct 
typedef struct Rox_EdgePreproc_Gray_Struct * Rox_EdgePreproc_Gray;

//! Create edge preprocessor object
//! \param  [out] obj the pointer to the object
//! \param  [in] width the width
//! \param  [in] height the height
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_edgepreproc_gray_new(Rox_EdgePreproc_Gray * obj, Rox_Sint width, Rox_Sint height);

//! Delete edge preprocessor  object
//! \param  [in] obj the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_edgepreproc_gray_del(Rox_EdgePreproc_Gray * obj);

//! Process edge preprocessor on image
//! \param  [in] obj					The pointer to the object
//! \param  [in] image				The image to process (with dimensions equals to those set in object constructor)
//! \param  [in] nbr_blur_passes	Number of gradientsobel pass (blur) needed 0 (no blur) to n
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_edgepreproc_gray_process(Rox_EdgePreproc_Gray obj, Rox_Image image, Rox_Uint nbr_blur_passes);

//! @} 

#endif // __OPENROX_EDGEPREPROC_GRAY__

