//==============================================================================
//
//    OPENROX   : File pyramid_npot_uchar.h
//
//    Contents  : API of pyramid non power of two uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PYRAMID_NPOT_UCHAR__
#define __OPENROX_PYRAMID_NPOT_UCHAR__

#include <generated/array2d_double.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Pyramid
//!   @{

//! Pyramid structure 
struct Rox_Pyramid_Npot_Uchar_Struct
{
   //! number of levels 
   Rox_Uint nb_levels;

   //! width of the first level 
   Rox_Uint base_width;

   //! height of the first level 
   Rox_Uint base_height;

   //! Homography 
   Rox_Array2D_Double homography;

   //! Per level remapping map 
   Rox_MeshGrid2D_Float * grids;

   //! Per level image 
   Rox_Image * levels;
};

//! Pyramid structure 
typedef struct Rox_Pyramid_Npot_Uchar_Struct * Rox_Pyramid_Npot_Uchar;

//! Create a new pyramid with non power of two zoom object
//! \param  [out] obj               the pointer to the newly created object
//! \param  [in]  width             the width of the base level
//! \param  [in]  height            the height of the base level
//! \param  [in]  nb_levels         number of levels (including the base level)
//! \param  [in]  zoom_perlevel     zoom factor for each level
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_npot_uchar_new(Rox_Pyramid_Npot_Uchar * obj, Rox_Sint width, Rox_Sint height, Rox_Uint nb_levels, Rox_Double zoom_perlevel);

//! Delete a pyramid object
//! \param  [out] obj               the pointer to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_npot_uchar_del(Rox_Pyramid_Npot_Uchar * obj);

//! Apply pyramid on an image using box filter subsampling
//! \param  [in] obj                the pyramid object
//! \param  [in] source             the image to subsample
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_npot_uchar_assign(Rox_Pyramid_Npot_Uchar obj, Rox_Image source);

//! @} 

#endif // __OPENROX_PYRAMID_NPOT_UCHAR_OBJECT__
