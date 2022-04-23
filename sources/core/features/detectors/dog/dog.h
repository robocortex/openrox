//==============================================================================
//
//    OPENROX   : File dog.h
//
//    Contents  : API of dog module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DOG__
#define __OPENROX_DOG__

#include <generated/array2d_float.h>
#include <baseproc/geometry/rectangle/rectangle.h>

//! \ingroup Vision
//! \defgroup Detectors Detectors

//! \ingroup Detectors
//! \addtogroup DOG
//! @{

//! A dog feature description structure
struct Rox_Dog_Feature_Struct
{
   //! row of this feature in the image space
   int i;
   //! column of this feature in the image space
   int j;
   //! level of this feature in the scale space
   int lvl;
   //! octave of this feature in the scale space
   int octave;
   //! Is it a minimal or a maximal feature
   int isminimal;
   //! x coordinates in level
   float x;
   //! y coordinates in level
   float y;
   //! precise level minimized
   float flvl;
   //! scale of this feature
   float scale;
   //! scale of this feature wrt its octave
   float octave_scale;
};

//! Structure
typedef struct Rox_Dog_Feature_Struct Rox_Dog_Feature_Struct;

//! Pointer to structure
typedef struct Rox_Dog_Feature_Struct * Rox_Dog_Feature;

//! Detect features using Lowe DOG method
//! \param [out] features a pointer to a set of detected features (allocated inside function)
//! \param [out] countFeatures detected features
//! \param [in] dogspace the input dogspace
//! \param [in] image_bounds image rectangle wherein the features must lie
//! \param [in] contrast_threshold threshold for contrast around feature
//! \param [in] curvature_threshold threshold for curvature in the area
//! \param [in] preliminary_threshold minimal dog value
//! \param [in] nbintervals number of intervals used in dogspace
//! \param [in] octave id of current octave
//! \param [in] sigma the current variance
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_dogdetector_process(Rox_Dog_Feature * features, Rox_Uint * countFeatures, Rox_Array2D_Float_Collection dogspace, Rox_Rect_Sint image_bounds, Rox_Float contrast_threshold, Rox_Float curvature_threshold, Rox_Float preliminary_threshold, Rox_Uint nbintervals, Rox_Uint octave, Rox_Float sigma);

//! Create a difference of gaussian collection given a scale space collection (one output image for two input images)
//! \param [out] dogspace a newly allocated collection of images containing the d.o.g.
//! \param [in] scalespace a collection of images to substract two by two.
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_dogspace_create(Rox_Array2D_Float_Collection * dogspace, Rox_Array2D_Float_Collection scalespace);

//! @}

#endif
