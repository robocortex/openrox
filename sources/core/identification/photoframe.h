//==============================================================================
//
//    OPENROX   : File photoframe.h
//
//    Contents  : API of photoframe module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PHOTOFRAME__
#define __OPENROX_PHOTOFRAME__

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <core/model/model_single_plane.h>

//! \ingroup  Identification_Photoframe
//! \defgroup Photoframe Photoframe
//! \brief Photoframe structures and methods.

//! \addtogroup Photoframe
//! @{

//! Information about photoframe identification pointer
typedef struct Rox_PhotoFrame_Struct * Rox_PhotoFrame;


//! Create a new object for identification of a plane
//! \param  [out]  photoframe     The newly created identification object
//! \param  [in ]  reference      A square reference template image (width == height)
//! \param  [in ]  border_width   The black border size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_new ( Rox_PhotoFrame * photoframe, Rox_Image reference, Rox_Uint border_width );

ROX_API Rox_ErrorCode rox_photoframe_new_model ( Rox_PhotoFrame * photoframe, const Rox_Model_Single_Plane model_single_plane, const Rox_Sint border_width );

//! Set score threshold
//! If the score is above this threshold the photoframe is selected and the search stops
//! Default score_threshold = 0.9
//! \param  [in ]  photoframe              The identification object to set
//! \param  [in ]  score_threshold         The new score threshold
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_set_score_threshold ( Rox_PhotoFrame photoframe, Rox_Double score_threshold );

//! Set the photoframe size in meters
//! \param  [out]  photoframe     The identification object to set
//! \param  [in ]  width_meters   The photoframe width in meters
//! \param  [in ]  height_meters  The photoframe height in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_set_size(Rox_PhotoFrame photoframe, Rox_Double width_meters, Rox_Double height_meters);

// //! Set the photoframe bound 
// //! \param  [out]  photoframe           The identification object to set
// //! \param  [in ]  side_min              The photoframe min side of in pixels
// //! \param  [in ]  side_max              The photoframe min side of in pixels
// //! \param  [in ]  area_min              
// //! \param  [in ]  area_max
// //! \return An error code
// //! \todo   To be tested
// ROX_API Rox_ErrorCode rox_photoframe_set_bounds(Rox_PhotoFrame photoframe, Rox_Double size_min, Rox_Double size_max, Rox_Double area_min, Rox_Double area_max);

//! Set a mask to the photoframe
//! \param  [in ]  photoframe            The identification object to set
//! \param  [in ]  mask                  The photoframe width in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_set_mask(Rox_PhotoFrame photoframe, Rox_Imask mask);


//! Delete a photoframe identification object
//! \param  [in ]  photoframe            The created identification object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_del(Rox_PhotoFrame * photoframe);


//! Mark a photoframe identification object as available for detection
//! \param  [in ]  photoframe            The created identification object to mark
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_reset(Rox_PhotoFrame photoframe);


//! Make a photoframe identification on an input image using fast orientation check
//! \param  [in ]  photoframe            The created identification object to mark
//! \param  [in ]  image                 The input image where the photoframe may be
//! \param  [in ]  H                     The homography where the photoframe "outside" should lie (warp a [0,1] x [0,1] region to the quadrilateral around the photoframe)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_make_with_fast_orientation_test (
   Rox_PhotoFrame photoframe, Rox_Image image, Rox_Matrix H);


//! Make a photoframe identification on an input image using slow orientation test (test all 4 possible orientation)
//! \param  [in ]  photoframe            The created identification object to mark
//! \param  [in ]  image                 The input image where the photoframe may be
//! \param  [in ]  H                     The homography where the photoframe "outside" should lie (warp a [0,1] x [0,1] region to the quadrilateral around the photoframe)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_make_with_full_orientation_test (
   Rox_PhotoFrame photoframe, Rox_Image image, Rox_Matrix H);


//! Make a photoframe identification on an input image
//! \param  [in ]  photoframe            The created identification object to mark
//! \param  [in ]  image                 The input image where the photoframe may be
//! \param  [in ]  H                     The homography where the photoframe "outside" should lie (warp a 1*1 region to the quadrilateral around the photoframe)
//! \param  [in ]  calibration           The camera calibration
//! \param  [in ]  orientation_method    Method used (0 for slower exhaustive orientation test, 1 for faster orientation detection)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_photoframe_make_se3 ( Rox_PhotoFrame photoframe, Rox_Image image, Rox_Matrix H, Rox_Matrix calibration, Rox_Uint orientation_method);

//! @}

#endif
