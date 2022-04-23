//==============================================================================
//
//    OPENROX   : File ident_photoframe_se3.h
//
//    Contents  : API of ident_photoframe_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_PHOTOFRAME_SE3__
#define __OPENROX_IDENT_PHOTOFRAME_SE3__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

#include <core/model/model_single_plane.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Identification
//! \addtogroup Identification_Photoframe
//! @{

//! Photoframe identifier structure pointer
typedef struct Rox_Ident_PhotoFrame_Struct * Rox_Ident_PhotoFrame_SE3;

//! Create a new object for identification of a plane on SE(3) with photoframes
//! \param  [out]  ident_photoframe_se3          The newly created identification object
//! \param  [in ]  image_width                   The width of the images to detect photoframes into
//! \param  [in ]  image_height                  The height of the images to detect photoframes into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_new ( 
   Rox_Ident_PhotoFrame_SE3 *ident_photoframe_se3, 
   const Rox_Sint image_width, 
   const Rox_Sint image_height
);

//! Delete an identification object
//! \param  [out]   ident_photoframe_se3          The created identification object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_del (
   Rox_Ident_PhotoFrame_SE3 * ident_photoframe_se3
);

//! Append a new photoframe 
//! \param  [out]   ident_photoframe_se3          The created identification object
//! \param  [in ]   image_model                   The image inside the photoframe
//! \param  [in ]   width_meter                   The model width in meters
//! \param  [in ]   height_meter                  The model height in meters
//! \param  [in ]   border_width                  The black border size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_addframe (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Image image_model, 
   const Rox_Double width_meter, 
   const Rox_Double height_meter, 
   const Rox_Sint border_width
);

//! Append a new photoframe model
//! \param  [out]  ident_photoframe_se3          The created identification object
//! \param  [in ]  model                         The image inside the photoframe
//! \param  [in ]  border_width                  The black border size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_addframe_model ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Model_Single_Plane model_single_plane, 
   const Rox_Sint border_width
);

//! Append a new masked photoframe model
//! \param  [out]  ident_photoframe_se3            The created identification object
//! \param  [in ]  model                           The image inside the photoframe
//! \param  [in ]  mask                            Mask to apply to the photoframe
//! \param  [in ]  width_meter                     The model width in meters
//! \param  [in ]  height_meter                    The model height in meters
//! \param  [in ]  border_width                    The black border size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_addframe_masked ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   // const Rox_Image model, 
   // const Rox_Double width_meter, 
   // const Rox_Double height_meter,
   const Rox_Model_Single_Plane model_single_plane, 
   const Rox_Sint border_width,
   const Rox_Imask mask
);

//! Retrieve number of photoframes added
//! \param  [out]  count                            The pointer to the result counter
//! \param  [in ]  ident_photoframe_se3             The created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_getcountframes ( 
   Rox_Sint * count, 
   const Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3
);

//! Process an input image, looking for photoframes
//! \param  [out]  ident_photoframe_se3             The created identification object
//! \param  [in ]  camera                            Containing the image to process
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_make (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Camera camera
);

//! Set orientation method for photoframe identification.
//! Once the quad is detected, the template inside may have one angle among four.
//! Select the method to solve this problem.
//!   Method 0 is the slowest but most robust.
//!   Method 1 is faster but less robust.
//! \param  [out]  ident_photoframe_se3          The created identification object
//! \param  [in ]  orientation_method             Method to use for orientation problem
//! \return An error code
//! \todo   To be tested
//! \todo   Transform "method" in Enum
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_orientation_method (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Sint orientation_method
);

//! Set the quad detection method for photoframe identification.
//!   Method 0 is faster but less robust.
//!   Method 1 is the slowest but most robust.
//! \param  [out]  ident_photoframe_se3              The created identification object
//! \param  [in ]  detection_method                  The method to use to extract the quad
//! \return An error code
//! \todo   To be tested
//! \todo   Transform "detection_method" in Enum
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_detection_method ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Sint detection_method
);

//! Set the type of photoframe
//! \param  [out]  ident_photoframe_se3             The identification object
//! \param  [in ]  black_to_white                   The type of photoframe: set to 0 to detect white (center) inside black (around) quads, set to 1 to detect black (center) inside white (around) quads
//! \return An error code
//! \todo   To be tested
//! \todo   Transform "black_to_white" in Enum = {black_inside_white, white_inside_black}
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_type ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Sint black_to_white
);

//! Set the type of photoframe
//! \param  [out]  ident_photoframe_se3             The identification object
//! \param  [in ]  segment_length_min               The minimum length of a segment in the image for quad detection
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_segment_length_min ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Double segment_length_min
);

//! Set the type of photoframe
//! \param  [out]  ident_photoframe_se3             The identification object
//! \param  [in ]  segment_length_min               The minimum length of a segment in the image for quad detection
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_segment_length_min ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Double segment_length_min
);

//! Set score threshold for photoframe detection
//! The same threshold is applied to all photoframes.
//! Value must be between 0.0 and 1.0
//! Default is 0.9
//! \param  [out]  ident_photoframe_se3             The identification object
//! \param  [in ]  threshold                        The new score threshold
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_score_threshold ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Double threshold
);

//! Set the bounds for the sides of the photoframe in the current image
//! The same bounds are applied to all photoframes.
//! Value must be between [0.0, Inf]
//! Default is [10, 1000]
//! \param  [out]  ident_photoframe_se3      The identification object
//! \param  [in ]  side_min                  The minimum length of a side in pixels
//! \param  [in ]  side_max                  The maximum length of a side in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_side_bounds ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   Rox_Double side_min, 
   Rox_Double side_max 
);

//! Set the bounds for the sides of the photoframe in the current image
//! The same bounds are applied to all photoframes.
//! Value must be between [0.0, Inf]
//! Default is [100, 1000000]
//! \param  [out]  ident_photoframe_se3      The identification object
//! \param  [in ]  area_min                  The minimum area of a photoframe in pixels²
//! \param  [in ]  area_max                  The maximum area of a photoframe in pixels²
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_set_area_bounds ( 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   Rox_Double area_min, 
   Rox_Double area_max 
);

//! Get the identification result
//! \param  [out]  is_identified                   The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out]  pose                            If the photoframe is identified, contains the pose between the reference photoframe and its pose in the current image
//! \param  [in ]  ident_photoframe_se3            The identification object
//! \param  [in ]  id                              The sequential photoframe identification
//! \return An error code
//! \todo   To be tested and renamed to rox_ident_photoframe_se3_get_pose
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_get_pose ( 
   Rox_Sint * is_identified, 
   Rox_MatSE3 pose, 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Sint id
);

//! Get the score result
//! \param  [out]  score                         The score
//! \param  [in ]  ident_photoframe_se3          The identification object
//! \param  [in ]  id                            The sequential photoframe identification
//! \return An error code
//! \todo   To be tested, merge with function rox_ident_photoframe_se3_get_pose
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_get_score (
   Rox_Double *score, 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   const Rox_Sint id
);

//! Get the result
//! \param  [out]  is_identified                 The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out]  score                         The score
//! \param  [out]  pose                          If the photoframe is identified, contains the pose between the reference photoframe and its pose in the current image
//! \param  [in ]  ident_photoframe_se3          The identification object
//! \param  [in ]  id                            The sequential photoframe identification
//! \return An error code
//! \todo   To be tested, merge with function rox_ident_photoframe_se3_get_pose
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_get_result (
   Rox_Sint * is_identified, 
   Rox_Double * score, 
   Rox_MatSE3 pose, 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   Rox_Sint id
);

//! Get the identification code number
//! \param  [out]  is_decoded                    The pointer to the result boolean (0 if not decoded, otherwise decoded)
//! \param  [out]  decoded_number                The code number encrypted in the photoframe
//! \param  [in ]  ident_photoframe_se3          The identification object
//! \param  [in ]  camera                        The camera containing the image to process
//! \param  [in ]  code_bits                     The number of bits (16 or 64) used to encrypt the code number
//! \param  [in ]  id                            The sequential id of photoframe identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_get_code (
   Rox_Sint * is_decoded, 
   Rox_Sint * decoded_number, 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3, 
   Rox_Camera camera, 
   Rox_Sint code_bits, 
   Rox_Sint id
);

// Get the best score
ROX_API Rox_ErrorCode rox_ident_photoframe_se3_get_best_result (
   Rox_Sint * identified, 
   Rox_Sint * best_photoframe_id, 
   Rox_Double * best_score,
   Rox_MatSE3 best_pose, 
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3
);

//! @}

#endif
