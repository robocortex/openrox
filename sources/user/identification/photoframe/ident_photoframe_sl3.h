//==============================================================================
//
//    OPENROX   : File ident_photoframe_sl3.h
//
//    Contents  : API of ident_photoframe_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_PHOTOFRAME_SL3__
#define __OPENROX_IDENT_PHOTOFRAME_SL3__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>

//! \addtogroup Identification_Photoframe
//! \brief Identification of n independent photoframes
//! @{

//! Pointer to the photoframe identifier structure 
typedef struct Rox_Ident_PhotoFrame_Struct * Rox_Ident_PhotoFrame_SL3;

//! Create a new object for identification of a plane on SL(3) with photoframes
//! \param  [out]  ident_photoframe_sl3      the newly created identification object
//! \param  [in ]  image_width               the width of the images to detect photoframes into
//! \param  [in ]  image_height              the height of the images to detect photoframes into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_new(Rox_Ident_PhotoFrame_SL3 * ident_photoframe_sl3, Rox_Sint image_width, Rox_Sint image_height);

//! Delete an identification object
//! \param [in] ident_photoframe_sl3 is the created identification object to delete
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_del(Rox_Ident_PhotoFrame_SL3 * ident_photoframe_sl3);

//! Append a new photoframe model
//! \param  [in ]  ident_photoframe_sl3      The created identification object
//! \param  [in ]  model                     The image inside the photoframe
//! \param  [in ]  border_width              The black border size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_addframe(Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Image model, Rox_Sint border_width);

//! Retrieve number of photoframes added
//! \param  [out]  count                     The pointer to the result counter
//! \param  [in ]  ident_photoframe_sl3      The created identification object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_getcountframes(Rox_Sint * count, Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3);

//! Process an input image, looking for photoframes
//! \param  [in ]  ident_photoframe_sl3      The created identification object
//! \param  [in ]  image                     The image to process
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_make(Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Image image);

//! Set the bounds for the sides of the photoframe in the current image
//! The same bounds are applied to all photoframes.
//! Value must be between [0.0, Inf]
//! Default is [10, 1000]
//! \param  [out]  ident_photoframe_sl3      The identification object
//! \param  [in ]  side_min                  The minimum length of a side in pixels
//! \param  [in ]  side_max                  The maximum length of a side in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_set_side_bounds ( Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Double side_min, Rox_Double side_max );

//! Set the bounds for the sides of the photoframe in the current image
//! The same bounds are applied to all photoframes.
//! Value must be between [0.0, Inf]
//! Default is [100, 1000000]
//! \param  [out]  ident_photoframe_sl3      The identification object
//! \param  [in ]  area_min                  The minimum area of a photoframe in pixels²
//! \param  [in ]  area_max                  The maximum area of a photoframe in pixels²
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_set_area_bounds ( Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Double area_min, Rox_Double area_max );

//! Retrieve the number of photoframes added
//! \param [out]  is_identified              The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param [out]  homography                 If the photoframe is identified, contains the homography between the reference photoframe and its pose in the current image
//! \param [in ]  ident_photoframe_sl3       The identification object
//! \param [in ]  id                         The sequential photoframe identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_getresult(Rox_Sint * is_identified, Rox_MatSL3 homography, Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Sint id);

//! Set the type of photoframe
//! \param  [out]  ident_photoframe_sl3        The identification object
//! \param  [in ]  black_to_white              The type of photoframe: set to 0 to detect white (center) inside black (around) quads, set to 1 to detect black (center) inside white (around) quads
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_set_type(Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Sint black_to_white);

//! Set score threshold for photoframe detection
//! The same threshold is applied to all photoframes.
//! Value must be between 0.0 and 1.0
//! Default is 0.9
//! \param  [out]  ident_photoframe_sl3       The identification object
//! \param  [in ]  score_threshold            The new score threshold
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_set_score_threshold ( Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Double score_threshold);

//! Set the bounds for the sides of the photoframe in the current image
//! The same bounds are applied to all photoframes.
//! Value must be between [0.0, Inf]
//! Default is [16, Inf]
//! \param  [out]  ident_photoframe_sl3      The identification object
//! \param  [in ]  side_min                  The minimum length of a side in pixels
//! \param  [in ]  side_max                  The maximum length of a side in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_set_side_bounds ( Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Double side_min, Rox_Double side_max );

//! Set the bounds for the sides of the photoframe in the current image
//! The same bounds are applied to all photoframes.
//! Value must be between [0.0, Inf]
//! Default is [16, Inf]
//! \param  [out]  ident_photoframe_sl3      The identification object
//! \param  [in ]  area_min                  The minimum area of a photoframe in pixels²
//! \param  [in ]  area_max                  The maximum area of a photoframe in pixels²
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_set_area_bounds ( Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Double area_min, Rox_Double area_max );

//! Get the score result
//! \param  [out]  score                         The score
//! \param  [in ]  ident_photoframe_se3          The identification object
//! \param  [in ]  id                            The sequential photoframe identification
//! \return An error code
//! \todo  To be tested, merge with function rox_ident_photoframe_se3_get_pose
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_getscore ( Rox_Double *score, Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, const Rox_Sint id);

//! Get the identification code nubmber
//! \param  [out]  is_decoded              The pointer to the result boolean (0 if not identified, otherwise identified)
//! \param  [out]  decoded_number          The decoded number encrypted in the photoframe
//! \param  [in ]  ident_photoframe_sl3    The identification object
//! \param  [in ]  image                   The image to process
//! \param  [in ]  code_bits               The number of bits (16 or 64) used to encrypt the code number
//! \param  [in ]  id                      The sequential id of photoframe identification
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_photoframe_sl3_get_code ( Rox_Sint * is_decoded, Rox_Sint * decoded_number, Rox_Ident_PhotoFrame_SL3 ident_photoframe_sl3, Rox_Image image, Rox_Sint code_bits, Rox_Sint id);



//! @} 

#endif
