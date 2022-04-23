//==============================================================================
//
//    OPENROX   : File model_projector_checkerboard.h
//
//    Contents  : API of model_projector_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __MODEL_PROJECTOR_CHECKERBOARD__
#define __MODEL_PROJECTOR_CHECKERBOARD__

#include <baseproc/geometry/point/point2d_struct.h>
#include <system/memory/datatypes.h>
#include <baseproc/image/image.h>

//! \ingroup Model
//! \addtogroup Model_Projector_CheckerBoard
//! @{

//! Define the pointer of the Rox_Model_Projector_CheckerBoard_Struct
typedef struct Rox_Model_Projector_CheckerBoard_Struct * Rox_Model_Projector_CheckerBoard;

//! The model checkerboard structure
struct Rox_Model_Projector_CheckerBoard_Struct
{
   //! The number of elements along the u-axis
   Rox_Sint cols;

   //! The number of elements along the v-axis
   Rox_Sint rows;

   //! The width between two elements in pixels
   Rox_Double space_width;

   //! The height between two elements in pixels
   Rox_Double space_height;

   //! The width of the projected image
   Rox_Sint image_width;

   //! The height of the projected image
   Rox_Sint image_height;

   //! The u coordinate of the projected pattern's center
   //! if the pattern is centered the center_u = image_width/2
   Rox_Double center_u;

   //! The v coordinate of the projected pattern's center
   //! if the pattern is centered the center_v = image_height/2
   Rox_Double center_v;

   //! Elements coordinates, in pixels
   Rox_Point2D_Double_Struct * elements;
};

//! Create the model object
//! \param [out] model The pointer to the model object
//! \return            An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_model_projector_checkerboard_new(Rox_Model_Projector_CheckerBoard * model);

//! Delete the model object
//! \param [in] model The pointer to the model object
//! \return           An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_model_projector_checkerboard_del(Rox_Model_Projector_CheckerBoard * model);

//! Define the checkerboard and its dimensions
//! An element is an inner (double) corner of the board
//! \param [in] model        The pointer to the model object
//! \param [in] cols         The number of elements along the u-axis
//! \param [in] rows         The number of elements along the v-axis
//! \param [in] space_width  The space width between two elements in pixels
//! \param [in] space_height The space height between two elements in pixels
//! \param [in] image_width  The projected image width
//! \param [in] image_height The projected image height
//! \param [in] center_u     The column of the pattern center, in pixels
//! \param [in] center_v     The row of the pattern center, in pixels
//! \return                  An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_model_projector_checkerboard_set_template_decentered(
   Rox_Model_Projector_CheckerBoard model,
   Rox_Sint                         cols,
   Rox_Sint                         rows,
   Rox_Double                         space_width,
   Rox_Double                         space_height,
   Rox_Sint                         image_width,
   Rox_Sint                         image_height,
   Rox_Double                       center_u,
   Rox_Double                       center_v );

//! Define the checkerboard and its dimensions
//! The pattern is centered in the projector image
//! An element is an inner (double) corner of the board
//! \param [in] model        The pointer to the model object
//! \param [in] cols         The number of elements along the u-axis
//! \param [in] rows         The number of elements along the v-axis
//! \param [in] space_width  The space width between two elements in pixels
//! \param [in] space_height The space height between two elements in pixels
//! \param [in] image_width  The projected image width
//! \param [in] image_height The projected image height
//! \return                  An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_model_projector_checkerboard_set_template(
   Rox_Model_Projector_CheckerBoard model,
   Rox_Sint                         cols,
   Rox_Sint                         rows,
   Rox_Double                         space_width,
   Rox_Double                         space_height,
   Rox_Sint                         image_width,
   Rox_Sint                         image_height );

//! Draw checkerboard on an image according to the model values
//! \param [in] model The pointer to the model object
//! \param [in] image The image to draw on
//! \return           An error code
ROX_API Rox_ErrorCode rox_model_projector_checkerboard_generate_image(Rox_Model_Projector_CheckerBoard model, Rox_Image image);

//! @}

#endif // __MODEL_PROJECTOR_CHECKERBOARD__
