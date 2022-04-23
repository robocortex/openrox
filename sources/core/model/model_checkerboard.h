//==============================================================================
//
//    OPENROX   : File model_checkerboard.h
//
//    Contents  : API of model_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MODEL_CHECKERBOARD__
#define __OPENROX_MODEL_CHECKERBOARD__

#include <system/memory/datatypes.h>
#include <generated/dynvec_point3d_double.h>

//! \ingroup Model
//! \addtogroup Model_CheckerBoard
//! @{

//! Define the pointer of the Rox_Model_CheckerBoard_Struct 
typedef struct Rox_Model_CheckerBoard_Struct * Rox_Model_CheckerBoard;

//! The model checkerboard structure 
struct Rox_Model_CheckerBoard_Struct
{
   //! The number of elements along the u-axis 
   Rox_Sint width;

   //! The number of elements along the v-axis 
   Rox_Sint height;

   //! The element width in meters 
   Rox_Double sizx;

   //! The element height in meters 
   Rox_Double sizy;
   
   //! The 3D points of the checkerboard organized row by row
   Rox_DynVec_Point3D_Double points3D;
};

//! Create the model object
//! \param  [out]  model          The pointer to the model object
//! \return An error code
ROX_API Rox_ErrorCode rox_model_checkerboard_new ( Rox_Model_CheckerBoard * model);

//! Delete the model object
//! \param  [out]  model          The pointer to the model object
//! \return An error code
ROX_API Rox_ErrorCode rox_model_checkerboard_del ( Rox_Model_CheckerBoard * model);

//! Define the checkerboard and its dimensions
//! An element is an inner (double) corner of the board
//! \param  [out]  model          The pointer to the model object
//! \param  [in ]  width          The number of elements along the u-axis (min 2)
//! \param  [in ]  height         The number of elements along the v-axis (min 2)
//! \param  [in ]  sizex          The element width in meters   (positive number)
//! \param  [in ]  sizey          The element height in meters  (positive number)
//! \return An error code
ROX_API Rox_ErrorCode rox_model_checkerboard_set_template (
   Rox_Model_CheckerBoard model, 
   const Rox_Sint width, 
   const Rox_Sint height, 
   const Rox_Double sizex, 
   const Rox_Double sizey
);

//! @} 

#endif // __OPENROX_MODEL_CHECKERBOARD__
