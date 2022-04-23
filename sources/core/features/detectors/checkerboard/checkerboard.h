//==============================================================================
//
//    OPENROX   : File checkerboard.h
//
//    Contents  : API of checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CHECKERBOARD__
#define __OPENROX_CHECKERBOARD__

#include <generated/array2d_point2d_double.h>
#include <generated/array2d_uint.h>
#include <baseproc/image/image.h>

//! \ingroup Camera_Calibration
//! \addtogroup CheckerBoard
//! @{

//! Checkerboard structure
struct Rox_CheckerBoard_Struct
{
   //! Checkerboard points
   Rox_Array2D_Point2D_Double points;

   //! Checkerboard indices
   Rox_Array2D_Uint indices;

   //! Energy
   Rox_Double energy;
};


//! checkerboard object
typedef struct Rox_CheckerBoard_Struct * Rox_CheckerBoard;


//!  Create a container object for a checkerboard
//! \param checkerboard the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_checkerboard_new(Rox_CheckerBoard * checkerboard);


//!  Delete a container object for a checkerboard
//! \param checkerboard the container pointer to delete
//! \return An error code
ROX_API  Rox_ErrorCode rox_checkerboard_del(Rox_CheckerBoard * checkerboard);


//!  Set the checkerboard size
//! \param checkerboard  The container pointer
//! \param height        The height of the checkerboard: the number of corner along the y direction
//! \param width         The width  of the checkerboard: the number of corner along the x direction
//! \return An error code
ROX_API Rox_ErrorCode rox_checkerboard_set_size (
   Rox_CheckerBoard checkerboard, 
   Rox_Sint height, 
   Rox_Sint width
);


//!  Compute energy for a checkerboard
//! \param checkerboard  The checkerboard to process
//! \return An error code
ROX_API  Rox_ErrorCode rox_checkerboard_compute_energy(Rox_CheckerBoard checkerboard);


//!  Check order for a checkerboard (if this checkerboard has a coding)
//! \param checkerboard  The checkerboard to process
//! \param source        Image source
//! \return An error code
ROX_API Rox_ErrorCode rox_checkerboard_check_order(Rox_CheckerBoard checkerboard, Rox_Image source);

//! @}

#endif // __OPENROX_CHECKERBOARD__
