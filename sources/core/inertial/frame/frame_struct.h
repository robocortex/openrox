//============================================================================
//
//    OPENROX   : File frame_struct.h
//
//    Contents  : API of frame module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_FRAME_STRUCT__
#define __OPENROX_FRAME_STRUCT__

#include <generated/array2d_double.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Maths
//! \addtogroup Frame
//! @{

//! The Rox_Frame_Struct object 
struct Rox_Frame_Struct
{
   //! Transformation w.r.t. origin 
   Rox_MatSE3 pose;

   //! Translation velocity (in m/s)
   Rox_Array2D_Double vt;
   
   //! Translation acceleration (in m/s^2)
   Rox_Array2D_Double at;

   //! Angular velocity (in rad/s)
   Rox_Array2D_Double wr;
   
   //! Acceleration (in rad/s^2)
   Rox_Array2D_Double ar;
};

//! @} 

#endif // __OPENROX_FRAME_STRUCT__ 
