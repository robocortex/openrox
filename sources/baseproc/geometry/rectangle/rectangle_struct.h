//==============================================================================
//
//    OPENROX   : File rectangle_struct.h
//
//    Contents  : Structure of rectangle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RECTANGLE_STRUCT__
#define __OPENROX_RECTANGLE_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup Rectangle
//! @{

//! Rectangle described with integers coordinates and sizes 
struct Rox_Rect_Sint_Struct
{
   //! x coordinate of top left corner
   Rox_Sint x;
   //! y coordinate of top left corner 
   Rox_Sint y;
   //! width 
   Rox_Sint width;
   //! height 
   Rox_Sint height;
};

//! Alias to rectangle structure
typedef struct Rox_Rect_Sint_Struct Rox_Rect_Sint_Struct;

//! Rectangle described with doubles coordinates and sizes 
struct Rox_Rect_Real_Struct
{
   //! x coordinate of top left corner 
   Rox_Double x;
   //! y coordinate of top left corner 
   Rox_Double y;
   //! width 
   Rox_Double width;
   //! height 
   Rox_Double height;
};

//! Alias to rectangle structure
typedef struct Rox_Rect_Real_Struct Rox_Rect_Real_Struct;

//! @} 

#endif
