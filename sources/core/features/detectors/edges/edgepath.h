//==============================================================================
//
//    OPENROX   : File edgepath.h
//
//    Contents  : API of edgepath module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGEPATH__
#define __OPENROX_EDGEPATH__

#include <system/memory/datatypes.h>

//! \ingroup Detectors
//! \addtogroup Edges
//! @{

//! The Rox_EdgeAnchor_Struct object
struct Rox_EdgeTurn_Struct
{
   //! Point U 
    
   Rox_Uint u;
   
   //! Point V
    
   Rox_Uint v;
   
   //! Point cardinal direction (0 left, 1 right, 2 top, 3 bottom)
    
   Rox_Uint direction;
};

//! Define the pointer of the Rox_EdgeTurn_Struct 
typedef struct Rox_EdgeTurn_Struct Rox_EdgeTurn_Struct;
 
//! The Rox_EdgeAnchor_Struct object

struct Rox_Edgel_Struct
{
   //! Point U 
    
   Rox_Uint u;
   
   //! Point V
    
   Rox_Uint v;
   
   //! Edgel score
    
   Rox_Uint score;
};

//! Define the pointer of the Rox_Edgel_Struct 
typedef struct Rox_Edgel_Struct Rox_Edgel_Struct;

//! The Rox_EdgeAnchor_Struct object

struct Rox_Segment_Part_Struct
{
   //! Edgel list
   Rox_Edgel_Struct * data;

   //! Count edgels
   Rox_Uint length;
};

//! Define the pointer of the Rox_Edgel_Struct 
typedef struct Rox_Segment_Part_Struct Rox_Segment_Part_Struct;

//! @} 

#endif // __OPENROX_EDGEPATH__
