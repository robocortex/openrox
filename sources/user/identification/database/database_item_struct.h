//==============================================================================
//
//    OPENROX   : File database_item_stuct.h
//
//    Contents  : Structure of database_item module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATABASE_ITEM_STRUCT__
#define __OPENROX_DATABASE_ITEM_STRUCT__

#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_dbindex.h>

//! \ingroup Database
//! \addtogroup Database_Item
//! @{

//! The Rox_Database_Item_Struct object 
struct Rox_Database_Item_Struct
{
   //! The Ehid point list 
   Rox_DynVec_Ehid_Point   dbpoints;

   //! The Ehid index list 
   Rox_DynVec_Ehid_DbIndex dbindices;
   
   //! The template width in pixels 
   Rox_Sint cols;
   
   //! The template height in pixels 
   Rox_Sint rows;

   //! The scales
   Rox_Double scales[9];

   //! The angle max : maximum angle in degrees
   Rox_Double angle_max;

   //! The sigma : standard deviation of gaussian noise
   Rox_Double sigma;
};

//! @} 

#endif // __OPENROX_DATABASE_ITEM__ 
