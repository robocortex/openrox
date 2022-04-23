//==============================================================================
//
//    OPENROX   : File ehid_window_struct.h
//
//    Contents  : Structure of ehid_window module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_WINDOW_STRUCT__
#define __OPENROX_EHID_WINDOW_STRUCT__

#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_dbindex.h>

//! \addtogroup EHID
//! @{

//! Ehid Window (a block part of a template) structure 
struct Rox_Ehid_Window_Struct
{
   //! Top position of the window in the image 
   Rox_Sint top;

   //! left position of the window in the image 
   Rox_Sint left;

   //! bottom position of the window in the image 
   Rox_Sint bottom;

   //! Right position of the window in the image 
   Rox_Sint right;

   //! Height of the window in the image 
   Rox_Sint height;

   //! Width of the window in the image 
   Rox_Sint width;

   //! Features for one view 
   Rox_DynVec_Ehid_Point localpoints;

   //! Maximum number of points for this window 
   Rox_Uint maxlocalpts;

   //! Features for all views 
   Rox_Uint countlocalviews;

   //! Global list of points for all views 
   Rox_DynVec_Ehid_Point globalpoints;

   //! Clustered points (result) 
   Rox_DynVec_Ehid_Point clusteredpoints;

   //! Dbindex 
   Rox_DynVec_Ehid_DbIndex dbindices;
};

//! define 
#define ROX_TYPE_EHID_WINDOW (sizeof(struct Rox_Ehid_Window_Struct) << 2)

//! @} 

#endif

