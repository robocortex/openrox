//==============================================================================
//
//    OPENROX   : File ansi_interaction_row_texture_matse3_model3d_zi.h
//
//    Contents  : API of ansi_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <system/memory/datatypes.h>

ROX_API Rox_ErrorCode rox_ansi_interaction_row_texture_matse3_model3d_zi (
   double * L_row_data, 
   const double ur, 
   const double vr, 
   const double Iu, 
   const double Iv, 
   const double zir, 
   const double ziur, 
   const double zivr,
   double ** K_data, 
   double ** tau_data 
);