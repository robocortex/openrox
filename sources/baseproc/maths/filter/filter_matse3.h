//=============================================================================
//
//    OPENROX   : File filter_matse3.h
//
//    Contents  : API of filter_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#ifndef __OPENROX_FILTER_MATSE3__
#define __OPENROX_FILTER_MATSE3__

//====== INCLUDED HEADERS   ===================================================

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matrix.h>

//====== EXPORTED MACROS    ===================================================

//====== INTERNAL MACROS    ===================================================

//====== EXPORTED DATATYPES ===================================================

struct Rox_Filter_MatSE3_Struct
{
   // Estimation
   Rox_MatSE3 Th;

   // Prediction
   Rox_MatSE3 Tp;

   // Correction
   Rox_MatSE3 Tc;

   // Error
   Rox_MatSE3 Te;

   // Velocity estimation
   Rox_Matrix Ah;

   // Velocity prediction
   Rox_Matrix Ap;

   // Velocity temporary
   Rox_Matrix At;

   // Time
   Rox_Double dt;

   // Inverse prediction
   Rox_MatSE3 Tp_inv;

   // Inverse error
   Rox_MatSE3 Te_inv;

   // Control gains
   Rox_Double Ka;
   Rox_Double Kg;

   // Temporary matrices
   Rox_Matrix Q;
   Rox_Matrix P;
   Rox_Matrix T;
   Rox_Matrix M;

   // Control matrices
   Rox_Matrix Wt;
   Rox_Matrix Wa;
};

typedef struct Rox_Filter_MatSE3_Struct * Rox_Filter_MatSE3;

//====== INTERNAL DATATYPES ===================================================

//====== EXPORTED FUNCTIONS ===================================================

//====== INTERNAL FUNCTIONS ===================================================

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in ]  dt             The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_new ( Rox_Filter_MatSE3 * filter_matse3, const Rox_Double dt );

//! Delete a filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_del ( Rox_Filter_MatSE3 * filter_matse3 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in ]  dt             The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_init_pose ( Rox_Filter_MatSE3 filter_matse3, const Rox_MatSE3 T0 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in ]  dt             The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_init_velocity ( Rox_Filter_MatSE3 filter_matse3, const Rox_Matrix A0 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in ]  dt             The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_update ( Rox_Filter_MatSE3 filter, const Rox_MatSE3 Tm );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in ]  dt             The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_reset ( Rox_Filter_MatSE3 filter_matse3 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in]  dt              The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_get_estimated_pose ( Rox_MatSE3 Th, const Rox_Filter_MatSE3 filter_matse3 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in]  dt              The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_get_predicted_pose ( Rox_MatSE3 Tp, const Rox_Filter_MatSE3 filter_matse3 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in]  dt              The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_get_estimated_velocity ( Rox_Matrix Ah, const Rox_Filter_MatSE3 filter_matse3 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in]  dt              The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_get_predicted_velocity ( Rox_Matrix Ap, const Rox_Filter_MatSE3 filter_matse3 );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in]  dt              The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_set_gain_pose ( Rox_Filter_MatSE3 filter_matse3, const Rox_Double Kg );

//! Create a new filter matse3
//! \param  [out]  filter_matse3  The matse3 filter
//! \param  [in]  dt              The time delta between two measures (in seconds)
//! \return An error code
ROX_API Rox_ErrorCode rox_filter_matse3_set_gain_velocity ( Rox_Filter_MatSE3 filter_matse3, const Rox_Double Ka );

#endif 
