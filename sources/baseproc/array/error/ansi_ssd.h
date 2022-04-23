//==============================================================================
//
//    OPENROX   : File ansi_ssd.h
//
//    Contents  : API of ansi_ssd module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_SSD__
#define __OPENROX_ANSI_SSD__

//! \ingroup Vector
//! \addtogroup arrayError
//! @{

//! For each element compute sum of squares
//! \param  [out]  error          A pointer to the result L2 error 
//! \param  [in ]  error_img      The array with errors per pixel
//! \param  [in ]  mask           The input mask
//! \return An error code
int rox_ansi_array2d_float_ss_mask (
   double * ss, 
   float ** array2d_data,
   const int rows,
   const int cols,
   unsigned int ** mask_data
);

int rox_ansi_array2d_double_ssd (
   double * ssd, 
   double ** array2d_1, 
   double ** array2d_2,
   const int rows,
   const int cols
);

int rox_ansi_array2d_float_ssd (
   float * ssd, 
   float ** array2d_1, 
   float ** array2d_2,
   const int rows,
   const int cols
);

int rox_ansi_array2d_uchar_ssd (
   float * ssd, 
   unsigned char ** array2d_1, 
   unsigned char ** array2d_2,
   const int rows,
   const int cols
);

int rox_ansi_array2d_double_ssd_mask (
   double * ssd, 
   double ** array2d_1, 
   double ** array2d_2,
   const int rows,
   const int cols,
   unsigned int ** mask
);

int rox_ansi_array2d_float_ssd_mask (
   float * ssd, 
   float ** array2d_1, 
   float ** array2d_2,
   const int rows,
   const int cols,
   unsigned int ** mask
);

int rox_ansi_array2d_uchar_ssd_mask (
   float * ssd, 
   unsigned char ** array2d_1_data, 
   unsigned char ** array2d_2_data,
   const int rows,
   const int cols,
   unsigned int ** mask_data
);

//! @} 
 
#endif // __OPENROX_ANSI_SSD__
