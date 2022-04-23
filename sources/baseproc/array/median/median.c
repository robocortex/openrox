//==============================================================================
//
//    OPENROX   : File median.c
//
//    Contents  : Implementation of medianrow module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "median.h"
#include <inout/system/errors_print.h>

#define _ROX_SWAP(a,b) {register Rox_Double t=(a);(a)=(b);(b)=t;}

Rox_ErrorCode rox_array2d_double_median_select ( Rox_Double * ret_median, Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint middle, ll, hh;

   if (!ret_median || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check we are computing median on a vector (rows or cols)
   if (cols < 1 || rows < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (cols > 1 && rows > 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Retrieve first row pointer
   Rox_Double * data = NULL;
   error = rox_array2d_double_get_data_pointer ( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Find the vector size (cols or rows)
   Rox_Uint size = (cols >= rows) ? cols : rows;

   // Quick select algorithm, cf numerical recipes
   Rox_Sint low = 0;
   Rox_Sint high = size - 1;
   Rox_Sint mid = (low + high + 1) / 2; //  Added + 1 such that the middle is half+1 for even sizes 

   for (;;)
   {      
      //  One element only 
      if (high <= low)
      {
         if (size % 2)   //  size is odd 
         {  
            *ret_median = data[mid];
         }
         else            //  size is even  
         {
            //  At the end of the algorithm the half of the data is ordered in increasing order, the rest of the data is unordered 
            //  The median is 0.5*(data[mid]+data[mid-1]) 
            *ret_median = 0.5*(data[mid]+data[mid-1]);
         }
         error = ROX_ERROR_NONE;
         goto function_terminate;
      }

      //  Two elements only 
      if (high == low + 1)
      {
         if (data[low] > data[high])
         {
            _ROX_SWAP(data[low], data[high]);
         }
         
         if (size % 2)   //  size is odd 
         {  
            *ret_median = data[mid];
         }
         else            //  size is even  
         {
            *ret_median = 0.5*(data[low]+data[high]);
         }
         error = ROX_ERROR_NONE;
         goto function_terminate;
      }

      //  Find median of low, middle and high items; swap into position low 
      middle = (low + high) / 2;
      
      if (data[middle] > data[high]) _ROX_SWAP(data[middle], data[high]);
      if (data[low] > data[high]) _ROX_SWAP(data[low], data[high]);
      if (data[middle] > data[low]) _ROX_SWAP(data[middle], data[low]);

      //  Swap low item (now in position middle) into position (low+1) 
      _ROX_SWAP(data[middle], data[low + 1]);

      //  Nibble from each end towards middle, swapping items when stuck 
      ll = low + 1;
      hh = high;

      for (;;)
      {
         do ll++; while (data[low] > data[ll]);
         do hh--; while (data[hh] > data[low]);
         if (hh < ll) break;
         _ROX_SWAP(data[ll], data[hh]);
      }

      //  Swap middle item (in position low) back into correct position 
      _ROX_SWAP(data[low], data[hh]) ;

      //  Re-set active partition 
      if (hh <= mid) low = ll;
      if (hh >= mid) high = hh - 1;
   }

   // Should never pass this point ?
   error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
   
function_terminate:
   return error;
}

int compar ( const void * one, const void * two )
{
   int ret = 0;
   
   if (*(const Rox_Double*)one < *(const Rox_Double*)two)
   {
      ret = -1;
   }

   if (*(const Rox_Double*)one > *(const Rox_Double*)two)
   {
      ret = +1;
   }

   return ret;
}


// Sort array in ascending order
Rox_ErrorCode rox_array2d_double_asort ( Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
 
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // qsort(void *base, size_t nmemb, size_t size, int (*compar)(const void *, const void *));
   Rox_Double * data = NULL;
   error = rox_array2d_double_get_data_pointer ( &data, input ) ;
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint size = cols*rows;
   
   qsort(data, size, sizeof(Rox_Double), compar);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_median  (Rox_Double * ret_median, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ret_median || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_asort ( input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check we are computing median on a vector (rows or cols)
   if (cols < 1 || rows < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (cols > 1 && rows > 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Retrieve first row pointer
   Rox_Double * data = NULL;
   error = rox_array2d_double_get_data_pointer ( &data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Find the vector size (cols or rows)
   Rox_Uint size = (cols >= rows) ? cols : rows;

   Rox_Sint low = 0;
   Rox_Sint high = size - 1;
   Rox_Sint mid = (low + high + 1) / 2; //  Added + 1 such that the middle is half+1 for even sizes 

   if (size % 2)   //  size is odd 
   {  
      *ret_median = data[mid];
   }
   else            //  size is even  
   {
      //  The median is 0.5*(data[mid]+data[mid-1]) 
      *ret_median = 0.5*(data[mid]+data[mid-1]);
   }

function_terminate:
   return error;
}
