//==============================================================================
//
//    OPENROX   : File ansi_array2d_save.c
//
//    Contents  : Implementation of ansi array2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_array2d_save.h"

#include <inout/system/errors_print.h>

int rox_ansi_array2d_uint_fprint ( FILE *file, unsigned int ** input_data, int rows, int cols )
{
   int error = 0;

   for ( int i = 0; i < rows; i++)
   {
      for ( int j = 0; j < cols; j++)
      {
         fprintf(file, "%3.12u ", input_data[i][j]);
      }
      fprintf(file, "\n");
   }

   return error;
}

int rox_ansi_array2d_float_fprint ( FILE *file, float ** input_data, int rows, int cols )
{
   int error = 0;

   for ( int i = 0; i < rows; i++)
   {
      for ( int j = 0; j < cols; j++)
      {
         fprintf(file, "%3.12f ", input_data[i][j]);
      }
      fprintf(file, "\n");
   }

   return error;
}


int rox_ansi_array2d_float_save ( const Rox_Char * filename, float ** input_data, int rows, int cols )
{
   int error = 0;
   FILE *file = NULL;

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = 15; goto function_terminate; }

   error = rox_ansi_array2d_float_fprint ( file, input_data, rows, cols );

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error; 
}


int rox_ansi_array2d_uint_save ( const char *filename, unsigned int ** input_data, int rows, int cols)
{
   int error = 0;
   FILE *file = NULL;

   if ( !input_data || !filename)
   { error = 1; goto function_terminate; }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = 15; goto function_terminate; }

   error = rox_ansi_array2d_uint_fprint ( file, input_data, rows, cols );

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}


int rox_ansi_array2d_float_fscan_transpose ( float ** data, int rows, int cols, FILE * file )
{
   int error = 0;

   for ( int v = 0; v < rows; v++ )
   {
      for ( int u = 0; u < cols; u++ )
      {
         int nbr = 0;
         nbr = fscanf(file, "%f", &data[u][v]);
         if (nbr != 1) 
         { error = -1; goto function_terminate; }
      }
   }

function_terminate: 
   return error;
}


int rox_ansi_array2d_float_fscan ( float ** data, int rows, int cols, FILE * file )
{
   int error = 0;

   for ( int v = 0; v < rows; v++ )
   {
      for ( int u = 0; u < cols; u++ )
      {
         int nbr = 0;
         nbr = fscanf(file, "%f", &data[v][u]);
         if (nbr != 1) 
         { error = -1; goto function_terminate; }
      }
   }

function_terminate: 
   return error;
}


int rox_ansi_array2d_float_padded_fscan ( float ** data, int rows, int cols, int used, FILE * file )
{
   int error = 0;

   for ( int v = 0; v < rows; v++ )
   {
      for ( int u = 0; u < used; u++ )
      {
         int nbr = 0;
         nbr = fscanf(file, "%f", &data [v][u]);
         if (nbr != 1) 
         { error = -1; goto function_terminate; }
      }
   }

function_terminate: 
   return error;
}


int rox_ansi_array2d_uint_fscan ( unsigned int  ** data, int rows, int cols, FILE * file )
{
   int error = 0;

   for ( int v = 0; v < rows; v++ )
   {
      for ( int u = 0; u < cols; u++ )
      {
         int nbr = 0;
         nbr = fscanf(file, "%u", &data[v][u]);
         if (nbr != 1) 
         { error = -1; goto function_terminate; }
      }
   }

function_terminate: 
   return error;
}
