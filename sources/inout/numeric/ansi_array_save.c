//==============================================================================
//
//    OPENROX   : File ansi_array_save.c
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

#include "ansi_array_save.h"

#include <inout/system/errors_print.h>

int rox_ansi_array_uint_fprint ( FILE *file, unsigned int * input_data, int size )
{
   int error = 0;

   for ( int i = 0; i < size; i++)
   {
      fprintf(file, "%3.12u ", input_data[i]);
   }
   fprintf(file, "\n");

   return error;
}

int rox_ansi_array_float_fprint ( FILE *file, float * input_data, int size )
{
   int error = 0;

   for ( int i = 0; i < size; i++)
   {
      fprintf(file, "%3.12f ", input_data[i]);
   }
   fprintf(file, "\n");

   return error;
}


int rox_ansi_array_float_save ( const Rox_Char * filename, float * input_data, int size )
{
   int error = 0;
   FILE *file = NULL;

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = 15; goto function_terminate; }

   error = rox_ansi_array_float_fprint ( file, input_data, size );

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error; 
}


int rox_ansi_array_uint_save ( const char *filename, unsigned int * input_data, int size )
{
   int error = 0;
   FILE *file = NULL;

   if ( !input_data || !filename)
   { error = 1; goto function_terminate; }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = 15; goto function_terminate; }

   error = rox_ansi_array_uint_fprint ( file, input_data, size );

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}

int rox_ansi_array_float_fscan ( float * data, int size, FILE * file )
{
   int error = 0;

   for ( int v = 0; v < size; v++ )
   {
         int nbr = 0;
         nbr = fscanf(file, "%f", &data[v]);
         if (nbr != 1) 
         { error = -1; goto function_terminate; }
   }

function_terminate: 
   return error;
}

int rox_ansi_array_uint_fscan ( unsigned int  * data, int size, FILE * file )
{
   int error = 0;

   for ( int v = 0; v < size; v++ )
   {

         int nbr = 0;
         nbr = fscanf(file, "%u", &data[v]);
         if (nbr != 1) 
         { error = -1; goto function_terminate; }
      
   }

function_terminate: 
   return error;
}
