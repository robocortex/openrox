//============================================================================
//
//    OPENROX   : File filepath.c
//
//    Contents  : Implementation of filepath module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "filepath.h"

#include <string.h>
#include <stdlib.h>

#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

/////////////////////////////////////////////////////////
//
// Example:
// Given path == "C:\\dir1\\dir2\\dir3\\file.exe"
// will return dir_ as   "C:\\dir1\\dir2\\dir3"
// Will return name_ as   "file"
// Will return ext_ as    ".exe"
//
/////////////////////////////////////////////////////////
Rox_ErrorCode rox_filepath_split ( Rox_Char * dir_, Rox_Char * name_, Rox_Char * ext_, const Rox_Char * input_path )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   char * base = NULL;
   char * ext = NULL;
   char nameKeep[FILENAME_MAX];
   char pathKeep[FILENAME_MAX];
   char path[FILENAME_MAX]; //preserve original input string
   char File_Ext[FILENAME_MAX];
   char baseK[FILENAME_MAX];
   int lenFullPath, lenExt_, lenBase_;
   char *sDelim={0};
   int   iDelim=0;
   int  rel=0, i;

   if ( !input_path || !dir_ || !name_ || !ext_ )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    // copy path
    strcpy(path, input_path);


        //determine type of path string (C:\\, C:/, \\, /, ./, .\\)
        if( (strlen(path) > 1) &&

            (     ((path[1] == ':')    && (path[2] == '\\'))
               || ((path[1] == ':' )   && (path[2] == '/'))
               || (path[0] == '\\')
               || (path[0] == '/' )
               || ((path[0] == '.' ) &&   (path[1] == '/' ))
               || ((path[0] == '.' ) &&   (path[1] == '\\'))
            )
           )
        {
            sDelim = (char *) calloc(5, sizeof(char));
            /*  //   */ if(path[0] == '\\') iDelim = '\\', strcpy(sDelim, "\\");
            /*  c:\\ */ if(path[1] == ':' ) iDelim = '\\', strcpy(sDelim, "\\"); // also satisfies path[2] == '\\'
            /*  /    */ if(path[0] == '/' ) iDelim = '/' , strcpy(sDelim, "/" );
            /* ./    */ if((path[0] == '.')&&(path[1] == '/')) iDelim = '/' , strcpy(sDelim, "/" );
            /* .\\   */ if((path[0] == '.')&&(path[1] == '\\')) iDelim = '\\' , strcpy(sDelim, "\\" );
            /*  \\\\ */ if((path[0] == '\\')&&(path[1] == '\\')) iDelim = '\\', strcpy(sDelim, "\\");
            if(path[0]=='.')
            {
                rel = 1;
                path[0]='*';
            }

            if(!strstr(path, "."))  // if no filename, set path to have trailing delim,
            {                      //set others to "" and return
                lenFullPath = (int) strlen(path);
                if(path[lenFullPath-1] != iDelim)
                {
                    strcat(path, sDelim);
                    dir_[0]=0;
                    name_[0]=0;
                    ext_[0]=0;
                }
            }
            else
            {
                nameKeep[0]=0;         //works with C:\\dir1\file.txt
                pathKeep[0]=0;
                // original_path[0]=0;        //preserves *path
                File_Ext[0]=0;
                baseK[0]=0;

                // Get lenth of full path
                lenFullPath = (int) strlen(path);

                strcpy(nameKeep, path);
                strcpy(pathKeep, path);
                //strcpy(original_path, path);
                strcpy(dir_, path); //capture path

                // Get length of extension:
                for(i=lenFullPath-1;i>=0;i--)
                {
                    if(pathKeep[i]=='.') break;
                }
                lenExt_ = (lenFullPath - i) -1;

                base = strtok(path, sDelim);
                while(base)
                {
                    strcpy(File_Ext, base);
                    base = strtok(NULL, sDelim);
                }

                strcpy(baseK, File_Ext);
                lenBase_ = (int) (strlen(baseK) - lenExt_);
                baseK[lenBase_-1]=0;
                strcpy(name_, baseK);

                dir_[lenFullPath -lenExt_ -lenBase_ -1] = 0;

                ext = strtok(File_Ext, ".");
                strcpy(ext_, "");
                while( ext != NULL )
                {
                    ext = strtok(NULL, ".");
                    // if ext is not null copy into ext
                    if(ext) sprintf(ext_, ".%s", ext); // keep the dot
                }
            }

            // Reset path to initial value
            //memset(path, 0, lenFullPath);
            // strcpy(path, original_path);

            if(rel) dir_[0] = '.';//replace first "." for relative path
            free(sDelim);
        }

function_terminate:
  return error;
}


Rox_ErrorCode rox_filepath_extract(Rox_Char * dir, Rox_Char * name, Rox_Char * ext, const Rox_Char * filepath)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   //Rox_Char drive[FILENAME_MAX];
   //size_t driveNumberOfElements = FILENAME_MAX;
   //size_t dirNumberOfElements = FILENAME_MAX;
   //size_t nameNumberOfElements = FILENAME_MAX;
   //size_t extNumberOfElements = FILENAME_MAX;
   //error = (Rox_ErrorCode) _splitpath_s(filepath, drive, driveNumberOfElements, dir, dirNumberOfElements, name, nameNumberOfElements, ext, extNumberOfElements);

   error = rox_filepath_split(dir, name, ext, filepath);
   if (error) error = ROX_ERROR_EXTERNAL;

   return error;
}
