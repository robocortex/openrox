//==============================================================================
//
//    OPENROX   : File fastst.c
//
//    Contents  : Implementation of fastst module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fastst.h"

#include "segmentpoint_tools.h"
#include "generated/dynvec_segment_point_struct.h"
#include <core/features/detectors/segment/segmentpoint_struct.h>
#include <inout/system/errors_print.h>

#define TEST_DARKER(A,B,C,D) (((*(A + C[D]))) > B)
#define IS_DARKER(A) goto is_corner
#define NOT_DARKER(A) goto check_lighter

#define TEST_LIGHTER(A,B,C,D) (((*(A + C[D]))) < B)
#define IS_LIGHTER(A) goto is_corner
#define NOT_LIGHTER(A) continue

Rox_ErrorCode rox_fastst_detector(Rox_DynVec_Segment_Point points, Rox_Image source, Rox_Sint barrier, Rox_Uint lvl)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint i = 0, j = 0;
   Rox_Uchar * ptr; // * row;
   Rox_Segment_Point_Struct toadd;
   // int nucleus, boundl;

   if (!points || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0 ;
   error = rox_array2d_uchar_get_stride(&stride, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   points->used = 0;

   {
      int tabs[16] = {
          0 + stride *  3,  1 + stride *  3,  2 + stride *  2,  3 + stride *  1,
          3 + stride *  0,  3 + stride * -1,  2 + stride * -2,  1 + stride * -3,
          0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1,
         -3 + stride *  0, -3 + stride *  1, -2 + stride *  2, -1 + stride *  3
      };

      for (i = 5; i < height - 5; i++)
      {
         Rox_Uchar * row = data[i];

         for (ptr = row + 5, j = 5; j < width - 5; ptr++,j++)
         {
            Rox_Sint nucleus = *ptr;
            Rox_Sint boundl = nucleus + barrier;

            if (TEST_DARKER(ptr, boundl, tabs, 0))
            {
               if (TEST_DARKER(ptr, boundl, tabs, 1))
               {
                  if (TEST_DARKER(ptr, boundl, tabs, 2))
                  {
                     if (TEST_DARKER(ptr, boundl, tabs, 3))
                     {
                        if (TEST_DARKER(ptr, boundl, tabs, 4))
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 5))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 6))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 7))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 8))
                                    {
                                       IS_DARKER(9);
                                    }
                                    else
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                           }
                           else
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_DARKER(ptr, boundl, tabs, 9))
                                                   {
                                                      if (TEST_DARKER(ptr, boundl, tabs, 10))
                                                      {
                                                         if (TEST_DARKER(ptr, boundl, tabs, 11))
                                                         {
                                                            IS_DARKER(16);
                                                         }
                                                         else
                                                         {
                                                            NOT_DARKER(16);
                                                         }
                                                      }
                                                      else
                                                      {
                                                         NOT_DARKER(15);
                                                      }
                                                   }
                                                   else
                                                   {
                                                      NOT_DARKER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                              else
                              {
                                 NOT_DARKER(7);
                              }
                           }
                        }
                        else
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_DARKER(ptr, boundl, tabs, 9))
                                                   {
                                                      if (TEST_DARKER(ptr, boundl, tabs, 10))
                                                      {
                                                         IS_DARKER(15);
                                                      }
                                                      else
                                                      {
                                                         NOT_DARKER(15);
                                                      }
                                                   }
                                                   else
                                                   {
                                                      NOT_DARKER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_DARKER(ptr, boundl, tabs, 9))
                                                   {
                                                      if (TEST_DARKER(ptr, boundl, tabs, 10))
                                                      {
                                                         IS_DARKER(15);
                                                      }
                                                      else
                                                      {
                                                         NOT_DARKER(15);
                                                      }
                                                   }
                                                   else
                                                   {
                                                      NOT_DARKER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                              else
                              {
                                 NOT_DARKER(7);
                              }
                           }
                           else
                           {
                              NOT_DARKER(6);
                           }
                        }
                     }
                     else
                     {
                        if (TEST_DARKER(ptr, boundl, tabs, 10))
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_DARKER(ptr, boundl, tabs, 9))
                                                   {
                                                      IS_DARKER(14);
                                                   }
                                                   else
                                                   {
                                                      NOT_DARKER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_DARKER(ptr, boundl, tabs, 9))
                                                   {
                                                      IS_DARKER(14);
                                                   }
                                                   else
                                                   {
                                                      NOT_DARKER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_DARKER(ptr, boundl, tabs, 9))
                                                   {
                                                      IS_DARKER(14);
                                                   }
                                                   else
                                                   {
                                                      NOT_DARKER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 NOT_DARKER(7);
                              }
                           }
                           else
                           {
                              NOT_DARKER(6);
                           }
                        }
                        else
                        {
                           NOT_DARKER(5);
                        }
                     }
                  }
                  else
                  {
                     if (TEST_DARKER(ptr, boundl, tabs, 9))
                     {
                        if (TEST_DARKER(ptr, boundl, tabs, 10))
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_DARKER(13);
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_DARKER(13);
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_DARKER(13);
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_DARKER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_DARKER(13);
                                                }
                                                else
                                                {
                                                   NOT_DARKER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                           }
                           else
                           {
                              NOT_DARKER(6);
                           }
                        }
                        else
                        {
                           NOT_DARKER(5);
                        }
                     }
                     else
                     {
                        NOT_DARKER(4);
                     }
                  }
               }
               else
               {
                  if (TEST_DARKER(ptr, boundl, tabs, 8))
                  {
                     if (TEST_DARKER(ptr, boundl, tabs, 9))
                     {
                        if (TEST_DARKER(ptr, boundl, tabs, 10))
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 15))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                IS_DARKER(12);
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                IS_DARKER(12);
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                IS_DARKER(12);
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                IS_DARKER(12);
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                           }
                           else
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 2))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_DARKER(ptr, boundl, tabs, 7))
                                             {
                                                IS_DARKER(12);
                                             }
                                             else
                                             {
                                                NOT_DARKER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                              else
                              {
                                 NOT_DARKER(7);
                              }
                           }
                        }
                        else
                        {
                           NOT_DARKER(5);
                        }
                     }
                     else
                     {
                        NOT_DARKER(4);
                     }
                  }
                  else
                  {
                     NOT_DARKER(3);
                  }
               }
            }
            else
            {
               if (TEST_DARKER(ptr, boundl, tabs, 7))
               {
                  if (TEST_DARKER(ptr, boundl, tabs, 8))
                  {
                     if (TEST_DARKER(ptr, boundl, tabs, 9))
                     {
                        if (TEST_DARKER(ptr, boundl, tabs, 6))
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 5))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 4))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 2))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 1))
                                       {
                                          IS_DARKER(10);
                                       }
                                       else
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 10))
                                          {
                                             IS_DARKER(11);
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 10))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 11))
                                          {
                                             IS_DARKER(11);
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 10))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 11))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 12))
                                          {
                                             IS_DARKER(11);
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 10))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 11))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 12))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 13))
                                          {
                                             IS_DARKER(11);
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                           }
                           else
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 10))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 11))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 12))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 13))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 14))
                                          {
                                             IS_DARKER(11);
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                              else
                              {
                                 NOT_DARKER(7);
                              }
                           }
                        }
                        else
                        {
                           if (TEST_DARKER(ptr, boundl, tabs, 10))
                           {
                              if (TEST_DARKER(ptr, boundl, tabs, 11))
                              {
                                 if (TEST_DARKER(ptr, boundl, tabs, 12))
                                 {
                                    if (TEST_DARKER(ptr, boundl, tabs, 13))
                                    {
                                       if (TEST_DARKER(ptr, boundl, tabs, 14))
                                       {
                                          if (TEST_DARKER(ptr, boundl, tabs, 15))
                                          {
                                             IS_DARKER(11);
                                          }
                                          else
                                          {
                                             NOT_DARKER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_DARKER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_DARKER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_DARKER(8);
                                 }
                              }
                              else
                              {
                                 NOT_DARKER(7);
                              }
                           }
                           else
                           {
                              NOT_DARKER(6);
                           }
                        }
                     }
                     else
                     {
                        NOT_DARKER(4);
                     }
                  }
                  else
                  {
                     NOT_DARKER(3);
                  }
               }
               else
               {
                  NOT_DARKER(2);
               }
            }

         check_lighter:
            boundl = nucleus - barrier;

            if (TEST_LIGHTER(ptr, boundl, tabs, 0))
            {
               if (TEST_LIGHTER(ptr, boundl, tabs, 1))
               {
                  if (TEST_LIGHTER(ptr, boundl, tabs, 2))
                  {
                     if (TEST_LIGHTER(ptr, boundl, tabs, 3))
                     {
                        if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                    {
                                       IS_LIGHTER(9);
                                    }
                                    else
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                           }
                           else
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                                                   {
                                                      if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                                      {
                                                         if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                                                         {
                                                            IS_LIGHTER(16);
                                                         }
                                                         else
                                                         {
                                                            NOT_LIGHTER(16);
                                                         }
                                                      }
                                                      else
                                                      {
                                                         NOT_LIGHTER(15);
                                                      }
                                                   }
                                                   else
                                                   {
                                                      NOT_LIGHTER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                              else
                              {
                                 NOT_LIGHTER(7);
                              }
                           }
                        }
                        else
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                                                   {
                                                      if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                                      {
                                                         IS_LIGHTER(15);
                                                      }
                                                      else
                                                      {
                                                         NOT_LIGHTER(15);
                                                      }
                                                   }
                                                   else
                                                   {
                                                      NOT_LIGHTER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                                                   {
                                                      if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                                      {
                                                         IS_LIGHTER(15);
                                                      }
                                                      else
                                                      {
                                                         NOT_LIGHTER(15);
                                                      }
                                                   }
                                                   else
                                                   {
                                                      NOT_LIGHTER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                              else
                              {
                                 NOT_LIGHTER(7);
                              }
                           }
                           else
                           {
                              NOT_LIGHTER(6);
                           }
                        }
                     }
                     else
                     {
                        if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                                                   {
                                                      IS_LIGHTER(14);
                                                   }
                                                   else
                                                   {
                                                      NOT_LIGHTER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                                                   {
                                                      IS_LIGHTER(14);
                                                   }
                                                   else
                                                   {
                                                      NOT_LIGHTER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                                                   {
                                                      IS_LIGHTER(14);
                                                   }
                                                   else
                                                   {
                                                      NOT_LIGHTER(14);
                                                   }
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 NOT_LIGHTER(7);
                              }
                           }
                           else
                           {
                              NOT_LIGHTER(6);
                           }
                        }
                        else
                        {
                           NOT_LIGHTER(5);
                        }
                     }
                  }
                  else
                  {
                     if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                     {
                        if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_LIGHTER(13);
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_LIGHTER(13);
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_LIGHTER(13);
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                                                {
                                                   IS_LIGHTER(13);
                                                }
                                                else
                                                {
                                                   NOT_LIGHTER(13);
                                                }
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                           }
                           else
                           {
                              NOT_LIGHTER(6);
                           }
                        }
                        else
                        {
                           NOT_LIGHTER(5);
                        }
                     }
                     else
                     {
                        NOT_LIGHTER(4);
                     }
                  }
               }
               else
               {
                  if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                  {
                     if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                     {
                        if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                IS_LIGHTER(12);
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                IS_LIGHTER(12);
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                IS_LIGHTER(12);
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                IS_LIGHTER(12);
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                           }
                           else
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 2))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                                          {
                                             if (TEST_LIGHTER(ptr, boundl, tabs, 7))
                                             {
                                                IS_LIGHTER(12);
                                             }
                                             else
                                             {
                                                NOT_LIGHTER(12);
                                             }
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                              else
                              {
                                 NOT_LIGHTER(7);
                              }
                           }
                        }
                        else
                        {
                           NOT_LIGHTER(5);
                        }
                     }
                     else
                     {
                        NOT_LIGHTER(4);
                     }
                  }
                  else
                  {
                     NOT_LIGHTER(3);
                  }
               }
            }
            else
            {
               if (TEST_LIGHTER(ptr, boundl, tabs, 7))
               {
                  if (TEST_LIGHTER(ptr, boundl, tabs, 8))
                  {
                     if (TEST_LIGHTER(ptr, boundl, tabs, 9))
                     {
                        if (TEST_LIGHTER(ptr, boundl, tabs, 6))
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 5))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 4))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 3))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 2))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 1))
                                       {
                                          IS_LIGHTER(10);
                                       }
                                       else
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                          {
                                             IS_LIGHTER(11);
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                    }
                                    else
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                                          {
                                             IS_LIGHTER(11);
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                 }
                                 else
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                                          {
                                             IS_LIGHTER(11);
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                              }
                              else
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                          {
                                             IS_LIGHTER(11);
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                           }
                           else
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                          {
                                             IS_LIGHTER(11);
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                              else
                              {
                                 NOT_LIGHTER(7);
                              }
                           }
                        }
                        else
                        {
                           if (TEST_LIGHTER(ptr, boundl, tabs, 10))
                           {
                              if (TEST_LIGHTER(ptr, boundl, tabs, 11))
                              {
                                 if (TEST_LIGHTER(ptr, boundl, tabs, 12))
                                 {
                                    if (TEST_LIGHTER(ptr, boundl, tabs, 13))
                                    {
                                       if (TEST_LIGHTER(ptr, boundl, tabs, 14))
                                       {
                                          if (TEST_LIGHTER(ptr, boundl, tabs, 15))
                                          {
                                             IS_LIGHTER(11);
                                          }
                                          else
                                          {
                                             NOT_LIGHTER(11);
                                          }
                                       }
                                       else
                                       {
                                          NOT_LIGHTER(10);
                                       }
                                    }
                                    else
                                    {
                                       NOT_LIGHTER(9);
                                    }
                                 }
                                 else
                                 {
                                    NOT_LIGHTER(8);
                                 }
                              }
                              else
                              {
                                 NOT_LIGHTER(7);
                              }
                           }
                           else
                           {
                              NOT_LIGHTER(6);
                           }
                        }
                     }
                     else
                     {
                        NOT_LIGHTER(4);
                     }
                  }
                  else
                  {
                     NOT_LIGHTER(3);
                  }
               }
               else
               {
                  NOT_LIGHTER(2);
               }
            }
         is_corner:
            toadd.i = i;
            toadd.j = j;
            toadd.score = 0;
            toadd.level = lvl;
            rox_dynvec_segment_point_append(points, &toadd);
         }
      }
   }

function_terminate:
   return error;
}
