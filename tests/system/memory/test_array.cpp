//==============================================================================
//
//    OPENROX   : File test_array.cpp
//
//    Contents  : Tests for array.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =======================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/memory/array.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(array)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_new)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint size_normal = 10, size_badsize = 0;
   Rox_Array array_badtype = NULL, array_badsize = NULL, array_normal = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR, dataType_badtype = (Rox_Datatype_Description_Enum)0;
   
   // NULL pointer
   error = rox_array_new ( NULL, dataType_normal, size_normal );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // bad or unknown data type
   error = rox_array_new ( &array_badtype, dataType_badtype, size_normal );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_TYPE);
   
   // bad size
   error = rox_array_new(&array_badsize, dataType_normal, size_badsize);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   // normal case
   error = rox_array_new(&array_normal, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   if (array_normal ) rox_array_del ( &array_normal );
   if (array_badsize) rox_array_del ( &array_badsize);
   if (array_badtype) rox_array_del ( &array_badtype);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_new_frombuffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint size_normal = 10, size_badsize = 0;
   Rox_Array data_return = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR, dataType_badtype = (Rox_Datatype_Description_Enum)0;
   unsigned char buffer_normal[16];

   // normal case
   error = rox_array_new_frombuffer ( &data_return, dataType_normal, size_normal, buffer_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if (data_return)rox_array_del(&data_return);

   //NULL pointer
   error = rox_array_new_frombuffer ( NULL, dataType_normal, size_normal, buffer_normal );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   if (data_return)rox_array_del(&data_return);

   //bad or unknown data type
   error = rox_array_new_frombuffer(&data_return, dataType_badtype, size_normal, buffer_normal);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_TYPE);
   if (data_return)rox_array_del(&data_return);

   //bad size
   error = rox_array_new_frombuffer(&data_return, dataType_normal, size_badsize, buffer_normal);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
   if (data_return)rox_array_del(&data_return);

   //buffer NULL
   error = rox_array_new_frombuffer(&data_return, dataType_normal, size_normal, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   if (data_return)rox_array_del(&data_return);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array data_null = NULL, data_normal = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR;
   Rox_Uint size_normal = 10;
   
   //normal case
   error = rox_array_new(&data_normal, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array_del(&data_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //NULL pointer
   error = rox_array_del(&data_null);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_copy)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array array_null = NULL, array_normal_1 = NULL, array_normal_2 = NULL, array_badsize = NULL, array_badtype = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR, dataType_badtype = ROX_TYPE_DOUBLE;
   Rox_Uint size_normal = 4, bad_size = 5;

   //normal case
   error = rox_array_new(&array_normal_1, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array_new(&array_normal_2, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array_copy(array_normal_2, array_normal_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //NULL pointers
   error = rox_array_copy(array_normal_2, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   error = rox_array_copy(NULL, array_normal_1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   //Bad size
   error = rox_array_new(&array_badsize, dataType_normal, bad_size);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array_copy(array_normal_2, array_badsize);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
   
   error = rox_array_copy(array_badsize, array_normal_2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   //Bad type
   error = rox_array_new(&array_badtype, dataType_badtype, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array_copy(array_normal_2, array_badtype);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_TYPE);

   error = rox_array_copy(array_badtype, array_normal_2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_TYPE);

   if (array_badtype) rox_array_del(&array_badtype);
   if (array_badsize) rox_array_del(&array_badsize);
   if (array_normal_2)rox_array_del(&array_normal_2);
   if (array_normal_1)rox_array_del(&array_normal_1);
   if (array_null)rox_array_del(&array_null);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_reference)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array array_null = NULL, array_normal = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR;
   Rox_Uint size_normal = 10;

   //normal case
   error = rox_array_new(&array_normal, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array_reference(&array_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //NULL pointer
   error = rox_array_reference(&array_null);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   if (array_normal)rox_array_del(&array_normal);
   if (array_null)rox_array_del(&array_null);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array_dereference)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array array_null = NULL, array_normal = NULL, array_updown = NULL;
   Rox_Datatype_Description dataType_normal = ROX_TYPE_UCHAR;
   Rox_Uint size_normal = 10;

   //normal case
   error = rox_array_new(&array_normal, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array_dereference(&array_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_EQUAL((unsigned long) array_normal, (unsigned long) NULL);

   //NULL pointer
   error = rox_array_dereference(&array_null);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);   
   
   //up and down case, add 2 refs, at third deref, array should be release
   error = rox_array_new(&array_updown, dataType_normal, size_normal);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   unsigned long addr = (unsigned long)array_updown;
   ROX_TEST_CHECK_EQUAL((unsigned long)array_updown, addr);

   error = rox_array_reference(&array_updown);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_EQUAL((unsigned long)array_updown, addr);
   error = rox_array_reference(&array_updown);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_EQUAL((unsigned long)array_updown, addr);

   error = rox_array_dereference(&array_updown);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_EQUAL((unsigned long)array_updown, addr);
   error = rox_array_dereference(&array_updown);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_EQUAL((unsigned long)array_updown, addr);
   error = rox_array_dereference(&array_updown);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_EQUAL((unsigned long)array_updown, (unsigned long) NULL);

   if (array_normal)rox_array_del(&array_normal);
   if (array_updown)rox_array_del(&array_updown);
   if (array_null)rox_array_del(&array_null);
}

ROX_TEST_SUITE_END()
