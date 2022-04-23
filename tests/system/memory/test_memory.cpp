//==============================================================================
//
//    OPENROX   : File test_memory.cpp
//
//    Contents  : Tests for memory.c
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
   #include <limits.h>
   #include <system/memory/datatypes.h>
   #include <system/memory/memory.h>
   #include <system/errors/errors.h>
   #include <inout/system/print.h>
}


//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(memory)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_memory_allocate)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   Rox_Ptr        pointer = NULL;
   const Rox_Size size_unit = 1;
   const Rox_Size size_zero = 0;
   const Rox_Size size_overflow = SIZE_MAX;

   //void * rox_memory_allocate(const Rox_Size element_size, const Rox_Size element_count)
   //expect an allocation
   pointer = (Rox_Ptr)rox_memory_allocate(size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;

   //expect null on zero size
   pointer = (Rox_Ptr)rox_memory_allocate(size_zero, size_zero);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;

   //expect null on overflow
   pointer = (Rox_Ptr)rox_memory_allocate(size_overflow, size_overflow);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_memory_reallocate)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ptr        pointer = NULL;
   Rox_Ptr        pointer_realloc = NULL;
   const Rox_Size size_unit = 1;
   const Rox_Size size_zero = 0;
   // const Rox_Size size_overflow = SIZE_MAX;

   //void * rox_memory_reallocate(void *pointer, const Rox_Size element_size, const Rox_Size element_count)
   
   //expect a re-allocation
   pointer = (Rox_Ptr)rox_memory_allocate(size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   pointer_realloc = (Rox_Ptr)rox_memory_reallocate(pointer, size_unit + size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer_realloc);
   pointer = NULL;
   pointer_realloc = NULL;

   //expect a re-allocation for same size (standard does not guarantee same address, GNU libc does though)
   pointer = (Rox_Ptr)rox_memory_allocate(size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   pointer_realloc = (Rox_Ptr)rox_memory_reallocate(pointer, size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer_realloc);
   pointer = NULL;
   pointer_realloc = NULL;

   //expect a re-allocation on inferior size (standard does not guarantee same address, GNU libc does though)
   pointer = (Rox_Ptr)rox_memory_allocate(size_unit + size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   pointer_realloc = (Rox_Ptr)rox_memory_reallocate(pointer, size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer_realloc);
   pointer = NULL;

   //expect null on zero
   pointer = (Rox_Ptr)rox_memory_allocate(size_unit, size_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);

   pointer_realloc = (Rox_Ptr)rox_memory_reallocate(pointer, size_zero, size_zero);
   ROX_TEST_CHECK_EQUAL(pointer_realloc, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;

   //expect null on null pointer
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate(pointer, size_zero, size_zero);
   ROX_TEST_CHECK_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);
   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_memory_delete)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ptr        pointer = NULL;
   // const Rox_Size size_unit = 1;

   //void rox_memory_delete(void * pointer)

   //expect nothing ever, just no crash ("undefined behavior" if pointer was not allocated through malloc family)
   rox_memory_delete(pointer);

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_memory_allocate_aligned)
{
   Rox_ErrorCode     error = ROX_ERROR_NONE;
   Rox_Ptr           pointer = NULL;
   Rox_Ptr           pointer_aligned = NULL;
   const Rox_Size    size_unit = 1;
   const Rox_Size    size_zero = 0;
   const Rox_Size    size_overflow = SIZE_MAX;
   // const Rox_Size    size_unaligned = 361;
   const Rox_Uchar   alignment_bytes_zero = 0;
   const Rox_Uchar   alignment_bytes_unit = 1;
   // const Rox_Uchar   alignment_bytes_over = UCHAR_MAX;

   //void * rox_memory_allocate_aligned(void **aligned_adress, const Rox_Size element_size, const Rox_Size element_count, const Rox_Uchar alignment_bytes)
   
   // expect null on aligned null pointer address
   pointer = (Rox_Ptr)rox_memory_allocate_aligned(NULL, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);

   // expect null on 0 alignment pointer
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_zero);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_EQUAL(pointer_aligned, (Rox_Ptr)NULL);

   //expect an allocation
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_NOT_EQUAL(pointer_aligned, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;

   // expect null on zero size
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_zero, size_zero, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;

   // expect null on overflow
   pointer = (Rox_Ptr) rox_memory_allocate_aligned((void**)&pointer_aligned, size_overflow, size_overflow, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_EQUAL(pointer_aligned, (Rox_Ptr)NULL);

   rox_memory_delete(pointer);
   pointer = NULL;
   
   // TEsts
#ifdef test_alignement
   float * pointer_float = NULL;

   pointer = (Rox_Ptr) rox_memory_allocate_aligned((void**)&pointer_float, sizeof(float), 1, 16);

   rox_log("sizeof(pointer_aligned) = %lu\n", sizeof(pointer_float));
   rox_log("sizeof(pointer_aligned[0]) = %lu\n", sizeof(pointer_float[0]));
   rox_log("size of 1 memory aligned on 16 bytes : %lu\n", sizeof(pointer_float) / sizeof(pointer_float[0]));

   #define ALIGNMENT_VALUE     32u

   if (((uintptr_t)pointer_float % ALIGNMENT_VALUE) == 0)
   {
   rox_log(" ptr is aligned ");
   }
   else
   {
   rox_log(" ptr is not aligned ");
   }
#endif
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_memory_reallocate_aligned)
{
   Rox_ErrorCode     error = ROX_ERROR_NONE;
   Rox_Ptr           pointer = NULL;
   Rox_Ptr           pointer_aligned = NULL;
   Rox_Ptr           pointer_realloc = NULL;
   Rox_Ptr           pointer_realloc_aligned = NULL;
   const Rox_Size    size_unit = 1;
   const Rox_Size    size_zero = 0;
   const Rox_Size    size_overflow = SIZE_MAX;
   // const Rox_Size    size_unaligned = 361;
   const Rox_Uchar   alignment_bytes_zero = 0;
   const Rox_Uchar   alignment_bytes_unit = 1;
   // const Rox_Uchar   alignment_bytes_over = UCHAR_MAX;

   //void *rox_memory_reallocate_aligned(void **aligned_adress, void *oldpointer, void *oldaligned, const Rox_Size oldsize, const Rox_Size element_size, const Rox_Size element_count, const Rox_Uchar alignment_bytes)
   
   //expect null on aligned null pointer address
   pointer = (Rox_Ptr)rox_memory_reallocate_aligned(NULL, NULL, NULL, size_unit, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);
   rox_memory_delete(pointer);
   pointer = NULL;
   
   //expect allocation
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, NULL, pointer, size_unit, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_NOT_EQUAL(pointer_realloc_aligned, (Rox_Ptr)NULL);
   rox_memory_delete(pointer_realloc);
   pointer = NULL;
   pointer_realloc = NULL;
   
   //expect null on aligned null pointer address
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, NULL, NULL, size_unit, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer, (Rox_Ptr)NULL);
   rox_memory_delete(pointer_realloc);
   pointer_realloc = NULL;
   
   //expect null on aligned null pointer address
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, pointer, NULL, size_unit, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   rox_memory_delete(pointer);
   pointer = NULL;
   pointer_realloc = NULL;
   
   //expect reallocation
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, pointer, pointer_aligned, size_unit, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   rox_memory_delete(pointer_realloc);
   pointer = NULL;

   //expect null on 0 alignment pointer
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, pointer, pointer_aligned, size_unit, size_unit, size_unit, alignment_bytes_zero);
   ROX_TEST_CHECK_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_EQUAL(pointer_realloc_aligned, (Rox_Ptr)NULL);
   rox_memory_delete(pointer);
   pointer = NULL;

   //expect an allocation
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, pointer, pointer_aligned, size_unit, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_NOT_EQUAL(pointer_realloc_aligned, (Rox_Ptr)NULL);
   rox_memory_delete(pointer_realloc);
   pointer = NULL;

   //expect null on zero size
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, pointer, pointer_aligned, size_unit, size_zero, size_zero, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_EQUAL(pointer_realloc_aligned, (Rox_Ptr)NULL);
   rox_memory_delete(pointer);
   pointer = NULL;

   //expect null on overflow
   pointer = (Rox_Ptr)rox_memory_allocate_aligned((void**)&pointer_aligned, size_unit, size_unit, alignment_bytes_unit);
   ROX_TEST_CHECK_NOT_EQUAL(pointer, (Rox_Ptr)NULL);
   pointer_realloc = (Rox_Ptr)rox_memory_reallocate_aligned((void**)&pointer_realloc_aligned, pointer, pointer_aligned, size_unit, size_overflow, size_overflow, alignment_bytes_unit);
   ROX_TEST_CHECK_EQUAL(pointer_realloc, (Rox_Ptr)NULL);
   ROX_TEST_CHECK_EQUAL(pointer_realloc_aligned, (Rox_Ptr)NULL);
   rox_memory_delete(pointer);
   pointer = NULL;
   
   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
