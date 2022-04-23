//==============================================================================
//
//    OPENROX   : File openrox_tests.hpp
//
//    Contents  : API of routines used for tests
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TESTS__
#define __OPENROX_TESTS__

#include "system/arch/compiler.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>
#include <cstring>
#include <cstdarg>

extern "C"
{
   #include <inout/system/errors_print.h>
}


#if ANDROID
   #include <android/log.h>
#endif

//Memory tests only suitable under windows for now
#ifdef _WIN32

   #define NOMINMAX
   #include "windows.h"
   #include "psapi.h"

#endif //_WIN32

#define ROX_TEST_CHECK_EQUAL(x,y) this->checkEqual(x, y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_NOT_EQUAL(x,y) this->checkNotEqual(x, y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_SUPERIOR(x,y) this->checkSuperior(x, y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(x,y) this->checkSuperiorOrEqual(x, y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_INFERIOR(x,y) this->checkInferior(x, y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_INFERIOR_OR_EQUAL(x,y) this->checkInferiorOrEqual(x, y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_ARRAY(x,y,z,w) this->checkArray(x,y,z,w, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_POSE(x,y,z,w) this->checkPose(x,y,z,w, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_CLOSE(x,y,z) this->checkClose(x,y,z, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK_SMALL(x,y) this->checkSmall(x,y, __FILE__, std::to_string(__LINE__))
#define ROX_TEST_CHECK(x) this->check(x, __FILE__, std::to_string(__LINE__))

//! Define log callback function pointer
typedef void (* rox_tests_log_callback)(const char* message);

//! Test framework class implementation
//! Subclass it to create a specific behavior in your project
class RoxTest
{
public:
   //! Feeded by ROX_TEST_CHECK_XXX results to return a proper value in the main function (ctest target)
   std::vector<int> _rox_test_suit_global_results;

   //! Test name
   std::string _name;

   //! Log callback
   static rox_tests_log_callback _log_callback;

public:
   //! How will be treated messages from test methods can be specific to each project
   //! @param message the message to send
   virtual void ROX_TEST_MESSAGE(std::string message, ...)
   {
      int final_n, n = ((int)message.size()) * 2; /* Reserve two times as much as the length of the message */
      std::unique_ptr<char[]> formatted;
      va_list ap;
      while (1) {
         formatted.reset(new char[n]); /* Wrap the plain char array into the unique_ptr */
         strcpy(&formatted[0], message.c_str());
         va_start(ap, message);
         final_n = vsnprintf(&formatted[0], n, message.c_str(), ap);
         va_end(ap);
         if (final_n < 0 || final_n >= n)
            n += abs(final_n - n + 1);
         else
            break;
      }

      if (_log_callback)
      {
         _log_callback(formatted.get());
      }
      else
      {
#ifdef ANDROID
      __android_log_print(ANDROID_LOG_INFO, "RoxTest", "%s", formatted.get());
#else
      std::cout << std::string(formatted.get()) << std::endl;
      std::ostringstream stream;
      stream << std::string(formatted.get()) << std::endl; std::ofstream output_stream;
      
      output_stream.open("test.log", std::ios_base::app);

      std::stringstream out;

      output_stream << stream.str();
#endif
      }
   }

public:
   //! Main test method
   virtual void rox_test_method() = 0;

   //! Displays a custom test message
   //! @param message the message to send
   //void ROX_TEST_MESSAGE(const std::string& message)
   //{
   //   std::string  rox_test_output = std::string("TEST : ") + std::string(message) + "...";
   //   ROX_TEST_MESSAGE(rox_test_output);
   //}

   //! Tests if a value is true or false (typically a pointer) and handles the result for this test case
   //! @param X a value to test
   template <typename T>
   void check(T X, std::string file, std::string line)
   {
      if (X)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }

   //! Tests if two values are equal and handles the result for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   template <typename T, typename U>
   void checkEqual(const T X, const U Y, const std::string file, const std::string line)
   {
      if (X == Y)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }

   //! Tests if two values are unequal and handles the results for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   template <typename T, typename U>
   void checkNotEqual(T X, U Y, std::string file, std::string line)
   {
      if (X != Y)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }

   //! Tests if a value is superior to another and handles the result for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   template <typename T, typename U>
   void checkSuperior(T X, U Y, std::string file, std::string line)
   {
      if (X > Y)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }
   //! Tests if a value is superior or equal to another and handles the result for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   template <typename T, typename U>
   void checkSuperiorOrEqual(T X, U Y, std::string file, std::string line)
   {
      if (X >= Y)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }
   //! Tests if a value is inferior to another and handles the result for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   template <typename T, typename U>
   void checkInferior(T X, U Y, std::string file, std::string line)
   {
      if (X < Y)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }
   //! Tests if a value is inferior or equal to another and handles the result for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   template <typename T, typename U>
   void checkInferiorOrEqual(T X, U Y, std::string file, std::string line)
   {
      if (X <= Y)
         setSuccess(file, line);
      else
         setFailure(file, line);
   }

   //! Tests if a an array is equal to another with an epsilon value of acceptance and handles the result for this test case
   //! @param A an array to compare
   //! @param R a reference array to compare
   //! @param Nb the number of elements to compare in the arrays
   //! @param E epsilon value accepted to test single elements
   template <typename T, typename U, typename V, typename W>
   bool checkArray(const T* A, const U* R, const V Nb, const W E, std::string file, std::string line)
   {
      for (V i = 0; i < Nb; ++i)
      {
         if (!checkClose(A[i], R[i], E, file, line))
            return false;
      }

      return true;
   }

   //! Tests if a pose is equal to another with an angle and a translation epsilon value of acceptance and handles the result for this test case
   //! @param P the pose to compare
   //! @param R the reference pose to compare
   //! @param angle_epsilon epsilon value accepted to test rotation
   //! @param translation_epsilon epsilon value accepted to test translation
   template <typename T, typename U, typename V, typename W>
   bool checkPose(const T* P, const U* R, const V angle_epsilon, const W translation_epsilon,  std::string file, std::string line)
   {
      for (int i = 0; i < 16; ++i)
      {
         if ((i + 1) % 4 != 0)
         {
            if (!checkClose(P[i], R[i], angle_epsilon, file, line))
               return false;
         }
         else
         {
            if (!checkClose(P[i], R[i], translation_epsilon, file, line))
               return false;
         }
      }
      return true;
   }

   //! Tests if a a value is close to another with an epsilon value of acceptance and handles the result for this test case
   //! @param X a value to compare
   //! @param Y a value to compare
   //! @param E epsilon value accepted to test single elements
   template <typename T, typename U, typename S>
   bool checkClose(const T V, const U R, const S E, std::string file, std::string line)
   {
      const double test_value = static_cast<const double>(V);
      const double test_reference = static_cast<const double>(R);
      const double test_epsilon = static_cast<const double>(E);

      if (fabs(test_value - test_reference) > test_epsilon)
      {
         setFailure(file, line);
         return false;
      }
      else
      {
         setSuccess(file, line);
         return true;
      }
   }

   //! Tests if a a value is close 0 with an epsilon value of acceptance and handles the result for this test case
   //! @param V a value to compare
   //! @param E epsilon value accepted to test single elements
   template <typename T, typename U>
   void checkSmall(T V, U E, std::string file, std::string line)
   {
      checkClose(V, 0, E, file, line);
   }

#ifdef _WIN32
   _CrtMemState _memstate1, _memstate2, _memstate3;
   void memoryTestSnapshot()
   {
      /* Encapsulate test method with a memory snapshot*/
      /*to check if there was a leak during this call*/
      _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
      _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDOUT);
      /* holds the memory states, take the memory snapshot*/
      _CrtMemCheckpoint(&_memstate1);
   }

   void memoryTestDiff()
   {
      /*take the memory snapshot*/
      _CrtMemCheckpoint(&_memstate2);
      /*compare two snapshots, any difference should caught by the debug heap */
      if (_CrtMemDifference(&_memstate3, &_memstate1, &_memstate2))
      {
         //ROX_TEST_CHECK(false);
         _CrtDumpMemoryLeaks();
      }
      else
      {
         //ROX_TEST_CHECK(true);
         ROX_TEST_MESSAGE("\n*** No memory leaks\n");
      }
   }
#else
   void memoryTestSnapshot() {};
   void memoryTestDiff() {};
#endif //_WIN32

private:
   //! Handles failure process
   inline void setFailure(std::string file, std::string line)
   {
      _rox_test_suit_global_results.push_back(-1);
      std::string  rox_test_output = "\n*** Failure on line: " + std::string(line);
      rox_test_output += " of file: " + std::string(file);
      ROX_TEST_MESSAGE(rox_test_output);
   }
   //! Handles success process
   inline void setSuccess(std::string file, std::string line)
   {
      _rox_test_suit_global_results.push_back(0);
      std::string  rox_test_output = "+++ Success on line: " + std::string(line);
      rox_test_output += " of file: " + std::string(file);
      //ROX_TEST_MESSAGE(rox_test_output);
   }

};

template<>  // explicit specialization for T = const char*
inline void RoxTest::checkEqual<const char*, const char*>(const char* X, const char* Y, const std::string file, const std::string line)
{
   const std::string x(X);
   const std::string y(Y);
   if (x.compare(y) == 0)
      setSuccess(file, line);
   else
      setFailure(file, line);
}
template<>  // explicit specialization for T = string
inline void RoxTest::checkEqual<const std::string, const std::string>(const std::string X, const std::string Y, const std::string file, const std::string line)
{
   if (X.compare(Y) == 0)
      setSuccess(file, line);
   else
      setFailure(file, line);
}


//! Begin each test suit with this macro
//! @param X the name of this tests suit
//! @remark mandatory
//TODO use of last instance created for _rox_test_base_instance pointer is tricky and ugly
# define ROX_TEST_SUITE_BEGIN(X)          \
std::vector<RoxTest*> _rox_test_base_instance;   \
std::string _rox_test_suit_name(#X);

//! Begin each test case with this macro
//! @param I the class to use as base class for this case, typically a subclass of RoxTest
//! @param X the name of this test case
//! @remark mandatory
# define ROX_TEST_CASE_DECLARE(I,X)                                                 \
class rox_test_case_##X : public I                                                  \
{                                                                                   \
public:                                                                             \
   void rox_test_method();                                                          \
   rox_test_case_##X()                                                              \
   {                                                                                \
      _name = #X;                                                                   \
      /* Avoid to see the results push_backs as a leak*/                            \
      /* TODO might be a better way, more flexible than hardcode number... */       \
      _rox_test_suit_global_results.reserve(1000000);                               \
      _rox_test_base_instance.push_back(this);                                      \
   }                                                                                \
};                                                                                  \
rox_test_case_##X rox_test_case_instance_##X;                                       \
void rox_test_case_##X::rox_test_method()



//! End each test suit with this macro
//! @remark mandatory
#define ROX_TEST_SUITE_END()                                                                             \
ROX_EXPORT int main()                                                                                    \
{                                                                                                        \
   std::vector<RoxTest*>::const_iterator it_instances = _rox_test_base_instance.begin();           \
   for (; it_instances != _rox_test_base_instance.end(); ++it_instances)                                 \
   {                                                                                                     \
      RoxTest* test = (*it_instances);                                                             \
      std::string  rox_test_output  =                                                                    \
      std::string("\n/*====================================================*/")                          \
      +std::string("\n/*                     ")                                                          \
      + test->_name                                                                                      \
      +std::string("                    ")                                                               \
      +std::string("\n/*====================================================*/\n");                      \
      test->ROX_TEST_MESSAGE(rox_test_output);                                                    \
      test->memoryTestSnapshot();                                                                        \
      /*Call the user implemented test*/                                                                 \
      test->rox_test_method();                                                                           \
      test->memoryTestDiff();                                                                            \
      test->ROX_TEST_MESSAGE(rox_test_output);                                                    \
      std::vector<int>::const_iterator it = test->_rox_test_suit_global_results.begin();                 \
      for (; it != test->_rox_test_suit_global_results.end(); ++it)                                      \
      {                                                                                                  \
         if (*it != 0)                                                                                   \
         {                                                                                               \
            std::string  rox_test_output = "\n--- Failure on test suite: " + _rox_test_suit_name + "\n"; \
            test->ROX_TEST_MESSAGE(rox_test_output);                                              \
            return -1;                                                                                   \
         }                                                                                               \
      }                                                                                                  \
   }                                                                                                     \
   std::string  rox_test_output = "\n+++ Success on test suite: " + _rox_test_suit_name + "\n";          \
   (*_rox_test_base_instance.begin())->ROX_TEST_MESSAGE(rox_test_output);                         \
   RoxTest::_log_callback = NULL;                                                                  \
   return 0;                                                                                             \
}

namespace rox
{
   //! rox_sdk tests class implementation
   class OpenROXTest : public RoxTest
   {
   public:
      //! How will be treated messages from test methods is specific to each project
      //! @param message the message to send
      /*
      virtual void ROX_TEST_MESSAGE(const std::string& message)override
      {
         ROX_INFO_DISPLAY(message.c_str());
         ROX_INFO_LOGFILE(message.c_str());
      }*/

   public:

      //TODO ?
      /*
      //! Tests if a matrix is equal to another with an epsilon value of acceptance and handles the result for this test case
      //! @param M a matrix to compare
      //! @param R a reference matrix to compare
      //! @param E epsilon value accepted to test single elements
      template <typename T, unsigned int padding = 0>
      void ROX_TEST_CHECK_MATRIX(const Array<T, padding>& M, const Array<T, padding>& R, double E)
      {
      for (unsigned int j = 0; j < M.getHeight(); ++j)
      {
      ROX_TEST_CHECK_ARRAY(M.getConstRowsData()[j], R.getConstRowsData()[j], M.getWidth(), E);
      }
      }

      //! Tests if an image is equal to another with an epsilon value of acceptance and handles the result for this test case
      //! @param I an image to compare
      //! @param R a reference image to compare
      //! @param E epsilon value accepted to test single elements
      template <typename T>
      void ROX_TEST_CHECK_IMAGE(const Image<T>& I, const Image<T>& R, double E)
      {
      ROX_TEST_CHECK_MATRIX(I, R, E);
      }

      //! Tests if a plane is equal to another with an epsilon value of acceptance and handles the result for this test case
      //! @param P a plane to compare
      //! @param R a reference plane to compare
      //! @param E epsilon value accepted to test single elements
      void ROX_TEST_CHECK_PLANE(const Plane3D& P, const Plane3D& R, double E)
      {
      ROX_TEST_CHECK_CLOSE(P.a, R.a, E);
      ROX_TEST_CHECK_CLOSE(P.b, R.b, E);
      ROX_TEST_CHECK_CLOSE(P.c, R.c, E);
      ROX_TEST_CHECK_CLOSE(P.d, R.d, E);
      }

      //! Tests if a line is equal to another with an epsilon value of acceptance and handles the result for this test case
      //! @param P a line to compare
      //! @param R a reference line to compare
      //! @param E epsilon value accepted to test single elements
      void ROX_TEST_CHECK_LINE(const ImplicitLine3D& L, const ImplicitLine3D& R, double E)
      {
      ROX_TEST_CHECK_PLANE(L.planes[0], R.planes[0], E);
      ROX_TEST_CHECK_PLANE(L.planes[1], R.planes[1], E);
      }

      //! Tests if a 3d point is equal to another with an epsilon value of acceptance and handles the result for this test case
      //! @param P a 3d point to compare
      //! @param R a reference 3d point to compare
      //! @param E epsilon value accepted to test single elements
      template <typename T>
      void ROX_TEST_CHECK_POINT3D(const Point3D<T>& P, const Point3D<T>& R, double E)
      {
      ROX_TEST_CHECK_CLOSE(P.X, R.X, E);
      ROX_TEST_CHECK_CLOSE(P.Y, R.Y, E);
      ROX_TEST_CHECK_CLOSE(P.Z, R.Z, E);
      }
      //! Tests if a 2d point is equal to another with an epsilon value of acceptance and handles the result for this test case
      //! @param P a 2d point to compare
      //! @param R a reference 2d point to compare
      //! @param E epsilon value accepted to test single elements
      template <typename T>
      void ROX_TEST_CHECK_POINT2D(const Point2D<T>& P, const Point2D<T>& R, double E)
      {
      ROX_TEST_CHECK_CLOSE(P.u, R.u, E);
      ROX_TEST_CHECK_CLOSE(P.v, R.v, E);
      }

      */
   };

}


#endif // __OPENROX_TESTS__

