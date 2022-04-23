//==============================================================================
//
//    OPENROX   : File polynomials.h
//
//    Contents  : API of polynomials module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POLYNOMIALS__
#define __OPENROX_POLYNOMIALS__

#include <system/memory/datatypes.h>
#include <generated/dynvec_point2d_double.h>

//! \ingroup Maths
//! \addtogroup polynomial
//! @{

//! A structure to represent a polynom of variable (not defined at compile time) degree
struct Rox_Polynom_Struct
{
   //! The polynom order
   Rox_Sint order;

   //! The polynom coefficients
   Rox_Double * coefficients;
};

//! Define the pointer of the Rox_Polynom_Struct
typedef struct Rox_Polynom_Struct * Rox_Polynom;

//! Create a new polynom object
//! \param  [out] obj               The created object pointer
//! \param  [in]  degree            The degree of the polynom
//! \return An error code
ROX_API Rox_ErrorCode rox_polynom_new(Rox_Polynom * obj, Rox_Uint degree);

//! Delete a polynom object
//! \param  [out] obj               The object pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_polynom_del(Rox_Polynom * obj);

//! Solve a polynomial of degree 2 : p(x) = c[0] * x^2 + c[1] * x^1 + c[2] * x^0 = 0
//! \param  [out] roots             The 2 computed roots
//! \param  [in]  coeff             The polynomial coefficients
//! \return An error code
ROX_API Rox_ErrorCode rox_quadratic_roots ( Rox_Complex roots, const Rox_Double coeff[3]);

//! Solve a polynomial of degree 3 : p(x) = c[0] * x^3 + c[1] * x^2 + c[2] * x^1 + c[4] * x^0 = 0
//! \param  [out] roots             The 3 computed roots
//! \param  [in]  coeff             The polynomial coefficients
//! \return An error code
ROX_API Rox_ErrorCode rox_cubic_roots ( Rox_Complex roots, const Rox_Double coeff[4]);

//! Solve a polynomial of degree 4 : p(x) = c[0] * x^4 + c[1] * x^3 + c[2] * x^2 + c[4] * x^1 + c[5] * x^0 = 0
//! \param  [out] roots             The 4 computed roots
//! \param  [in]  coeff             The polynomial coefficients
//! \return An error code
ROX_API Rox_ErrorCode rox_quartic_roots(Rox_Complex roots, const Rox_Double coeff[5]);

//! Solve a polynomial of degree n : p(x) = c[0] * x^0 + c[1] * x^1 + c[2] * x^2 + c[3] * x^3 + ... + c[n] * x^n = 0
//! \warning this function is very experimental !!!
//! \param  [out] roots             The result roots array (size n)
//! \param  [out] nbroots           The pointer to the result number of roots
//! \param  [in]  coeff             The n+1 array of coefficients
//! \param  [in]  degree            The degree n value
//! \return An error code
ROX_API Rox_ErrorCode rox_polynomial_roots_sturm(Rox_Double * roots, Rox_Uint * nbroots, Rox_Double * coeffs, Rox_Uint degree);

//! Evaluate a polynomial given its coefficients and a given variable value
//! \param  [out] res               The result value pointer
//! \param  [out] coeffs            The n+1 array of coefficients
//! \param  [in]  degree            The degree of the polynomial
//! \param  [in]  value             The value of the variable
//! \return An error code
ROX_API Rox_ErrorCode rox_polynomial_eval(Rox_Double * res, Rox_Double * coeffs, Rox_Uint degree, Rox_Double value);

//! Compute a polynomial of degree N which fits as well as possible the input
//! \param  [out] res               The result polynomial coefficients
//! \param  [in]  degree            The degree of the polynomial
//! \param  [in]  input             The list of points to fit
//! \return An error code
ROX_API Rox_ErrorCode rox_polynomial_fit(Rox_Double * res, Rox_Uint degree, Rox_DynVec_Point2D_Double input);

//! @}

#endif
