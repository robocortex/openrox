//==============================================================================
//
//    OPENROX   : File polynomials.c
//
//    Contents  : Implementation of polynomials module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "polynomials.h"

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <system/memory/memory.h>
#include <system/errors/errors.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/decomposition/qr.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_polynom_new(Rox_Polynom *obj, Rox_Uint degree)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Polynom ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Polynom) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->coefficients = (Rox_Double *) rox_memory_allocate(sizeof(Rox_Double), degree + 1);
   if (!ret->coefficients) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = ret;

function_terminate:
   if (error) rox_memory_delete(ret);
   return error;
}

Rox_ErrorCode rox_polynom_del(Rox_Polynom *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Polynom todel = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   rox_memory_delete(todel->coefficients);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadratic_roots(Rox_Complex comp, const Rox_Double coeff[3])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double a = coeff[0];
   Rox_Double b = coeff[1];
   Rox_Double c = coeff[2];
   Rox_Double norm = sqrt(a * a + b * b + c * c);

   if (ROX_IS_ZERO_DOUBLE(norm)) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   a /= norm;
   b /= norm;
   c /= norm;

   comp[0].real = 0.0;
   comp[0].imag = 0.0;
   comp[1].real = 0.0;
   comp[1].imag = 0.0;

   if (ROX_IS_ZERO_DOUBLE(a))
   {
      if (ROX_IS_ZERO_DOUBLE(b))
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; 
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         comp[0].real = -c / b;
         goto function_terminate;
      }
   }
   else
   {
      Rox_Double d = b * b - 4 * a * c;
      Rox_Double a2 = 2 * a;

      comp[0].real = -b;
      comp[1].real = -b;

      if (d < 0.0)
      {
         comp[0].imag = sqrt(-d);
         comp[1].imag = -comp[0].imag;
      }
      else
      {
         Rox_Double tmp = sqrt(d);

         comp[0].real += tmp;
         comp[0].imag = 0.0;

         comp[1].real -= tmp;
         comp[1].imag = 0.0;
      }

      comp[0].real /= a2;
      comp[0].imag /= a2;
      comp[1].real /= a2;
      comp[1].imag /= a2;

      goto function_terminate;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_cubic_roots(Rox_Complex comp, const Rox_Double coeff[4])
{
   Rox_Double theta, fact;

   if (ROX_IS_ZERO_DOUBLE(coeff[0]))
   {
      // It is a quadratic polynomial
      return rox_quadratic_roots(&comp[0], &coeff[1]);
   }
   else
   {
      // Solve equation x^3 + b * x^2 + c * x + d = 0
      Rox_Double b = coeff[1] / coeff[0];
      Rox_Double c = coeff[2] / coeff[0];
      Rox_Double d = coeff[3] / coeff[0];

      Rox_Double b_3 = b / 3.0;

      Rox_Double p = c - b * b_3;
      Rox_Double q = 2.0 * b_3 * b_3 * b_3  - c * b_3 + d;

      // Discriminant
      Rox_Double dis = q * q / 4.0 + p * p * p / 27.0;

      if (dis < 0.0)
      {
         // 3 real solutions

         Rox_Double th = -q / 2.0 * sqrt(-27.0 / (p * p * p));

         if (th > 1.0) th = 1.0;
         if (th < -1.0) th = -1.0;

         theta = 1.0 / 3.0 * acos(th);
         fact = sqrt(-4.0 / 3.0 * p);

         comp[0].real =  fact * cos(theta);
         comp[1].real = -fact * cos(theta + ROX_PI / 3.0);
         comp[2].real = -fact * cos(theta - ROX_PI / 3.0);

         comp[0].imag = 0.0;
         comp[1].imag = 0.0;
         comp[2].imag = 0.0;
      }
      else if (dis > 0.0)
      {
         // 1 real and 2 complex conjugate solutions
         Rox_Double u, v;
         Rox_Double sign;

         Rox_Double factor = -q / 2.0 + sqrt(dis);

         if (factor > 0.0) sign = 1.0;
         else sign = -1.0;

         u = sign * pow(sign * factor, 1.0 / 3.0);

         factor = -q / 2.0 - sqrt(dis);
         if (factor > 0.0) sign = 1.0;
         else sign = -1.0;

         v = sign * pow(sign * factor, 1.0 / 3.0);

         comp[0].real = u + v;
         comp[1].real = - (u + v) / 2.0;
         comp[2].real = comp[1].real;

         comp[0].imag = 0.0;
         comp[1].imag = (u - v) / 2.0 * sqrt(3.0);
         comp[2].imag = -comp[1].imag;
      }
      else // dis == Zero
      {
         // 3 real solutions of which at least 2 are equal

         // if D == 0 and p == 0 => q == 0
         if (ROX_IS_ZERO_DOUBLE(p))
         {
            comp[0].real = 0.0;
            comp[1].real = 0.0;
            comp[2].real = 0.0;
            comp[0].imag = 0.0;
            comp[1].imag = 0.0;
            comp[2].imag = 0.0;
         }
         else
         {
            comp[0].real = 3.0 * q / p;
            comp[1].real = -comp[0].real / 2.0;
            comp[2].real = comp[1].real;
            comp[0].imag = 0.0;
            comp[1].imag = 0.0;
            comp[2].imag = 0.0;
         }
      }

      // back substitution
      comp[0].real -= b_3;
      comp[1].real -= b_3;
      comp[2].real -= b_3;

      return ROX_ERROR_NONE;
   }
}

Rox_ErrorCode rox_quartic_roots(Rox_Complex comp, const Rox_Double coeff[5])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double h, j;
   Rox_Double p_alpha, p_beta, p_gamma;
   Rox_Double coeff_cubic[4];
   Rox_Double coeff_quadratic[3];

   if (ROX_IS_ZERO_PREC(coeff[0], 1e-32))
   {
      //It is a cubic polynomial
      return rox_cubic_roots(&comp[0], &coeff[1]);
   }

   // Solve  x^4 + a*x^3 * b*x^2 + c*x + d = 0
   // Solve quartic equation using Descartes-Euler-Cardano algorithm

   // Adjust coefficients
   Rox_Double a = coeff[1] / coeff[0];
   Rox_Double b = coeff[2] / coeff[0];
   Rox_Double c = coeff[3] / coeff[0];
   Rox_Double d = coeff[4] / coeff[0];

   // Reduce to solve a cubic equation
   p_alpha = b - (3.0 / 8.0) * a * a ; // e2
   p_beta  = c + a * (a * a / 8.0 - b / 2.0); // e1
   p_gamma = d - (3.0 / 256.0) * a * a * a * a + a * a * b / 16.0 - a * c / 4.0 ; // e0
   comp[0].real = 0.0;  comp[1].real = 0.0; comp[2].real = 0.0;  comp[3].real = 0.0;
   comp[0].imag = 0.0;  comp[1].imag = 0.0; comp[2].imag = 0.0;  comp[3].imag = 0.0;

   if (ROX_IS_ZERO_DOUBLE(p_gamma))
   {
      coeff_cubic[0] = 1.0;
      coeff_cubic[1] = 0.0;
      coeff_cubic[2] = p_alpha;
      coeff_cubic[3] = p_beta;
      rox_cubic_roots(&comp[0], coeff_cubic);
   }
   else if (ROX_IS_ZERO_DOUBLE(p_beta))
   {
      Rox_Double coeff2[3];
      Rox_Complex_Struct comp2[2];
      Rox_Double length;
      Rox_Double eps;

      coeff2[0] = 1.0;
      coeff2[1] = p_alpha;
      coeff2[2] = p_gamma;

      error = rox_quadratic_roots(comp2, coeff2);
      if (error) return error;

      length = sqrt(comp2[0].real * comp2[0].real + comp2[0].imag * comp2[0].imag);
      if (comp2[0].imag >= 0.0) eps = 1.0;
      else eps = -1.0;

      comp[0].real = eps / sqrt(2.0) * sqrt(length + comp2[0].real);
      comp[1].real = -comp[0].real;

      comp[0].imag = 1.0 / sqrt(2.0) * sqrt(length - comp2[0].real);
      comp[1].imag = -comp[0].imag;

      length = sqrt(comp2[1].real * comp2[1].real + comp2[1].imag * comp2[1].imag);
      if (comp2[1].imag >= 0.0) eps = 1.0;
      else eps = -1.0;

      comp[2].real = eps / sqrt(2.0) * sqrt(length + comp2[1].real);
      comp[3].real = -comp[2].real;

      comp[2].imag = 1.0 / sqrt(2.0) * sqrt(length - comp2[1].real);
      comp[3].imag = -comp[2].imag;
   }
   else
   {
      Rox_Double root = 0.0;

      coeff_cubic[0] = 1.0;
      coeff_cubic[1] = 2.0 * p_alpha;
      coeff_cubic[2] = p_alpha * p_alpha - 4.0 * p_gamma;
      coeff_cubic[3] = -p_beta * p_beta;

      if (rox_cubic_roots(&comp[0], coeff_cubic) != ROX_ERROR_NONE)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      for ( Rox_Sint i = 0; i < 3; i++)
      {
         if (ROX_IS_ZERO_DOUBLE(comp[i].imag))
         {
            root = fabs(comp[i].real);
            break;
         }
      }

      h = sqrt(root);
      j = (p_alpha + root - p_beta / h) / 2.0;

      coeff_quadratic[0] = 1.0;
      coeff_quadratic[1] = h;
      coeff_quadratic[2] = j;
      rox_quadratic_roots(&comp[0], coeff_quadratic);

      coeff_quadratic[1] = -h;
      coeff_quadratic[2] = p_gamma / j;
      rox_quadratic_roots(&comp[2], coeff_quadratic);
   }

   // back substitute
   for ( Rox_Sint i = 0; i < 4; i++)
   {
      comp[i].real -= a / 4.0;
   }

function_terminate:
   return error;
}

Rox_Uint rox_polynom_division_remainder(Rox_Polynom u, Rox_Polynom v, Rox_Polynom r)
{
   int k, j;
   double *nr, *end, *uc;

   nr = r->coefficients;
   end = &u->coefficients[u->order];

   uc = u->coefficients;
   while (uc <= end) *nr++ = *uc++;

   if (v->coefficients[v->order] < 0.0)
   {
      for (k = u->order - v->order - 1; k >= 0; k -= 2)
      {
         r->coefficients[k] = -r->coefficients[k];
      }

      for (k = u->order - v->order; k >= 0; k--)
      {
         for (j = v->order + k - 1; j >= k; j--)
         {
            r->coefficients[j] = -r->coefficients[j] - r->coefficients[v->order + k] * v->coefficients[j - k];
         }
      }
   }
   else
   {
      for (k = u->order - v->order; k >= 0; k--)
      {
         for (j = v->order + k - 1; j >= k; j--)
         {
            r->coefficients[j] -= r->coefficients[v->order + k] * v->coefficients[j - k];
         }
      }
   }

   k = v->order - 1;
   while (k >= 0 && fabs(r->coefficients[k]) < 1e-12)
   {
      r->coefficients[k] = 0.0;
      k--;
   }

   r->order = (k < 0) ? 0 : k;

   return (r->order);
}


Rox_ErrorCode rox_polynom_build_sturm_sequence(Rox_Uint *count_poly, Rox_Polynom *sturmseq, Rox_Uint degree)
{
   Rox_Uint      i;
   double   f, *fp, *fc;

   if (!sturmseq) return ROX_ERROR_NULL_POINTER;
   if (degree == 0) return ROX_ERROR_INVALID_VALUE;

   //Assign basic orders
   sturmseq[0]->order = degree;
   sturmseq[1]->order = degree - 1;

   //calculate the derivative and normalise the leading coefficient.
   f = fabs(sturmseq[0]->coefficients[degree] * degree);
   fp = sturmseq[1]->coefficients;
   fc = sturmseq[0]->coefficients + 1;
   for (i = 1; i <= degree; i++)
   {
      *fp++ = *fc++ * i / f;
   }

   // construct the rest of the Sturm sequence

   for (i = 2; i <= degree; i++)
   {
      if (!rox_polynom_division_remainder(sturmseq[i - 2], sturmseq[i - 1], sturmseq[i])) break;

      // reverse the sign and normalise

      f = -fabs(sturmseq[i]->coefficients[sturmseq[i]->order]);
      for (fp = &sturmseq[i]->coefficients[sturmseq[i]->order]; fp >= sturmseq[i]->coefficients; fp--)
      {
         *fp /= f;
      }
   }

   sturmseq[i]->coefficients[0] = - sturmseq[i]->coefficients[0];   // reverse the sign

   *count_poly = i;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_polynom_sturm_numroots(Rox_Uint *nbroots, Rox_Uint np, Rox_Polynom *sturmseq, Rox_Sint *atneg, Rox_Sint *atpos)
{
   int atposinf, atneginf;
   double   f, lf;
   Rox_Uint i;

   atposinf = 0;
   atneginf = 0;

   //changes at positive infinity
   lf = sturmseq[0]->coefficients[sturmseq[0]->order];

   for (i = 1; i <= np; i++)
   {
      f = sturmseq[i]->coefficients[sturmseq[i]->order];
      if (lf == 0.0 || lf * f < 0) atposinf++;
      lf = f;
   }

   // changes at negative infinity

   if (sturmseq[0]->order & 1)
   {
      lf = -sturmseq[0]->coefficients[sturmseq[0]->order];
   }
   else
   {
      lf = sturmseq[0]->coefficients[sturmseq[0]->order];
   }

   for (i = 1; i <= np; i++)
   {
      if (sturmseq[i]->order & 1)
      {
         f = -sturmseq[i]->coefficients[sturmseq[i]->order];
      }
      else
      {
         f = sturmseq[i]->coefficients[sturmseq[i]->order];
      }

      if (lf == 0.0 || lf * f < 0) atneginf++;
      lf = f;
   }

   *atneg = atneginf;
   *atpos = atposinf;
   *nbroots = (atneginf - atposinf);

   return ROX_ERROR_NONE;
}


Rox_ErrorCode rox_polynomial_sturm_signchange(Rox_Uint *nbchanges, Rox_Uint np, Rox_Polynom *sturmseq, Rox_Double a)
{
   Rox_Uint changes, i;
   double f = 0, lf = 0;

   changes = 0;

   rox_polynomial_eval(&lf, sturmseq[0]->coefficients, sturmseq[0]->order, a);

   for (i = 1; i <= np; i++)
   {
      rox_polynomial_eval(&f, sturmseq[i]->coefficients, sturmseq[i]->order, a);
      if (lf == 0.0 || lf * f < 0) changes++;
      lf = f;
   }

   *nbchanges = changes;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_polynom_sturm_modrf(Rox_Double *root, Rox_Double *coefs, Rox_Uint order, Rox_Double a, Rox_Double b)
{
   int      its;
   double   fa, fb, x, fx, lfx;
   double   *fp, *scoef, *ecoef;

   scoef = coefs;
   ecoef = &coefs[order];

   fb = fa = *ecoef;
   for (fp = ecoef - 1; fp >= scoef; fp--)
   {
      fa = a * fa + *fp;
      fb = b * fb + *fp;
   }

   // if there is no sign difference the method won't work

   if (fa * fb > 0.0) return ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;

   if (fabs(fa) < 1e-14)
   {
      *root = a;
      return ROX_ERROR_NONE;
   }

   if (fabs(fb) < 1e-14)
   {
      *root = b;
      return ROX_ERROR_NONE;
   }

   lfx = fa;

   for (its = 0; its < 64; its++)
   {
      x = (fb * a - fa * b) / (fb - fa);

      fx = *ecoef;
      for (fp = ecoef - 1; fp >= scoef; fp--)
      {
         fx = x * fx + *fp;
      }

      if (fabs(x) > 1e-14)
      {
         if (fabs(fx / x) < 1e-14)
         {
            *root = x;
            return ROX_ERROR_NONE;
         }
      }
      else if (fabs(fx) < 1e-14)
      {
         *root = x;
         return ROX_ERROR_NONE;
      }

      if ((fa * fx) < 0)
      {
         b = x;
         fb = fx;
         if ((lfx * fx) > 0) fa /= 2;
      }
      else
      {
         a = x;
         fa = fx;
         if ((lfx * fx) > 0) fb /= 2;
      }

      lfx = fx;
   }

   return ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
}

Rox_ErrorCode rox_polynomial_sturm_bisection(Rox_Double *roots, Rox_Uint countpoly, Rox_Polynom *sturmseq, Rox_Double min, Rox_Double max, Rox_Sint atmin, Rox_Sint atmax)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbroots;
   Rox_Uint its;
   Rox_Double mid;
   Rox_Sint   n1 = 0, n2 = 0;
   Rox_Uint atmid;

   nbroots = atmin - atmax;

   //Only one root
   if (nbroots == 1)
   {
      //Try a simple method first, if succeed exit
      error = rox_polynom_sturm_modrf(&roots[0], sturmseq[0]->coefficients, sturmseq[0]->order, min, max);
      if (!error) { error = ROX_ERROR_NONE; goto function_terminate; }

      //Harder method
      for (its = 0; its < 64; its++)
      {
         mid = (min + max) / 2;

         rox_polynomial_sturm_signchange(&atmid, countpoly, sturmseq, mid);

         if (fabs(mid) > 1.0e-14)
         {
            if (fabs((max - min) / mid) < 1.0e-14)
            {
               roots[0] = mid;
               error = ROX_ERROR_NONE; goto function_terminate;

            }
         }
         else if (fabs(max - min) < 1.0e-14)
         {
            roots[0] = mid;
            error = ROX_ERROR_NONE; goto function_terminate;
         }

         if ((atmin - atmid) == 0) min = mid;
         else max = mid;
      }

      if (its == 800)
      {
         roots[0] = mid;
         error = ROX_ERROR_ALGORITHM_FAILURE; goto function_terminate;
      }

      error = ROX_ERROR_NONE; goto function_terminate;
   }

   // more than one root in the interval, we have to bisect...

   for (its = 0; its < 64; its++)
   {
      mid = (min + max) / 2;
      rox_polynomial_sturm_signchange(&atmid, countpoly, sturmseq, mid);

      n1 = atmin - atmid;
      n2 = atmid - atmax;


      if (n1 != 0 && n2 != 0)
      {
         error = rox_polynomial_sturm_bisection(roots, countpoly, sturmseq, min, mid, atmin, atmid);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_polynomial_sturm_bisection(&roots[n1], countpoly, sturmseq, mid, max, atmid, atmax); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }

      if (n1 == 0) min = mid;
      else max = mid;
   }

   if (its == 800)
   {
      error = ROX_ERROR_ALGORITHM_FAILURE; goto function_terminate;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_polynomial_roots_sturm(Rox_Double * roots, Rox_Uint * nbroots, Rox_Double * coeff, Rox_Uint degree)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Polynom * sturmseq = NULL;
   Rox_Uint idseq, idroot;
   Rox_Sint idcoef, i;
   Rox_Uint countroot;
   Rox_Uint countpoly, countchange;
   Rox_Double norm, abs0, factor, scale;
   Rox_Sint atmin, atmax;
   Rox_Double min, max;

   if (!roots || !nbroots || !coeff) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   countroot = 0;

   //Allocate sturm sequence
   error = ROX_ERROR_NONE;
   sturmseq = (Rox_Polynom *) rox_memory_allocate(sizeof(Rox_Polynom), degree + 1);
   if (!sturmseq)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (idseq = 0; idseq < degree + 1; idseq++)
   {
      error |= rox_polynom_new(&sturmseq[idseq], degree);
   }
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Check that the polynom is defined to the biggest order
   if (fabs(coeff[degree]) < DBL_EPSILON)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //format and normalize polynom so that the biggest order is 1
   norm = 1.0 / coeff[degree];
   sturmseq[0]->order = degree;
   for (idcoef = 0; idcoef <= (Rox_Sint) degree; idcoef++)
   {
      sturmseq[0]->coefficients[idcoef] = coeff[idcoef] * norm;
   }

   //Reduce coefficients range
   factor = 1.0;
   abs0 = fabs(coeff[0]);
   if (abs0 > degree)
   {
      factor = pow(abs0, -1.0 / degree);
      scale = factor;

      for (idcoef = degree - 1; idcoef >= 0; idcoef--)
      {
         sturmseq[0]->coefficients[idcoef] *= scale;
         scale *= factor;
      }
   }

   //Build the sturm sequence from the initial polynom
   error = rox_polynom_build_sturm_sequence(&countpoly, sturmseq, degree);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Compute root count
   error = rox_polynom_sturm_numroots(&countroot, countpoly, sturmseq, &atmin, &atmax);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Compute the brackets
   min = -1;
   error = rox_polynomial_sturm_signchange(&countchange, countpoly, sturmseq, min); ROX_ERROR_CHECK_TERMINATE(error)
   for (i = 0; countchange != atmin && i != 32; i++)
   {
      min *= 10.0;
      error = rox_polynomial_sturm_signchange(&countchange, countpoly, sturmseq, min); ROX_ERROR_CHECK_TERMINATE(error)
   }
   if (countchange != atmin)
   {
      atmin = countchange;
   }
   max = 1;
   error = rox_polynomial_sturm_signchange(&countchange, countpoly, sturmseq, max); ROX_ERROR_CHECK_TERMINATE(error)
   for (i = 0; countchange != atmax && i != 32; i++)
   {
      max *= 10.0;
      error = rox_polynomial_sturm_signchange(&countchange, countpoly, sturmseq, max); ROX_ERROR_CHECK_TERMINATE(error)
   }
   if (countchange != atmax)
   {
      atmax = countchange;
   }
   countroot = atmin - atmax;

   //Bisection processing to refine brackets
   error = rox_polynomial_sturm_bisection(roots, countpoly, sturmseq, min, max, atmin, atmax);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //No valid real root
   if (countroot == 0)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Cancel the coefficients range reduction
   for (idroot = 0; idroot < countroot; idroot++)
   {
      roots[idroot] /= factor;
   }
   *nbroots = countroot;
   error = ROX_ERROR_NONE;

function_terminate:
   // Delete sturm sequence
   for (idseq = 0; idseq < degree + 1; idseq++)
   {
      rox_polynom_del(&sturmseq[idseq]);
   }
   rox_memory_delete(sturmseq);

   return error;
}

Rox_ErrorCode rox_polynomial_eval(Rox_Double *res, Rox_Double *coeffs, Rox_Uint degree, Rox_Double value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double result;
   Rox_Sint iddeg;

   if (!res || !coeffs)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Starting with the biggest exponent
   result = coeffs[degree];

   // Recursive evaluation
   for (iddeg = degree - 1; iddeg >= 0; iddeg--)
   {
      result = value * result + coeffs[iddeg];
   }

   *res = result;

function_terminate:
   return error;
}

Rox_ErrorCode rox_polynomial_fit(Rox_Double *res, Rox_Uint degree, Rox_DynVec_Point2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double vandermonde = NULL, vecd = NULL, vece = NULL, vecx = NULL, Q = NULL, R = NULL, P = NULL, iR = NULL;

   Rox_Double **dv = NULL;
   Rox_Double *dd = NULL, *dx = NULL;

   if (!res || !input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (degree == 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&vandermonde, input->used, degree + 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&vecd, input->used, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&vece, input->used, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&vecx, degree + 1, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Q, input->used, input->used); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&R, input->used, degree + 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&P, degree + 1, degree + 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&iR, degree + 1, input->used); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dv, vandermonde);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &dd, vecd);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_get_data_pointer ( &dx, vecx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint idpt = 0; idpt < input->used; idpt++)
   {
      dv[idpt][0] = 1;
      for (Rox_Uint deg = 0; deg < degree; deg++)
      {
         dv[idpt][deg + 1] = dv[idpt][deg] * input->data[idpt].u;
      }

      dd[idpt] = input->data[idpt].v;
   }

   error = rox_array2d_double_qr(Q, R, vandermonde); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(vece, Q, vecd); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse(iR, R); 
   ROX_ERROR_CHECK_TERMINATE ( error ); // To replace with triangular inversion ...

   error = rox_array2d_double_mulmatmat(vecx, iR, vece); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint deg = 0; deg < degree + 1; deg++)
   {
      res[deg] = dx[deg];
   }

function_terminate:
   ROX_ERROR_CHECK(rox_array2d_double_del(&vandermonde));
   ROX_ERROR_CHECK(rox_array2d_double_del(&vecd));
   ROX_ERROR_CHECK(rox_array2d_double_del(&vece));
   ROX_ERROR_CHECK(rox_array2d_double_del(&vecx));
   ROX_ERROR_CHECK(rox_array2d_double_del(&Q));
   ROX_ERROR_CHECK(rox_array2d_double_del(&R));
   ROX_ERROR_CHECK(rox_array2d_double_del(&P));
   ROX_ERROR_CHECK(rox_array2d_double_del(&iR));

   return error;
}
