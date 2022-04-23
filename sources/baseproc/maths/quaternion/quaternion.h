//==============================================================================
//
//    OPENROX   : File quaternion.h
//
//    Summary   : Quaternion library definitions
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUATERNION__
#define __OPENROX_QUATERNION__

#include <baseproc/maths/linalg/matso3.h>

//! \ingroup  Linalg
//! \defgroup Quaternion Quaternion
//! \brief Define types, macros and functions for quaternion manipulation.
//! A quaternion is a 1 x 4 vector q = [cos( theta/2 ), sin( theta/2 )*u'];

//! \ingroup Quaternion
//! \brief   Quaternion object.
typedef struct _Rox_Array2D_Double * Rox_Quaternion;

//! \ingroup Quaternion
//! \brief Allocate memory and initialize a quaternion object.
//! \param  [out] q                 Pointer on newly allocated quaternion object.
//! \warning Must be freed with rox_quaternion_del
//! \return    An error code
ROX_API Rox_ErrorCode rox_quaternion_new( Rox_Quaternion * q );

//! \ingroup Quaternion
//! \brief Create and initialize a quaternion object.
//! \param  [in]  data              Pointer to real quaternion data
//! \param  [out] q                 Newly created quaternion structure.
//! \pre \a data points on allocated memory of \a size elements.
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_new_init( Rox_Quaternion * q, const Rox_Double data[4] );

//! \ingroup Quaternion
//! \brief Create and initialize a quaternion object.
//! \param  [in]  data              Pointer to float quaternion data
//! \param  [out] q                 Newly created quaternion structure.
//! \pre \a data points on allocated memory of \a size elements.
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_new_init_float( Rox_Quaternion * q, const Rox_Float data[4] );

//! \ingroup Quaternion
//! \brief Copy constructor
//! \param  [out] q_copy            Quaternion
//! \param  [in] 	q                 Quaternion instance to be copied
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_new_copy( Rox_Quaternion * q_copy, const Rox_Quaternion q );

//! \ingroup Quaternion
//! \brief Delete quaternion object.
//! \param  [in]  q                 Pointer on quaternion object.
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_del( Rox_Quaternion * q );

//! \ingroup Quaternion
//! \brief Get pointer to data
//! \param  [out] data              Output Data
//! \param  [in]  q                 Quaternion
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_get_data( Rox_Double data[4], const Rox_Quaternion q );

//! \ingroup Quaternion
//! \brief Set quaternion to unit
//! \param  [out] q                 Quaternion
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_set_unit( Rox_Quaternion q );

// Copy

//! \ingroup Quaternion
//! \brief Copy all data from a quaternion to another
//! \param [out]  q_out  Destination quaternion
//! \param [in]  q_inp   Source quaternion
//! \pre \a q_inp and \a q_out are valid quaternion.
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_copy( Rox_Quaternion q_out, const Rox_Quaternion q_inp );

//! \ingroup Quaternion
//! \brief Multiply two quaternion ( q_out = q_in1 o q_in2 )
//! \param[out] q_out Output quaternion
//! \param[in]  q_in1 Input quaternion
//! \param[in]  q_in2 Input quaternion
//! \pre \a v_in1 and v_in2 and \a v_out are valid quaternions.
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_mul( Rox_Quaternion q_out, const Rox_Quaternion q_in1, const Rox_Quaternion q_in2 );

//! \ingroup Quaternion
//! \brief Display a quaternion on screen
//! \param[in] q Quaternion
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_print( const Rox_Quaternion q );

//! \ingroup Quaternion
//! \brief Save a quaternion in a file
//! \param[in] filename File name
//! \param[in] q Quaternion
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_save( const char *filename, const Rox_Quaternion q );

// ====== INTERNAL FUNCTIONS =================================================

//! \ingroup Quaternion
//! \brief Display a quaternion in human-readable form.
//! \param[in] q quaternion
//! \param[out] stream Display output
//! \param[in] precision Numeric precision
//! \pre \a q is valid.
//! \pre \a stream is either \c NULL or valid.
//! \remarks Silent if \a stream is \c NULL
//!  Invoking <tt>rox_quaternion_print( q, stdout );</tt> displays:
//! \verbatim
//!    Quaternion ( 4 ):
//!      1.000000000 0.000000000 0.000000000 0.000000000
//! \endverbatim
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_print_stream( const Rox_Quaternion q, FILE* stream, Rox_Uint precision );

//! \ingroup Quaternion
//! \brief Multiply quaternion elements with a scalar
//! \param[in]  q_inp Input quaternion
//! \param[in]  s  Scalar
//! \param[out] q_out Output quaternion
//! \f$
//! \forall ( k ) \quad
//! \mbox{vo}_{( k )} = \mbox{s} \times \mbox{vi}_{( k )}
//! \f$
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_scm( const Rox_Quaternion q_out, const Rox_Quaternion q_inp, const Rox_Double s );

//! \internal
//! \ingroup Quaternion
//! \brief Quaternion sum-squares
//! \param[in] q Quaternion
//! \return Sum-squares
//! \pre \a q is valid.
//! \post Return a positive value.
//! \f$ \sum_k \mbox{q}_k^2 \f$
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_sumsquares(Rox_Double * sumsquares, const Rox_Quaternion q );

//! \internal
//! \ingroup Quaternion
//! \brief Quaternion norm
//! \param[in] q Quaternion
//! \return An error code
//! \pre \a q is valid.
//! \post Return a positive value.
//! \f$ \sqrt{\sum_k \mbox{q}_k^2} \f$
ROX_API Rox_ErrorCode rox_quaternion_norm(Rox_Double * norm, const Rox_Quaternion q );

//! \internal
//! \ingroup Quaternion
//! \brief Normalize quaternion, i.e. set norm = 1.0
//! \param[out] q Quaternion
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_normalize( Rox_Quaternion q );

//! \ingroup Quaternion
//! \brief Check quaternion validity.
//! \param[in] q Pointer on quaternion object.
//! \remarks Abort if quaternion is not valid.
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_check( const Rox_Quaternion q );

//! \ingroup Quaternion
//! \brief Check if index is in the range of a valid quaternion.
//! \param[in] q Quaternion object.
//! \param[in] i index.
//! \remarks Abort if i is not in the range [0, v->size-1].
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_check_range( const Rox_Quaternion q, const Rox_Uint i );

//! \ingroup Quaternion
//! \brief Transform a rotation matrix into a quaternion ( full check of rotation properties included ).
//! \param[out]   q  Quaternion vector
//! \param[in]    R  Rotation matrix in SO3
//! \return An error code
ROX_API Rox_ErrorCode rox_quaternion_from_matso3( Rox_Quaternion q, const Rox_MatSO3 R );

#endif // __OPENROX_QUATERNION__

