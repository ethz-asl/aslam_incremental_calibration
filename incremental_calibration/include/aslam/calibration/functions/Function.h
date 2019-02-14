/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Function.h
    \brief This file defines the class Fonction, which is an interface for any
           kind of functions
  */

#ifndef ASLAM_CALIBRATION_FUNCTIONS_FUNCTION_H
#define ASLAM_CALIBRATION_FUNCTIONS_FUNCTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    /** The Function class is an interface to any kind of mathematical functions
        \brief Function
      */
    template <typename Y, typename X> class Function {
    public:
      /** \name Types
        @{
        */
      /// Domain
      typedef X Domain;
      /// Codomain
      typedef Y Codomain;
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~Function();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the function value for the given argument
      virtual Y getValue(const X& argument) const = 0;
      /// Access the function value for the given argument
      virtual Y operator()(const X& argument) const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/functions/Function.tpp"

#endif // ASLAM_CALIBRATION_FUNCTIONS_FUNCTION_H
