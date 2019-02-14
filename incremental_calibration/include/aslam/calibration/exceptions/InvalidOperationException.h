/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file InvalidOperationException.h
    \brief This file defines the InvalidOperationException class, which
           represents invalid operations exceptions.
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_INVALIDOPERATIONEXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_INVALIDOPERATIONEXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class InvalidOperationException represents invalid operations
        exceptions.
        \brief Invalid operation exception
      */
    class InvalidOperationException :
      public Exception {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Constructs exception from message
      InvalidOperationException(const std::string& msg = "", const std::string&
        filename = " ", size_t line = 0, const std::string& function = " ");
      /// Copy constructor
      InvalidOperationException(const InvalidOperationException& other)
        throw ();
      /// Assignment operator
      InvalidOperationException& operator = (const InvalidOperationException&
        other) throw();
      /// Destructor
      virtual ~InvalidOperationException() throw ();
      /** @}
        */

    protected:

    };

  }
}

#endif // ASLAM_CALIBRATION_EXCEPTIONS_INVALIDOPERATIONEXCEPTION_H
