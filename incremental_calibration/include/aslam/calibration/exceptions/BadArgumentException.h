/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file BadArgumentException.h
    \brief This file defines the BadArgumentException class, which
           is thrown whenever the arguments of a function are invalid
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_BADARGUMENTEXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_BADARGUMENTEXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class BadArgumentException represents any
        exceptions occuring when the arguments passed to a function are invalid.
        \brief Bad argument exception
      */
    template <typename X> class BadArgumentException :
      public Exception {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs exception from argument and string
      BadArgumentException(const X& argument, const std::string& msg, const
        std::string& filename = " ", size_t line = 0, const std::string&
        function = " ");
      /// Destructor
      virtual ~BadArgumentException() {}
      /** @}
        */
    };

    template <typename X>
    BadArgumentException<X>::BadArgumentException(const X& argument,
        const std::string& msg, const std::string& filename, size_t line, const
        std::string& function) : Exception(msg, filename, line, function) {
      std::stringstream stream;
      stream << "[argument = " << argument << "]";
      mOutputMessage.append(stream.str());
    }

  }
}

#endif // ASLAM_CALIBRATION_EXCEPTIONS_BADARGUMENTEXCEPTION_H
