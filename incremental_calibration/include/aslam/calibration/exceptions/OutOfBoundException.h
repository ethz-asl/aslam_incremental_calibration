/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file OutOfBoundException.h
    \brief This file defines the OutOfBoundException class, which represents any
           exceptions occuring when trying to access unallocated memory.
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_OUTOFBOUNDEXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_OUTOFBOUNDEXCEPTION_H

#include <cstddef>
#include <string>
#include <sstream>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class OutOfBoundException represents any exceptions occuring when
        trying to access unallocated memory.
        \brief Out of bounds exception
      */
    template <typename X> class OutOfBoundException :
      public Exception {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs exception from argument and string
      OutOfBoundException(const X& argument, const std::string& msg, const
        std::string& filename = " ", size_t line = 0, const std::string&
        function = " ");
      /// Constructs exception from argument and string
      OutOfBoundException(const X& argument, const X& bound, const std::string&
        msg, const std::string& filename = " ", size_t line = 0, const
        std::string& function = " ");
      /// Destructor
      virtual ~OutOfBoundException() {}
      /** @}
        */
    };

    template <typename X>
    OutOfBoundException<X>::OutOfBoundException(const X& argument, const
        std::string& msg, const std::string& filename, size_t line, const
        std::string& function) : Exception(msg, filename, line, function) {
      std::stringstream stream;
      stream << "[argument = " << argument << "]";
      mOutputMessage.append(stream.str());
    }

    template <typename X>
    OutOfBoundException<X>::OutOfBoundException(const X& argument, const X&
        bound, const std::string& msg, const std::string& filename, size_t line,
        const std::string& function) :
        Exception(msg, filename, line, function) {
      std::stringstream stream;
      stream << "[argument = " << argument << "]";
      stream << "[bound = " << bound << "]";
      mOutputMessage.append(stream.str());
    }
  }
}

#endif // ASLAM_CALIBRATION_EXCEPTIONS_OUTOFBOUNDEXCEPTION_H
