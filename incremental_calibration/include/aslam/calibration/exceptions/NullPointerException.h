/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file NullPointerException.h
    \brief This file defines the NullPointerException class, which
           represents null pointer exceptions.
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_NULLPOINTEREXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_NULLPOINTEREXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class NullPointerException represents null pointer exceptions.
        \brief Null pointer exception
      */
    class NullPointerException :
      public Exception {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Constructs exception from message
      NullPointerException(const std::string& name = "", const std::string&
        filename = " ", size_t line = 0, const std::string& function = " ");
      /// Copy constructor
      NullPointerException(const NullPointerException& other)
        throw ();
      /// Assignment operator
      NullPointerException& operator = (const NullPointerException&
        other) throw();
      /// Destructor
      virtual ~NullPointerException() throw ();
      /** @}
        */

    protected:

    };

  }
}

#endif // ASLAM_CALIBRATION_EXCEPTIONS_NULLPOINTEREXCEPTION_H
