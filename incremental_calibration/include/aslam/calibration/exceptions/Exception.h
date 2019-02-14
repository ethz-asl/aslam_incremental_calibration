/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Exception.h
    \brief This file defines the Exception class, which is the base class for
           all exceptions.
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_EXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_EXCEPTION_H

#include <cstddef>

#include <stdexcept>
#include <string>

namespace aslam {
  namespace calibration {

    /** The class Exception represents the base class for all exceptions.
        \brief Exception base class
      */
    class Exception :
      public std::exception {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Constructs exception from message
      Exception(const std::string& msg = "", const std::string&
        filename = " ", size_t line = 0, const std::string& function = " ");
      /// Copy constructor
      Exception(const Exception& other) throw ();
      /// Assignment operator
      Exception& operator = (const Exception& other) throw();
      /// Destructor
      virtual ~Exception() throw ();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the exception string
      virtual const char* what() const throw();
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// Message in the exception
      std::string mMsg;
      /// Filename where the exception occurs
      std::string mFilename;
      /// Function where the exception occurs
      std::string mFunction;
      /// Line number where the exception occurs
      size_t mLine;
      /// Output message
      std::string mOutputMessage;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EXCEPTIONS_EXCEPTION_H
