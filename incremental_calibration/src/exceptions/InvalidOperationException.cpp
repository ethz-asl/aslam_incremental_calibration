/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    InvalidOperationException::InvalidOperationException(const std::string& msg,
        const std::string& filename, size_t line, const std::string& function) :
        Exception(msg, filename, line, function) {
    }

    InvalidOperationException::InvalidOperationException(const
        InvalidOperationException& other) throw() :
        Exception(other) {
    }

    InvalidOperationException& InvalidOperationException::operator =
        (const InvalidOperationException& other) throw() {
      if (this != &other) {
        Exception::operator=(other);
      }
      return *this;
    }

    InvalidOperationException::~InvalidOperationException() throw () {
    }

  }
}
