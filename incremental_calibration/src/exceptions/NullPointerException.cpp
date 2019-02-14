/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/exceptions/NullPointerException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    NullPointerException::NullPointerException(const std::string&
        name, const std::string& filename, size_t line, const std::string&
        function) : Exception(name + std::string(" is a null pointer"),
        filename, line, function) {
    }

    NullPointerException::NullPointerException(const NullPointerException&
        other) throw() :
        Exception(other) {
    }

    NullPointerException& NullPointerException::operator = (const
        NullPointerException& other) throw() {
      if (this != &other) {
        Exception::operator=(other);
      }
      return *this;
    }

    NullPointerException::~NullPointerException() throw () {
    }

  }
}
