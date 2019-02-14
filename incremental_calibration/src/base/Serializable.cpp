/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors/Destructor                                                    */
/******************************************************************************/

    Serializable::~Serializable() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    std::ostream& operator << (std::ostream& stream, const Serializable& obj) {
      obj.write(stream);
      return stream;
    }

    std::istream& operator >> (std::istream& stream, Serializable& obj) {
      obj.read(stream);
      return stream;
    }

    std::ofstream& operator << (std::ofstream& stream,
        const Serializable& obj) {
      obj.write(stream);
      return stream;
    }

    std::ifstream& operator >> (std::ifstream& stream, Serializable& obj) {
      obj.read(stream);
      return stream;
    }

  }
}
