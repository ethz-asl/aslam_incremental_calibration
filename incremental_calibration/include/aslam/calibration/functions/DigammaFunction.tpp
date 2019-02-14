/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include <boost/math/special_functions/digamma.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename X>
    DigammaFunction<X>::DigammaFunction() {
    }

    template <typename X>
    DigammaFunction<X>::~DigammaFunction() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X>
    double DigammaFunction<X>::getValue(const VariableType& argument) const {
      return boost::math::digamma<X>(argument);
    }

  }
}
