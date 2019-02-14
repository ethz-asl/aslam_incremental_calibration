/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors/Destructor                                                    */
/******************************************************************************/

    template <typename X>
    DiscreteDistribution<X>::~DiscreteDistribution() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X>
    double DiscreteDistribution<X>::getValue(const X& argument) const {
      return pmf(argument);
    }

  }
}
