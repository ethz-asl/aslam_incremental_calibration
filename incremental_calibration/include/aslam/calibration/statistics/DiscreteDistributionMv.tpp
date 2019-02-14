/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors/Destructor                                                    */
/******************************************************************************/

    template <typename X, int M, int N>
    DiscreteDistribution<X, M, N>::~DiscreteDistribution() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X, int M, int N>
    double DiscreteDistribution<X, M, N>::getValue(const Eigen::Matrix<X, M, N>&
        argument) const {
      return pmf(argument);
    }

  }
}
