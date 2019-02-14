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
    ContinuousDistribution<X, M, N>::~ContinuousDistribution() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X, int M, int N>
    double ContinuousDistribution<X, M, N>::
    getValue(const Eigen::Matrix<X, M, N>&
        argument) const {
      return pdf(argument);
    }

  }
}
