/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

namespace OuterProduct {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  template <typename X, size_t M, size_t N>
  Eigen::Matrix<X, M, N> compute(const Eigen::Matrix<X, M, 1>& v1,
      const Eigen::Matrix<X, N, 1>& v2) {
    return v1 * v2.transpose();
  };

  template <typename X, size_t M>
  Eigen::Matrix<X, M, M> compute(const Eigen::Matrix<X, M, 1>& v) {
    Eigen::Matrix<X, M, M> result = Eigen::Matrix<X, M, M>::Zero(v.size(),
      v.size());
    for (size_t i = 0; i < (size_t)v.size(); ++i)
      for (size_t j = i; j < (size_t)v.size(); ++j) {
        result(i, j) = v(i) * v(j);
        result(j, i) = result(i, j);
      }
    return result;
  };

}
