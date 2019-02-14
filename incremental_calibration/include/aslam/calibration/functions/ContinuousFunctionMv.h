/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ContinuousFunctionMv.h
    \brief This file defines the class ContinuousFunctionMv, which is an
           interface to the multivariate continuous functions
  */

#include <Eigen/Core>

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The class ContinuousFunctionMv is an interface to the multivariate
        continuous functions.
        \brief Multivariate continuous function
      */
    template <typename Y, typename X, int M, int N>
    class ContinuousFunction :
      public virtual Function<Y, Eigen::Matrix<X, M, N> > {
    public:
      /// \cond
      // Template parameters assertion
      static_assert(M > 0 || M == Eigen::Dynamic, "M should be larger than 0!");
      static_assert(N > 0 || N == Eigen::Dynamic, "N should be larger than 0!");
      /// \endcond

      /** \name Types
        @{
        */
      /// Domain type
      typedef X DomainType;
      /// Codomain type
      typedef Y CodomainType;
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~ContinuousFunction();
      /** @}
        */

    };

  }
}

#include "aslam/calibration/functions/ContinuousFunctionMv.tpp"
