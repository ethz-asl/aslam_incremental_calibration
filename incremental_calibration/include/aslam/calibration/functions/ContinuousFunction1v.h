/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ContinuousFunction1v.h
    \brief This file contains an interface to the univariate continuous
           functions
  */

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The ContinuousFunction1v class represents an interface to the univariate
        continuous functions.
        \brief Univariate continuous function
      */
    template <typename Y, typename X> class ContinuousFunction<Y, X> :
      public virtual Function<Y, X> {
    public:
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

      /** \name Accessors
        @{
        */
      /// Access the function's number of variables
      virtual size_t getNumVariables() const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/functions/ContinuousFunction1v.tpp"
