/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file DiscreteFunction1v.h
    \brief This file defines the class DiscreteFunction1v, which is an interface
           to the univariate functions
  */

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The class DiscreteFunction1v is an interface to the univariate discrete
        functions.
        \brief Univariate discrete function
      */
    template <typename Y, typename X> class DiscreteFunction<Y, X> :
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
      virtual ~DiscreteFunction();
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

#include "aslam/calibration/functions/DiscreteFunction1v.tpp"
