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
    ContinuousDistribution<X>::~ContinuousDistribution() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X>
    double ContinuousDistribution<X>::getValue(const X& argument) const {
      return pdf(argument);
    }

  }
}
