/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors/Destructor                                                    */
/******************************************************************************/

    template <typename Y, typename X>
    DiscreteFunction<Y, X>::~DiscreteFunction() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename Y, typename X>
    size_t DiscreteFunction<Y, X>::getNumVariables() const {
      return 1;
    }

  }
}
