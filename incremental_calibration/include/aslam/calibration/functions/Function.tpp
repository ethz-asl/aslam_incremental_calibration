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
    Function<Y, X>::~Function() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename Y, typename X>
    Y Function<Y, X>::operator()(const X& argument) const {
      return getValue(argument);
    }

  }
}
