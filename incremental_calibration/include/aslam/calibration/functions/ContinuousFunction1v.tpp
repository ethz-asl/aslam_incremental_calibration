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
    ContinuousFunction<Y, X>::~ContinuousFunction() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename Y, typename X>
    size_t ContinuousFunction<Y, X>::getNumVariables() const {
      return 1;
    }

  }
}
