/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Statics                                                                    */
/******************************************************************************/

    template <class C> C* Singleton<C>::instance = 0;

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <class C>
    Singleton<C>::Singleton() {
      if (instance)
        throw InvalidOperationException(
          "Singleton<C>::Singleton(): a singleton cannot be instantiated");
      instance = (C*)this;
    }

    template <class C>
    Singleton<C>::~Singleton() {
      instance = 0;
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <class C>
    C& Singleton<C>::getInstance() {
      if (!instance)
        new C();
      return *instance;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <class C>
    bool Singleton<C>::exists() {
      return (instance != 0);
    }

  }
}
