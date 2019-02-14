/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/functions/IncompleteGammaQFunction.h"

#include <boost/math/special_functions/gamma.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncompleteGammaQFunction::IncompleteGammaQFunction(double alpha) :
        mAlpha(alpha) {
    }

    IncompleteGammaQFunction::IncompleteGammaQFunction(const
        IncompleteGammaQFunction& other) :
        mAlpha(other.mAlpha) {
    }

    IncompleteGammaQFunction& IncompleteGammaQFunction::operator = (const
        IncompleteGammaQFunction& other) {
      if (this != &other) {
        mAlpha = other.mAlpha;
      }
      return *this;
    }

    IncompleteGammaQFunction::~IncompleteGammaQFunction() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void IncompleteGammaQFunction::read(std::istream& stream) {
    }

    void IncompleteGammaQFunction::write(std::ostream& stream) const {
      stream << "alpha: " << mAlpha;
    }

    void IncompleteGammaQFunction::read(std::ifstream& stream) {
    }

    void IncompleteGammaQFunction::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double IncompleteGammaQFunction::getValue(const VariableType& argument)
        const {
      return boost::math::gamma_q(mAlpha, argument);
    }

    double IncompleteGammaQFunction::getAlpha() const {
      return mAlpha;
    }

    void IncompleteGammaQFunction::setAlpha(double alpha) {
      mAlpha = alpha;
    }

  }
}
