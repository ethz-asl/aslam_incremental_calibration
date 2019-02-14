/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/functions/IncompleteGammaPFunction.h"

#include <boost/math/special_functions/gamma.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncompleteGammaPFunction::IncompleteGammaPFunction(double alpha) :
        mAlpha(alpha) {
    }

    IncompleteGammaPFunction::IncompleteGammaPFunction(const
        IncompleteGammaPFunction& other) :
        mAlpha(other.mAlpha) {
    }

    IncompleteGammaPFunction& IncompleteGammaPFunction::operator = (const
        IncompleteGammaPFunction& other) {
      if (this != &other) {
        mAlpha = other.mAlpha;
      }
      return *this;
    }

    IncompleteGammaPFunction::~IncompleteGammaPFunction() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void IncompleteGammaPFunction::read(std::istream& stream) {
    }

    void IncompleteGammaPFunction::write(std::ostream& stream) const {
      stream << "alpha: " << mAlpha;
    }

    void IncompleteGammaPFunction::read(std::ifstream& stream) {
    }

    void IncompleteGammaPFunction::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double IncompleteGammaPFunction::getValue(const VariableType& argument)
        const {
      return boost::math::gamma_p(mAlpha, argument);
    }

    double IncompleteGammaPFunction::getAlpha() const {
      return mAlpha;
    }

    void IncompleteGammaPFunction::setAlpha(double alpha) {
      mAlpha = alpha;
    }

  }
}
