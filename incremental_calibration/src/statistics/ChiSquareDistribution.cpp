/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/statistics/ChiSquareDistribution.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ChiSquareDistribution::ChiSquareDistribution(double degrees) :
        GammaDistribution<>(0.5 * degrees, 0.5) {
    }

    ChiSquareDistribution::ChiSquareDistribution(const ChiSquareDistribution&
        other) :
        GammaDistribution<>(other) {
    }

    ChiSquareDistribution& ChiSquareDistribution::operator =
        (const ChiSquareDistribution& other) {
      if (this != &other) {
        GammaDistribution<>::operator=(other);
      }
      return *this;
    }

    ChiSquareDistribution::~ChiSquareDistribution() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void ChiSquareDistribution::read(std::istream& stream) {
    }

    void ChiSquareDistribution::write(std::ostream& stream) const {
      stream << "degrees: " << getDegrees();
    }

    void ChiSquareDistribution::read(std::ifstream& stream) {
    }

    void ChiSquareDistribution::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    void ChiSquareDistribution::setDegrees(double degrees) {
      setShape(degrees * 0.5);
    }

    double ChiSquareDistribution::getDegrees() const {
      return getShape() * 2;
    }

    ChiSquareDistribution::Median ChiSquareDistribution::getMedian() const {
      return getDegrees() * pow(1.0 - 2.0 / (9 * getDegrees()), 3);
    }

  }
}
