/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "aslam/calibration/car/CovarianceEstimator.h"

#include <algorithm>

#include <aslam/calibration/exceptions/InvalidOperationException.h>
#include <aslam/calibration/statistics/ChiSquareDistribution.h>
#include <aslam/calibration/statistics/Histogram.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CovarianceEstimator<1>::CovarianceEstimator() :
        _numHistMeasurements(-1) {
    }

    CovarianceEstimator<1>::~CovarianceEstimator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    Eigen::Matrix<double, 1, 1> CovarianceEstimator<1>::getMean() const {
      if (!_estimator.getValid())
        throw InvalidOperationException("CovarianceEstimator::getMean(): "
          "not enough measurements");
      return (Eigen::Matrix<double, 1, 1>()
        << _estimator.getDistribution().getMean()).finished();
    }

    Eigen::Matrix<double, 1, 1> CovarianceEstimator<1>::getCovariance() const {
      if (!_estimator.getValid())
        throw InvalidOperationException("CovarianceEstimator::getCovariance(): "
          "not enough measurements");
      return (Eigen::Matrix<double, 1, 1>()
        << _estimator.getDistribution().getVariance()).finished();
    }

    double CovarianceEstimator<1>::getEstChiSquaredMean() {
      if (!_estimator.getValid())
        throw InvalidOperationException(
          "CovarianceEstimator::getEstChiSquaredMean(): "
          "not enough measurements");
      if (_numHistMeasurements != static_cast<int>(_measurements.size()))
        buildHistogram();
      return _histogram->getMean();
    }

    double CovarianceEstimator<1>::getEstChiSquaredVariance() {
      if (!_estimator.getValid())
        throw InvalidOperationException(
          "CovarianceEstimator::getEstChiSquaredVariance(): "
          "not enough measurements");
      if (_numHistMeasurements != static_cast<int>(_measurements.size()))
        buildHistogram();
      return _histogram->getVariance();
    }

    double CovarianceEstimator<1>::getEstChiSquaredMode() {
      if (!_estimator.getValid())
        throw InvalidOperationException(
          "CovarianceEstimator::getEstChiSquaredMode(): "
          "not enough measurements");
      if (_numHistMeasurements != static_cast<int>(_measurements.size()))
        buildHistogram();
      return _histogram->getMode();
    }

    double CovarianceEstimator<1>::getChiSquaredMean() const {
      return ChiSquareDistribution().getMean();
    }

    double CovarianceEstimator<1>::getChiSquaredVariance() const {
      return ChiSquareDistribution().getVariance();
    }

    double CovarianceEstimator<1>::getChiSquaredMode() const {
      return ChiSquareDistribution().getMode();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CovarianceEstimator<1>::addMeasurement(const
        Eigen::Matrix<double, 1, 1>& data) {
      _estimator.addPoint(data(0));
      _measurements.push_back(data);
    }

    void CovarianceEstimator<1>::reset() {
      _estimator.reset();
      _measurements.clear();
      _numHistMeasurements = -1;
    }

    void CovarianceEstimator<1>::buildHistogram() {
      std::vector<double> errors;
      errors.reserve(_measurements.size());
      const NormalDistribution<1>& dist = _estimator.getDistribution();
      for (auto it = _measurements.cbegin(); it != _measurements.cend(); ++it)
       errors.push_back(dist.mahalanobisDistance((*it)(0)));
      auto max = std::max_element(errors.begin(), errors.end());
      auto min = std::min_element(errors.begin(), errors.end());
      if (max == errors.end() || min == errors.end())
        throw InvalidOperationException(
          "CovarianceEstimator::buildHistogram(): not enough measurements");
      _histogram = std::make_shared<Histogram<double, 1> >(*min, *max, 0.01);
      _histogram->addSamples(errors);
      _numHistMeasurements = _measurements.size();
    }

  }
}
