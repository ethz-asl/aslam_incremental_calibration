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

#include <algorithm>

#include <aslam/calibration/exceptions/InvalidOperationException.h>
#include <aslam/calibration/statistics/ChiSquareDistribution.h>
#include <aslam/calibration/statistics/Histogram.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <int M>
    CovarianceEstimator<M>::CovarianceEstimator() :
        _numHistMeasurements(-1) {
    }

    template <int M>
    CovarianceEstimator<M>::~CovarianceEstimator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <int M>
    Eigen::Matrix<double, M, 1> CovarianceEstimator<M>::getMean() const {
      if (!_estimator.getValid())
        throw InvalidOperationException("CovarianceEstimator::getMean(): "
          "not enough measurements");
      return _estimator.getDistribution().getMean();
    }

    template <int M>
    Eigen::Matrix<double, M, M> CovarianceEstimator<M>::getCovariance() const {
      if (!_estimator.getValid())
        throw InvalidOperationException("CovarianceEstimator::getCovariance(): "
          "not enough measurements");
      return _estimator.getDistribution().getCovariance();
    }

    template <int M>
    double CovarianceEstimator<M>::getEstChiSquaredMean() {
      if (!_estimator.getValid())
        throw InvalidOperationException(
          "CovarianceEstimator::getEstChiSquaredMean(): "
          "not enough measurements");
      if (_numHistMeasurements != static_cast<int>(_measurements.size()))
        buildHistogram();
      return _histogram->getMean();
    }

    template <int M>
    double CovarianceEstimator<M>::getEstChiSquaredVariance() {
      if (!_estimator.getValid())
        throw InvalidOperationException(
          "CovarianceEstimator::getEstChiSquaredVariance(): "
          "not enough measurements");
      if (_numHistMeasurements != static_cast<int>(_measurements.size()))
        buildHistogram();
      return _histogram->getVariance();
    }

    template <int M>
    double CovarianceEstimator<M>::getEstChiSquaredMode() {
      if (!_estimator.getValid())
        throw InvalidOperationException(
          "CovarianceEstimator::getEstChiSquaredMode(): "
          "not enough measurements");
      if (_numHistMeasurements != static_cast<int>(_measurements.size()))
        buildHistogram();
      return _histogram->getMode();
    }

    template <int M>
    double CovarianceEstimator<M>::getChiSquaredMean() const {
      return ChiSquareDistribution(M).getMean();
    }

    template <int M>
    double CovarianceEstimator<M>::getChiSquaredVariance() const {
      return ChiSquareDistribution(M).getVariance();
    }

    template <int M>
    double CovarianceEstimator<M>::getChiSquaredMode() const {
      return ChiSquareDistribution(M).getMode();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <int M>
    void CovarianceEstimator<M>::addMeasurement(const
        Eigen::Matrix<double, M, 1>& data) {
      _estimator.addPoint(data);
      _measurements.push_back(data);
    }

    template <int M>
    void CovarianceEstimator<M>::reset() {
      _estimator.reset();
      _measurements.clear();
      _numHistMeasurements = -1;
    }

    template <int M>
    void CovarianceEstimator<M>::buildHistogram() {
      std::vector<double> errors;
      errors.reserve(_measurements.size());
      const NormalDistribution<M>& dist = _estimator.getDistribution();
      for (auto it = _measurements.cbegin(); it != _measurements.cend(); ++it)
       errors.push_back(dist.mahalanobisDistance((*it)));
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
