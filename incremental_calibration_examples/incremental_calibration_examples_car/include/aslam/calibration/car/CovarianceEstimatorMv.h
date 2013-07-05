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

/** \file CovarianceEstimatorMv.h
    \brief This file defines the CovarianceEstimator class which implements
           a covariance estimator for multivariate data.
  */

#include <vector>
#include <memory>

#include <Eigen/Core>

#include <aslam/calibration/statistics/EstimatorML.h>
#include <aslam/calibration/statistics/NormalDistribution.h>

namespace aslam {
  namespace calibration {

    template <typename T, int M> class Histogram;

    /** The class CovarianceEstimator implements a covariance estimator for
        multivariate data.
        \brief Covariance estimator for multivariate data.
      */
    template <int M> class CovarianceEstimator {
    public:
      /** \name Types definitions
        @{
        */
      /// Self type
      typedef CovarianceEstimator Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      CovarianceEstimator();
      /// Copy constructor
      CovarianceEstimator(const Self& other) = delete;
      /// Copy assignment operator
      CovarianceEstimator& operator = (const Self& other) = delete;
      /// Move constructor
      CovarianceEstimator(Self&& other) = delete;
      /// Move assignment operator
      CovarianceEstimator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~CovarianceEstimator();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the estimated mean
      Eigen::Matrix<double, M, 1> getMean() const;
      /// Returns the estimated covariance
      Eigen::Matrix<double, M, M> getCovariance() const;
      /// Returns the mean of the histogram of errors
      double getEstChiSquaredMean();
      /// Returns the variance of the histogram of errors
      double getEstChiSquaredVariance();
      /// Returns the mode of the histogram of errors
      double getEstChiSquaredMode();
      /// Returns the mean of the chi-square distribution
      double getChiSquaredMean() const;
      /// Returns the variance of the chi-square distribution
      double getChiSquaredVariance() const;
      /// Returns the mode of the chi-square distribution
      double getChiSquaredMode() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds a measurement to the estimator
      void addMeasurement(const Eigen::Matrix<double, M, 1>& data);
      /// Resets the estimator
      void reset();
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Creates the histogram from the data
      void buildHistogram();
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Estimator
      EstimatorML<NormalDistribution<M> > _estimator;
      /// Stored measurements
      std::vector<Eigen::Matrix<double, M, 1> > _measurements;
      /// Histogram of the errors
      std::shared_ptr<Histogram<double, 1> > _histogram;
      /// Number of measurements when histogram was built
      int _numHistMeasurements;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/car/CovarianceEstimatorMv.tpp"
