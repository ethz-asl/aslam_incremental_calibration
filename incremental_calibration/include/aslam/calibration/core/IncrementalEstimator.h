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

/** \file IncrementalEstimator.h
    \brief This file defines the IncrementalEstimator class, which implements
           an incremental estimator for robotic calibration problems.
  */

#ifndef ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H
#define ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>

namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm;
    class SparseQrLinearSystemSolver;

  }
  namespace calibration {

    /** The class IncrementalEstimator implements an incremental estimator
        for robotic calibration problems.
        \brief Incremental estimator
      */
    class IncrementalEstimator {
    public:
      /** \name Types definitions
        @{
        */
      /// Design variables container
      typedef std::vector<boost::shared_ptr<aslam::backend::DesignVariable> >
        DVContainer;
      /// Design variables container iterator
      typedef DVContainer::iterator DVContainerIt;
      /// Design variables const iterator
      typedef DVContainer::const_iterator DVContainerConstIt;
      /// Error terms container
      typedef std::vector<boost::shared_ptr<aslam::backend::ErrorTerm> >
        ETContainer;
      /// Error terms container iterator
      typedef ETContainer::iterator ETContainerIt;
      /// Error terms const iterator
      typedef ETContainer::const_iterator ETContainerConstIt;
      /// Self type
      typedef IncrementalEstimator Self;
      /// Solver type
      typedef aslam::backend::SparseQrLinearSystemSolver LinearSolver;
      /// Options for the incremental estimator
      struct Options {
        /// Mutual information threshold
        double _miTol;
        /// QR treshold for rank-deficiency
        double _qrTol;
        /// Verbosity of the optimizer
        bool _verbose;
        /// Perform column normalization
        bool _colNorm;
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs estimator with specific marginalized variables and options
      IncrementalEstimator(const DVContainer& designVariablesMarg,
        const DVContainer& designVariablesInv = DVContainer(),
        const Options& options = _defaultOptions);
      /// Copy constructor
      IncrementalEstimator(const Self& other) = delete;
      /// Copy assignment operator
      IncrementalEstimator& operator = (const Self& other) = delete;
      /// Move constructor
      IncrementalEstimator(Self&& other) = delete;
      /// Move assignment operator
      IncrementalEstimator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~IncrementalEstimator();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Add a measurement batch to the estimator
      bool addMeasurementBatch(const ETContainer& errorTermsNew,
        const DVContainer& designVariablesNew);
      /// Returns the covariance matrix of the marginalized variables
      Eigen::MatrixXd getMarginalizedCovariance() const;
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns informative error terms container
      const ETContainer& getErrorTermsInfo() const;
      /// Returns informative error terms container
      ETContainer& getErrorTermsInfo();
      /// Returns error terms begin iterator
      ETContainerIt getETBegin();
      /// Returns informative error terms cbegin iterator
      ETContainerConstIt getETCBegin() const;
      /// Returns informative error terms end iterator
      ETContainerIt getETEnd();
      /// Returns informative error terms cend iterator
      ETContainerConstIt getETCEnd() const;
      /// Returns marginalized design variables container
      const DVContainer& getDesignVariablesMarg() const;
      /// Returns marginalized design variables container
      DVContainer& getDesignVariablesMarg();
      /// Returns marginalized design variables begin iterator
      DVContainerIt getDVMBegin();
      /// Returns marginalized design variables cbegin iterator
      DVContainerConstIt getDVMCBegin() const;
      /// Returns marginalized design variables end iterator
      DVContainerIt getDVMEnd();
      /// Returns marginalized design variables cend iterator
      DVContainerConstIt getDVMCEnd() const;
      /// Returns informative design variables container
      const DVContainer& getDesignVariablesInfo() const;
      /// Returns informative design variables container
      DVContainer& getDesignVariablesInfo();
      /// Returns informative design variables begin iterator
      DVContainerIt getDVIBegin();
      /// Returns informative design variables cbegin iterator
      DVContainerConstIt getDVICBegin() const;
      /// Returns informative design variables end iterator
      DVContainerIt getDVIEnd();
      /// Returns informative design variables cend iterator
      DVContainerConstIt getDVICEnd() const;
      /// Returns the current options
      const Options& getOptions() const;
      /// Returns the current options
      Options& getOptions();
      /// Returns the last mutual information
      double getMutualInformation() const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Stored informative error terms
      ETContainer _errorTermsInfo;
      /// Stored informative design variables
      DVContainer _designVariablesInfo;
      /// Stored marginalized design variables
      DVContainer _designVariablesMarg;
      /// Stored time-invariant design variables
      DVContainer _designVariablesInv;
      /// Previous sum log diag(R)
      double _sumLogDiagROld;
      /// Mutual information
      double _mi;
      /// Options
      Options _options;
      /// Default options
      static const struct Options _defaultOptions;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H
