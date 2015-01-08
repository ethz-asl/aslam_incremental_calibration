/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
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

/** \file Calibrator.h
    \brief This file defines the Calibrator class which implements the
           calibration algorithm.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_CALIBRATOR_H
#define ASLAM_CALIBRATION_EGOMOTION_CALIBRATOR_H

#include <vector>
#include <unordered_map>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>

#include "aslam/calibration/egomotion/algo/CalibratorOptions.h"
#include "aslam/calibration/egomotion/data/MeasurementsContainer.h"
#include "aslam/calibration/egomotion/data/MotionMeasurement.h"

namespace sm {

  class PropertyTree;

}
namespace bsplines {

  struct NsecTimePolicy;

}
namespace aslam {
  namespace calibration {

    class OptimizationProblemSpline;
    class IncrementalEstimator;
    struct DesignVariables;

    /** The class Calibrator implements the calibration algorithm.
        \brief Calibration algorithm.
      */
    class Calibrator {
    public:
      /** \name Types definitions
        @{
        */
      /// Options for the calibrator
      typedef CalibratorOptions Options;
      /// Shared pointer to incremental estimator
      typedef boost::shared_ptr<IncrementalEstimator> IncrementalEstimatorSP;
      /// Rotation spline
      typedef aslam::splines::OPTBSpline<
        bsplines::UnitQuaternionBSpline<Eigen::Dynamic,
        bsplines::NsecTimePolicy>::CONF>::BSpline RotationSpline;
      /// Rotation spline shared pointer
      typedef boost::shared_ptr<RotationSpline> RotationSplineSP;
      /// Translation spline
      typedef aslam::splines::OPTBSpline<
        bsplines::EuclideanBSpline<Eigen::Dynamic, 3,
        bsplines::NsecTimePolicy>::CONF>::BSpline TranslationSpline;
      /// Euclidean spline shared pointer
      typedef boost::shared_ptr<TranslationSpline> TranslationSplineSP;
      /// Optimization problem shared pointer
      typedef boost::shared_ptr<OptimizationProblemSpline>
        OptimizationProblemSplineSP;
      /// Motion measurements
      typedef MeasurementsContainer<MotionMeasurement>::Type MotionMeasurements;
      /// Design variables shared pointer
      typedef boost::shared_ptr<DesignVariables> DesignVariablesSP;
      /// Self type
      typedef Calibrator Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs calibrator with configuration in property tree
      Calibrator(const sm::PropertyTree& config);
      /// Copy constructor
      Calibrator(const Self& other) = delete;
      /// Copy assignment operator
      Calibrator& operator = (const Self& other) = delete;
      /// Move constructor
      Calibrator(Self&& other) = delete;
      /// Move assignment operator
      Calibrator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~Calibrator() = default;
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the current options
      const Options& getOptions() const {
        return options_;
      }
      /// Returns the current options
      Options& getOptions() {
        return options_;
      }
      /// Returns the incremental estimator
      const IncrementalEstimatorSP getEstimator() const {
        return estimator_;
      }
      /// Returns the incremental estimator
      IncrementalEstimatorSP getEstimator() {
        return estimator_;
      }
      /// Returns the current translation spline
      const TranslationSplineSP& getTranslationSpline() const {
        return translationSpline_;
      }
      /// Returns the current rotation spline
      const RotationSplineSP& getRotationSpline() const {
        return rotationSpline_;
      }
      /// Unprocessed measurements in the pipeline?
      bool unprocessedMeasurements() const {
        return !motionMeasurements_.empty();
      }
      /// Returns the information gain history
      const std::vector<double>& getInformationGainHistory() const {
        return infoGainHistory_;
      }
      /// Returns the calibration variables history
      const std::unordered_map<size_t, std::vector<Eigen::VectorXd> >
          getDesignVariablesHistory() const {
        return designVariablesHistory_;
      }
      /// Returns the calibration design variables
      const DesignVariablesSP& getDesignVariables() const {
        return designVariables_;
      }
      /// Returns the calibration design variables
      DesignVariablesSP& getDesignVariables() {
        return designVariables_;
      }
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds a motion measurement
      void addMotionMeasurement(const MotionMeasurement& motion,
        sm::timing::NsecTime timestamp, size_t idx = 0);
      /// Adds the currently stored measurements to the estimator
      void addMeasurements();
      /// Clears the stored measurements
      void clearMeasurements();
      /// Predicts the stored measurements
      void predict();
      /// Clears the predictions
      void clearPredictions();
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Adds a new measurement
      void addMeasurement(sm::timing::NsecTime timestamp);
      /// Initializes the splines
      void initSplines(size_t idx = 0);
      /// Adds motion error terms
      void addMotionErrorTerms(const OptimizationProblemSplineSP& batch, size_t
        idx = 0);
      /// Predicts motion measurements
      void predictMotion(size_t idx = 0);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options options_;
      /// Incremental estimator
      IncrementalEstimatorSP estimator_;
      /// Calibration design variables
      DesignVariablesSP designVariables_;
      /// Current translation spline
      TranslationSplineSP translationSpline_;
      /// Current rotation spline
      RotationSplineSP rotationSpline_;
      /// Current batch starting timestamp
      sm::timing::NsecTime currentBatchStartTimestamp_;
      /// Last timestamp
      sm::timing::NsecTime lastTimestamp_;
      /// Information gain history
      std::vector<double> infoGainHistory_;
      /// Motion measurements
      std::unordered_map<size_t, MotionMeasurements> motionMeasurements_;
      /// Predicted motion measurements
      std::unordered_map<size_t, MotionMeasurements> motionMeasurementsPred_;
      /// Motion measurements errors
      std::unordered_map<size_t, std::vector<Eigen::VectorXd> >
        motionMeasurementsPredErrors_;
      /// Motion measurements squared errors
      std::unordered_map<size_t, std::vector<double> >
        motionMeasurementsPredErrors2_;
      /// Calibration variables history
      std::unordered_map<size_t, std::vector<Eigen::VectorXd> >
        designVariablesHistory_;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_CALIBRATOR_H
