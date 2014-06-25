/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

/** \file splinesToFile.h
    \brief This file contains a utility to write spline data to file.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_SPLINES_TO_FILE_H
#define ASLAM_CALIBRATION_TIME_DELAY_SPLINES_TO_FILE_H

#include <fstream>

#include <boost/shared_ptr.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>

namespace bsplines {

  struct NsecTimePolicy;

}
namespace aslam {
  namespace calibration {

    class IncrementalEstimator;

    /** \name Types definitions
      @{
      */
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
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Write spline data from incremental estimator to file
    void writeSplines(const IncrementalEstimatorSP& estimator, double dt,
      std::ofstream& stream);
    /// Write spline data from spline structure
    void writeSplines(const TranslationSplineSP& transSpline, const
      RotationSplineSP& rotSpline, double dt, std::ofstream& stream);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_SPLINES_TO_FILE_H
