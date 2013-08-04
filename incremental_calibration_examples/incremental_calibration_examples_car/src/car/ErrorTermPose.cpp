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

#include "aslam/calibration/car/ErrorTermPose.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/RotationVector.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermPose::ErrorTermPose(const aslam::backend::TransformationExpression&
        T, const Input& xm, const Covariance& Q) :
        _T(T),
        _xm(xm),
        _Q(Q) {
      setInvR(_Q.inverse());
      aslam::backend::DesignVariable::set_t dv;
      _T.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermPose::ErrorTermPose(const ErrorTermPose& other) :
        ErrorTermFs<6>(other),
        _T(other._T),
        _xm(other._xm),
        _Q(other._Q) {
    }

    ErrorTermPose& ErrorTermPose::operator =
        (const ErrorTermPose& other) {
      if (this != &other) {
        ErrorTermFs<6>::operator=(other);
       _T = other._T;
       _xm = other._xm;
       _Q = other._Q;
      }
      return *this;
    }

    ErrorTermPose::~ErrorTermPose() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const ErrorTermPose::Input& ErrorTermPose::getInput() const {
      return _xm;
    }

    ErrorTermPose::Input& ErrorTermPose::getInput() {
      return _xm;
    }

    void ErrorTermPose::setInput(const Input& xm) {
      _xm = xm;
    }

    const ErrorTermPose::Covariance& ErrorTermPose::getCovariance() const {
      return _Q;
    }

    ErrorTermPose::Covariance& ErrorTermPose::getCovariance() {
      return _Q;
    }

    void ErrorTermPose::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermPose::evaluateErrorImplementation() {
      const Eigen::Matrix4d T = _T.toTransformationMatrix();
      Input e;
      e.head<3>() = T.topRightCorner<3, 1>();
      const sm::kinematics::EulerAnglesYawPitchRoll ypr;
      e.tail<3>() =
        ypr.rotationMatrixToParameters(T.topLeftCorner<3, 3>());
      error_t error = _xm - e;
      error(3) = sm::kinematics::angleMod(error(3));
      error(4) = sm::kinematics::angleMod(error(4));
      error(5) = sm::kinematics::angleMod(error(5));
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermPose::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {
      Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
      const Eigen::Matrix4d T = _T.toTransformationMatrix();
      J.topRightCorner<3, 3>() =
        sm::kinematics::crossMx(T.topRightCorner<3, 1>());
      const sm::kinematics::EulerAnglesYawPitchRoll ypr;
      J.bottomRightCorner<3, 3>() = (ypr.parametersToSMatrix(
        ypr.rotationMatrixToParameters(T.topLeftCorner<3, 3>()))).inverse();
      _T.evaluateJacobians(jacobians, -J);
    }

  }
}
