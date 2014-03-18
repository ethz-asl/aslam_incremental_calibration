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

#include "aslam/calibration/car/error-terms/ErrorTermWheel.h"

#include <cmath>

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermWheel::ErrorTermWheel(const EuclideanExpression& v_v_mw, const
        ScalarExpression& k, double measurement, const Covariance& sigma2Wheel,
        bool frontEnabled) :
        _v_v_mw(v_v_mw),
        _k(k),
        _measurement(measurement),
        _sigma2Wheel(sigma2Wheel),
        _frontEnabled(frontEnabled) {
      setInvR(_sigma2Wheel.inverse());
      DesignVariable::set_t dv;
      v_v_mw.getDesignVariables(dv);
      k.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermWheel::ErrorTermWheel(const ErrorTermWheel& other) :
        ErrorTermFs<3>(other),
        _v_v_mw(other._v_v_mw),
        _k(other._k),
        _measurement(other._measurement),
        _sigma2Wheel(other._sigma2Wheel),
        _frontEnabled(other._frontEnabled) {
    }

    ErrorTermWheel& ErrorTermWheel::operator = (const ErrorTermWheel& other) {
      if (this != &other) {
        ErrorTermFs<3>::operator=(other);
       _v_v_mw = other._v_v_mw;
       _k = other._k;
       _measurement = other._measurement;
       _sigma2Wheel = other._sigma2Wheel;
       _frontEnabled = other._frontEnabled;
      }
      return *this;
    }

    ErrorTermWheel::~ErrorTermWheel() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double ErrorTermWheel::getMeasurement() const {
      return _measurement;
    }

    void ErrorTermWheel::setMeasurement(double measurement) {
      _measurement = measurement;
    }

    const ErrorTermWheel::Covariance& ErrorTermWheel::getCovariance() const {
      return _sigma2Wheel;
    }

    void ErrorTermWheel::setCovariance(const Covariance& sigma2Wheel) {
      _sigma2Wheel = sigma2Wheel;
    }

    bool ErrorTermWheel::getFrontEnabled() const {
      return _frontEnabled;
    }

    void ErrorTermWheel::setFrontEnabled(bool enabled) {
      _frontEnabled = enabled;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermWheel::evaluateErrorImplementation() {
      error_t error;
      const double v0 = _v_v_mw.toValue()(0);
      const double v1 = _v_v_mw.toValue()(1);
      const double v2 = _v_v_mw.toValue()(2);
      const double k = _k.toScalar();
      if (_frontEnabled) {
        const double temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
        error(0) = _measurement - k * (v0 / temp + v1 * v1 / (v0 * temp));
        error(1) = 0.0;
        error(2) = -v2;
      }
      else {
        error(0) = _measurement - k * v0;
        error(1) = -v1;
        error(2) = -v2;
      }
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermWheel::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      const double k = _k.toScalar();
      const double v0 = _v_v_mw.toValue()(0);
      Eigen::Matrix<double, 3, 3> J_v_v_mw =
        Eigen::Vector3d(k, 1.0, 1.0).asDiagonal();
      Eigen::Matrix<double, 3, 1> J_k(v0, 0.0, 0.0);
      if (_frontEnabled) {
        const double v1 = _v_v_mw.toValue()(1);
        const double temp1 = std::sqrt(v0 * v0 + v1 * v1);
        J_v_v_mw(0, 0) = k * v0 / temp1;
        J_v_v_mw(0, 1) = k * v1 / temp1;
        J_v_v_mw(1, 1) = 0.0;
        const double temp2 = std::sqrt(v1 * v1 / (v0 * v0) + 1);
        J_k(0, 0) = (v0 / temp2 + v1 * v1 / (v0 * temp2));
      }
      _v_v_mw.evaluateJacobians(jacobians, -J_v_v_mw);
      _k.evaluateJacobians(jacobians, -J_k);
    }

  }
}
