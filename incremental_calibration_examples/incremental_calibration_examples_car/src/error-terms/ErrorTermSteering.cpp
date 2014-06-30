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

#include "aslam/calibration/car/error-terms/ErrorTermSteering.h"

#include <cmath>

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermSteering::ErrorTermSteering(const
        aslam::backend::EuclideanExpression& v_v_mw, double measurement, double
        sigma2, VectorDesignVariable<4>* params) :
        _v_v_mw(v_v_mw),
        _measurement(measurement),
        _sigma2(sigma2),
        _params(params) {
      Eigen::Matrix<double, 1, 1> sigma2_mat;
      sigma2_mat << sigma2;
      setInvR(sigma2_mat.inverse());
      aslam::backend::DesignVariable::set_t dv;
      v_v_mw.getDesignVariables(dv);
      dv.insert(params);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermSteering::ErrorTermSteering(const ErrorTermSteering& other) :
        ErrorTermFs<1>(other),
        _v_v_mw(other._v_v_mw),
        _measurement(other._measurement),
        _sigma2(other._sigma2),
        _params(other._params) {
    }

    ErrorTermSteering& ErrorTermSteering::operator = (const ErrorTermSteering&
        other) {
      if (this != &other) {
        ErrorTermFs<1>::operator=(other);
       _v_v_mw = other._v_v_mw;
       _measurement = other._measurement;
       _sigma2 = other._sigma2;
       _params = other._params;
      }
      return *this;
    }

    ErrorTermSteering::~ErrorTermSteering() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double ErrorTermSteering::getMeasurement() const {
      return _measurement;
    }

    void ErrorTermSteering::setMeasurement(double measurement) {
      _measurement = measurement;
    }

    double ErrorTermSteering::getVariance() const {
      return _sigma2;
    }

    void ErrorTermSteering::setVariance(double sigma2) {
      _sigma2 = sigma2;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermSteering::evaluateErrorImplementation() {
      error_t error;
      const double a0 = _params->getValue()(0);
      const double a1 = _params->getValue()(1);
      const double a2 = _params->getValue()(2);
      const double a3 = _params->getValue()(3);
      const double v0 = _v_v_mw.toValue()(0);
      const double v1 = _v_v_mw.toValue()(1);
      const double phi = std::atan2(v1, v0);
      error(0) = a0 + a1 * _measurement + a2 * _measurement * _measurement +
        a3 * _measurement * _measurement * _measurement - phi;
      error(0) = sm::kinematics::angleMod(error(0));
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermSteering::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {
      const double v0 = _v_v_mw.toValue()(0);
      const double v1 = _v_v_mw.toValue()(1);
      Eigen::Matrix<double, 1, 4> J_a = Eigen::Matrix<double, 1, 4>::Zero();
      J_a(0, 0) = 1;
      J_a(0, 1) = _measurement;
      J_a(0, 2) = _measurement * _measurement;
      J_a(0, 3) = _measurement * _measurement * _measurement;
      jacobians.add(_params, J_a);
      Eigen::Matrix<double, 1, 3> J_v_v_mw =
        Eigen::Matrix<double, 1, 3>::Zero();
      J_v_v_mw(0, 0) = -v1 / (v0 * v0 * (v1 * v1 / (v0 * v0) + 1));
      J_v_v_mw(0, 1) = 1 / (v0 * (v1 * v1 / (v0 * v0) + 1));
      _v_v_mw.evaluateJacobians(jacobians, -J_v_v_mw);
    }

  }
}
