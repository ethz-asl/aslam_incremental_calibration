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

#include "aslam/calibration/car/error-terms/ErrorTermVelocities.h"

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermVelocities::ErrorTermVelocities(const EuclideanExpression& r_v_mr,
        const EuclideanExpression& r_om_mr, const Input& r_v_mr_m, const Input&
        r_om_mr_m, const Covariance& sigma2_v, const Covariance& sigma2_om) :
        _r_v_mr(r_v_mr),
        _r_om_mr(r_om_mr),
        _r_v_mr_m(r_v_mr_m),
        _r_om_mr_m(r_om_mr_m),
        _sigma2_v(sigma2_v),
        _sigma2_om(sigma2_om) {
      Eigen::Matrix<double, 6, 6> sigma2 = Eigen::Matrix<double, 6, 6>::Zero();
      sigma2.topLeftCorner<3, 3>() = sigma2_v;
      sigma2.bottomRightCorner<3, 3>() = sigma2_om;
      setInvR(sigma2.inverse());
      DesignVariable::set_t dv;
      _r_v_mr.getDesignVariables(dv);
      _r_om_mr.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermVelocities::ErrorTermVelocities(const ErrorTermVelocities& other) :
        ErrorTermFs<6>(other),
        _r_v_mr(other._r_v_mr),
        _r_om_mr(other._r_om_mr),
        _r_v_mr_m(other._r_v_mr_m),
        _r_om_mr_m(other._r_om_mr_m),
        _sigma2_v(other._sigma2_v),
        _sigma2_om(other._sigma2_om) {
    }

    ErrorTermVelocities& ErrorTermVelocities::operator =
        (const ErrorTermVelocities& other) {
      if (this != &other) {
        ErrorTermFs<6>::operator=(other);
       _r_v_mr = other._r_v_mr;
       _r_om_mr = other._r_om_mr;
       _r_v_mr_m = other._r_v_mr_m;
       _r_om_mr_m = other._r_om_mr_m;
       _sigma2_v = other._sigma2_v;
       _sigma2_om = other._sigma2_om;
      }
      return *this;
    }

    ErrorTermVelocities::~ErrorTermVelocities() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermVelocities::evaluateErrorImplementation() {
      error_t error;
      error.head<3>() = _r_v_mr_m - _r_v_mr.toValue();
      error.tail<3>() = _r_om_mr_m - _r_om_mr.toValue();
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermVelocities::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      Eigen::Matrix<double, 6, 3> Jv = Eigen::Matrix<double, 6, 3>::Zero();
      Jv.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
      _r_v_mr.evaluateJacobians(jacobians, -Jv);
      Eigen::Matrix<double, 6, 3> Jom = Eigen::Matrix<double, 6, 3>::Zero();
      Jom.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
      _r_om_mr.evaluateJacobians(jacobians, -Jom);
    }

  }
}
