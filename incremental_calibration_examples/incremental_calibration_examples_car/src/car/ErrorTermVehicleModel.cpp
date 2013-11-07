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

#include "aslam/calibration/car/ErrorTermVehicleModel.h"

#include <Eigen/Dense>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermVehicleModel::ErrorTermVehicleModel(
        const aslam::backend::EuclideanExpression& v_oo,
        const aslam::backend::EuclideanExpression& om_oo, const Covariance& Q) :
        _v_oo(v_oo),
        _om_oo(om_oo),
        _Q(Q) {
      setInvR(_Q.inverse());
      aslam::backend::DesignVariable::set_t dv;
      v_oo.getDesignVariables(dv);
      om_oo.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermVehicleModel::ErrorTermVehicleModel(const
        ErrorTermVehicleModel& other) :
        ErrorTermFs<4>(other),
        _v_oo(other._v_oo),
        _om_oo(other._om_oo),
        _Q(other._Q) {
    }

    ErrorTermVehicleModel& ErrorTermVehicleModel::operator =
        (const ErrorTermVehicleModel& other) {
      if (this != &other) {
        ErrorTermFs<4>::operator=(other);
       _v_oo = other._v_oo;
       _om_oo = other._om_oo;
       _Q = other._Q;
      }
      return *this;
    }

    ErrorTermVehicleModel::~ErrorTermVehicleModel() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const ErrorTermVehicleModel::Covariance&
        ErrorTermVehicleModel::getCovariance() const {
      return _Q;
    }

    ErrorTermVehicleModel::Covariance& ErrorTermVehicleModel::getCovariance() {
      return _Q;
    }

    void ErrorTermVehicleModel::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermVehicleModel::evaluateErrorImplementation() {
      // build the error term
      error_t error;
      error(0) = _v_oo.toValue()(1);
//      error(1) = _v_oo.toValue()(2);
      error(1) = 0;
//      error(2) = _om_oo.toValue()(0);
//      error(3) = _om_oo.toValue()(1);
      error(2) = 0;
      error(3) = 0;
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermVehicleModel::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {
      // Jacobian with respect to v_oo
      Eigen::Matrix<double, 4, 3> Hv = Eigen::Matrix<double, 4, 3>::Zero();
      Hv(0, 1) = 1.0;
//      Hv(1, 2) = 1.0;

      // Jacobian with respect to om_oo
      Eigen::Matrix<double, 4, 3> Ho = Eigen::Matrix<double, 4, 3>::Zero();
//      Ho(2, 0) = 1.0;
//      Ho(3, 1) = 1.0;

      // pass the Jacobians with the chain rule
      _v_oo.evaluateJacobians(jacobians, Hv);
      _om_oo.evaluateJacobians(jacobians, Ho);
    }

  }
}
