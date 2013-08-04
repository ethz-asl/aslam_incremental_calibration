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

#include "aslam/calibration/car/ErrorTermRws.h"

#include <Eigen/Dense>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermRws::ErrorTermRws(
        const aslam::backend::EuclideanExpression& v_oo,
        const aslam::backend::EuclideanExpression& om_oo,
        VectorDesignVariable<11>* params,
        const Input& odo, const Covariance& Q) :
        _v_oo(v_oo),
        _om_oo(om_oo),
        _params(params),
        _odo(odo),
        _Q(Q) {
      setInvR(_Q.inverse());
      aslam::backend::DesignVariable::set_t dv;
      v_oo.getDesignVariables(dv);
      om_oo.getDesignVariables(dv);
      dv.insert(params);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermRws::ErrorTermRws(const ErrorTermRws& other) :
        ErrorTermFs<2>(other),
        _v_oo(other._v_oo),
        _om_oo(other._om_oo),
        _params(other._params),
        _odo(other._odo),
        _Q(other._Q) {
    }

    ErrorTermRws& ErrorTermRws::operator = (const ErrorTermRws& other) {
      if (this != &other) {
        ErrorTermFs<2>::operator=(other);
       _v_oo = other._v_oo;
       _om_oo = other._om_oo;
       _params = other._params;
       _odo = other._odo;
       _Q = other._Q;
      }
      return *this;
    }

    ErrorTermRws::~ErrorTermRws() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const ErrorTermRws::Input& ErrorTermRws::getInput() const {
      return _odo;
    }

    ErrorTermRws::Input& ErrorTermRws::getInput() {
      return _odo;
    }

    void ErrorTermRws::setInput(const Input& odo) {
      _odo = odo;
    }

    const ErrorTermRws::Covariance& ErrorTermRws::getCovariance() const {
      return _Q;
    }

    ErrorTermRws::Covariance& ErrorTermRws::getCovariance() {
      return _Q;
    }

    void ErrorTermRws::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermRws::evaluateErrorImplementation() {
      // useful pre-computations
      const double v_oo_x = _v_oo.toValue()(0);
      const double om_oo_z = _om_oo.toValue()(2);
      const double e_r = _params->getValue()(1);
      const double k_rl = _params->getValue()(7);
      const double k_rr = _params->getValue()(8);

      // build the error term
      error_t error;
      error(0) = _odo(0) - (v_oo_x - e_r * om_oo_z) / k_rl;
      error(1) = _odo(1) - (v_oo_x + e_r * om_oo_z) / k_rr;
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermRws::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {
      // useful pre-computations
      const double v_oo_x = _v_oo.toValue()(0);
      const double om_oo_z = _om_oo.toValue()(2);
      const double e_r = _params->getValue()(1);
      const double k_rl = _params->getValue()(7);
      const double k_rr = _params->getValue()(8);

      // Jacobian with respect to odometry parameters
      Eigen::Matrix<double, 2, 11> Ht = Eigen::Matrix<double, 2, 11>::Zero();

      // Jacobian with respect to v_oo
      Eigen::Matrix<double, 2, 3> Hv = Eigen::Matrix<double, 2, 3>::Zero();

      // Jacobian with respect to om_oo
      Eigen::Matrix<double, 2, 3> Ho = Eigen::Matrix<double, 2, 3>::Zero();

      // v_rl measurement
      Ht(0, 1) = -om_oo_z / k_rl;
      Ht(0, 7) = -(v_oo_x - e_r * om_oo_z) / k_rl / k_rl;
      Hv(0, 0) = 1.0 / k_rl;
      Ho(0, 2) = -e_r / k_rl;

      // v_rr measurement
      Ht(1, 1) = om_oo_z / k_rr;
      Ht(1, 8) = -(v_oo_x + e_r * om_oo_z) / k_rr / k_rr;
      Hv(1, 0) = 1.0 / k_rr;
      Ho(1, 2) = e_r / k_rr;

      // pass the Jacobians with the chain rule
      _v_oo.evaluateJacobians(jacobians, -Hv);
      _om_oo.evaluateJacobians(jacobians, -Ho);
      jacobians.add(_params, -Ht);
    }

  }
}
