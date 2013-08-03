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

#include "aslam/calibration/car/ErrorTermDMI.h"

#include <Eigen/Dense>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermDMI::ErrorTermDMI(
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

    ErrorTermDMI::ErrorTermDMI(const ErrorTermDMI& other) :
        ErrorTermFs<1>(other),
        _v_oo(other._v_oo),
        _om_oo(other._om_oo),
        _params(other._params),
        _odo(other._odo),
        _Q(other._Q) {
    }

    ErrorTermDMI& ErrorTermDMI::operator = (const ErrorTermDMI& other) {
      if (this != &other) {
        ErrorTermFs<1>::operator=(other);
       _v_oo = other._v_oo;
       _om_oo = other._om_oo;
       _params = other._params;
       _odo = other._odo;
       _Q = other._Q;
      }
      return *this;
    }

    ErrorTermDMI::~ErrorTermDMI() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const ErrorTermDMI::Input& ErrorTermDMI::getInput() const {
      return _odo;
    }

    ErrorTermDMI::Input& ErrorTermDMI::getInput() {
      return _odo;
    }

    void ErrorTermDMI::setInput(const Input& odo) {
      _odo = odo;
    }

    const ErrorTermDMI::Covariance& ErrorTermDMI::getCovariance() const {
      return _Q;
    }

    ErrorTermDMI::Covariance& ErrorTermDMI::getCovariance() {
      return _Q;
    }

    void ErrorTermDMI::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermDMI::evaluateErrorImplementation() {
      // useful pre-computations
      const double v_oo_x = _v_oo.toValue()(0);
      const double om_oo_z = _om_oo.toValue()(0);
      const double e_r = _params->getValue()(1);

      // build the error term
      error_t error;
      error(0) = _odo(0) - (v_oo_x - e_r * om_oo_z);
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermDMI::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians) {
      // useful pre-computations
      const double om_oo_z = _om_oo.toValue()(0);
      const double e_r = _params->getValue()(1);

      // Jacobian with respect to odometry parameters
      Eigen::Matrix<double, 1, 11> Ht = Eigen::Matrix<double, 1, 11>::Zero();

      // Jacobian with respect to v_oo
      Eigen::Matrix<double, 1, 3> Hv = Eigen::Matrix<double, 1, 3>::Zero();

      // Jacobian with respect to om_oo
      Eigen::Matrix<double, 1, 3> Ho = Eigen::Matrix<double, 1, 3>::Zero();

      // v_rl measurement
      Ht(0, 1) = -om_oo_z;
      Hv(0, 0) = 1.0;
      Ho(0, 0) = -e_r;

      // pass the Jacobians with the chain rule
      _v_oo.evaluateJacobians(_jacobians, -Hv);
      _om_oo.evaluateJacobians(_jacobians, -Ho);
      _jacobians.add(_params, -Ht);
    }

  }
}
