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

#include "aslam/calibration/car/ErrorTermFws.h"

#include <Eigen/Dense>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermFws::ErrorTermFws(const aslam::backend::EuclideanExpression& v_oo,
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

    ErrorTermFws::ErrorTermFws(const ErrorTermFws& other) :
        ErrorTermFs<2>(other),
        _v_oo(other._v_oo),
        _om_oo(other._om_oo),
        _params(other._params),
        _odo(other._odo),
        _Q(other._Q) {
    }

    ErrorTermFws& ErrorTermFws::operator = (const ErrorTermFws& other) {
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

    ErrorTermFws::~ErrorTermFws() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const ErrorTermFws::Input& ErrorTermFws::getInput() const {
      return _odo;
    }

    ErrorTermFws::Input& ErrorTermFws::getInput() {
      return _odo;
    }

    void ErrorTermFws::setInput(const Input& odo) {
      _odo = odo;
    }

    const ErrorTermFws::Covariance& ErrorTermFws::getCovariance() const {
      return _Q;
    }

    ErrorTermFws::Covariance& ErrorTermFws::getCovariance() {
      return _Q;
    }

    void ErrorTermFws::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermFws::evaluateErrorImplementation() {
      // useful pre-computations
      const Eigen::Vector3d v_oo = _v_oo.toValue();
      const Eigen::Vector3d om_oo = _om_oo.toValue();
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double L = _params->getValue()(0);
      const double e_f = _params->getValue()(2);
      const double k_fl = _params->getValue()(9);
      const double k_fr = _params->getValue()(10);
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));

      // build the error term
      error_t error;
      error(0) = _odo(0) - (v_oo_x - e_f * om_oo_z) / cos(phi_L) / k_fl;
      error(1) = _odo(1) - (v_oo_x + e_f * om_oo_z) / cos(phi_R) / k_fr;
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermFws::evaluateJacobiansImplementation() {
      // useful pre-computations
      const Eigen::Vector3d v_oo = _v_oo.toValue();
      const Eigen::Vector3d om_oo = _om_oo.toValue();
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double L = _params->getValue()(0);
      const double e_f = _params->getValue()(2);
      const double k_fl = _params->getValue()(9);
      const double k_fr = _params->getValue()(10);
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));

      // Jacobian with respect to odometry parameters
      Eigen::Matrix<double, 2, 11> Ht = Eigen::Matrix<double, 2, 11>::Zero();

      // Jacobian with respect to v_oo
      Eigen::Matrix<double, 2, 3> Hv = Eigen::Matrix<double, 2, 3>::Zero();

      // Jacobian with respect to om_oo
      Eigen::Matrix<double, 2, 3> Ho = Eigen::Matrix<double, 2, 3>::Zero();

      // v_fl measurement
      const double vflDenom = k_fl * cos(phi_L) *
        ((v_oo_x - e_f * om_oo_z) * (v_oo_x - e_f * om_oo_z) +
        (L * om_oo_z) * (L * om_oo_z));
      Ht(0, 0) = L * om_oo_z * om_oo_z * (v_oo_x - e_f * om_oo_z) / vflDenom;
      Ht(0, 2) = -om_oo_z * (v_oo_x - e_f * om_oo_z) *
        (v_oo_x - e_f * om_oo_z) / vflDenom;
      Ht(0, 9) = -(v_oo_x - e_f * om_oo_z) / k_fl / k_fl / cos(phi_L);
      Hv(0, 0) = (v_oo_x - e_f * om_oo_z) * (v_oo_x - e_f * om_oo_z) / vflDenom;
      Ho(0, 2) = (L * L * v_oo_x * om_oo_z - e_f * ((v_oo_x - e_f * om_oo_z) *
        (v_oo_x - e_f * om_oo_z) + (L * om_oo_z) * (L * om_oo_z))) / vflDenom;

      // v_fr measurement
      const double vfrDenom = k_fl * cos(phi_R) *
        ((v_oo_x + e_f * om_oo_z) * (v_oo_x + e_f * om_oo_z) +
        (L * om_oo_z) * (L * om_oo_z));
      Ht(1, 0) = L * om_oo_z * om_oo_z * (v_oo_x + e_f * om_oo_z) / vfrDenom;
      Ht(1, 2) = om_oo_z * (v_oo_x + e_f * om_oo_z) * (v_oo_x + e_f * om_oo_z) /
        vfrDenom;
      Ht(1, 10) = -(v_oo_x + e_f * om_oo_z) / k_fr / k_fr / cos(phi_R);
      Hv(1, 0) = (v_oo_x + e_f * om_oo_z) * (v_oo_x + e_f * om_oo_z) / vfrDenom;
      Ho(1, 2) = (L * L * v_oo_x * om_oo_z + e_f * ((v_oo_x + e_f * om_oo_z) *
        (v_oo_x + e_f * om_oo_z) + (L * om_oo_z) * (L * om_oo_z))) / vfrDenom;

      // pass the Jacobians with the chain rule
      _v_oo.evaluateJacobians(_jacobians, -Hv);
      _om_oo.evaluateJacobians(_jacobians, -Ho);
      _jacobians.add(_params, -Ht);
    }

  }
}
