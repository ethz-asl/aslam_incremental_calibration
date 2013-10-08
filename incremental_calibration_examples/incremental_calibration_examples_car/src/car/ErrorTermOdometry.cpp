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

#include "aslam/calibration/car/ErrorTermOdometry.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermOdometry::ErrorTermOdometry(
        const aslam::backend::EuclideanExpression& v_oo,
        const aslam::backend::EuclideanExpression& om_oo,
        VectorDesignVariable<12>* params,
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

    ErrorTermOdometry::ErrorTermOdometry(const ErrorTermOdometry& other) :
        ErrorTermFs<5>(other),
        _v_oo(other._v_oo),
        _om_oo(other._om_oo),
        _params(other._params),
        _odo(other._odo),
        _Q(other._Q) {
    }

    ErrorTermOdometry& ErrorTermOdometry::operator =
        (const ErrorTermOdometry& other) {
      if (this != &other) {
        ErrorTermFs<5>::operator=(other);
       _v_oo = other._v_oo;
       _om_oo = other._om_oo;
       _params = other._params;
       _odo = other._odo;
       _Q = other._Q;
      }
      return *this;
    }

    ErrorTermOdometry::~ErrorTermOdometry() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const ErrorTermOdometry::Input& ErrorTermOdometry::getInput() const {
      return _odo;
    }

    ErrorTermOdometry::Input& ErrorTermOdometry::getInput() {
      return _odo;
    }

    void ErrorTermOdometry::setInput(const Input& odo) {
      _odo = odo;
    }

    const ErrorTermOdometry::Covariance& ErrorTermOdometry::getCovariance()
        const {
      return _Q;
    }

    ErrorTermOdometry::Covariance& ErrorTermOdometry::getCovariance() {
      return _Q;
    }

    void ErrorTermOdometry::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermOdometry::evaluateErrorImplementation() {
      // useful pre-computations
      const Eigen::Vector3d v_oo = _v_oo.toValue();
      const Eigen::Vector3d om_oo = _om_oo.toValue();
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double L = _params->getValue()(0);
      const double e_r = _params->getValue()(1);
      const double e_f = _params->getValue()(2);
      const double a0 = _params->getValue()(3);
      const double a1 = _params->getValue()(4);
      const double a2 = _params->getValue()(5);
      const double a3 = _params->getValue()(6);
      const double k_rl = _params->getValue()(7);
      const double k_rr = _params->getValue()(8);
      const double k_fl = _params->getValue()(9);
      const double k_fr = _params->getValue()(10);
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
      const double phi = atan(L * om_oo_z / v_oo_x);

      // build the error term
      error_t error;
      error(0) = _odo(0) - (a0 + a1 * phi + a2 * phi * phi +
        a3 * phi * phi * phi);
      error(0) = sm::kinematics::angleMod(error(0));
      error(1) = _odo(1) - (v_oo_x - e_r * om_oo_z) / k_rl;
      error(2) = _odo(2) - (v_oo_x + e_r * om_oo_z) / k_rr;
      error(3) = _odo(3) - (v_oo_x - e_f * om_oo_z) / cos(phi_L) / k_fl;
      error(4) = _odo(4) - (v_oo_x + e_f * om_oo_z) / cos(phi_R) / k_fr;
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermOdometry::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {
      // useful pre-computations
      const Eigen::Vector3d v_oo = _v_oo.toValue();
      const Eigen::Vector3d om_oo = _om_oo.toValue();
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double L = _params->getValue()(0);
      const double e_r = _params->getValue()(1);
      const double e_f = _params->getValue()(2);
      const double a1 = _params->getValue()(4);
      const double a2 = _params->getValue()(5);
      const double a3 = _params->getValue()(6);
      const double k_rl = _params->getValue()(7);
      const double k_rr = _params->getValue()(8);
      const double k_fl = _params->getValue()(9);
      const double k_fr = _params->getValue()(10);
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
      const double phi = atan(L * om_oo_z / v_oo_x);

      // Jacobian with respect to odometry parameters
      Eigen::Matrix<double, 5, 12> Ht = Eigen::Matrix<double, 5, 12>::Zero();

      // Jacobian with respect to v_oo
      Eigen::Matrix<double, 5, 3> Hv = Eigen::Matrix<double, 5, 3>::Zero();

      // Jacobian with respect to om_oo
      Eigen::Matrix<double, 5, 3> Ho = Eigen::Matrix<double, 5, 3>::Zero();

      // steering measurement
      const double sDenom = v_oo_x * v_oo_x + (L * om_oo_z) * (L * om_oo_z);
      Ht(0, 0) = om_oo_z * v_oo_x * (a1 + 2 * a2 * phi + 3 * a3 * phi * phi) /
        sDenom;
      Ht(0, 3) = 1;
      Ht(0, 4) = phi;
      Ht(0, 5) = phi * phi;
      Ht(0, 6) = phi * phi * phi;
      Hv(0, 0) = -L * om_oo_z * (a1 + 2 * a2 * phi + 3 * a3 * phi * phi) /
        sDenom;
      Ho(0, 2) = L * v_oo_x * (a1 + 2 * a2 * phi + 3 * a3 * phi * phi) / sDenom;

      // v_rl measurement
      Ht(1, 1) = -om_oo_z / k_rl;
      Ht(1, 7) = -(v_oo_x - e_r * om_oo_z) / k_rl / k_rl;
      Hv(1, 0) = 1.0 / k_rl;
      Ho(1, 2) = -e_r / k_rl;

      // v_rr measurement
      Ht(2, 1) = om_oo_z / k_rr;
      Ht(2, 8) = -(v_oo_x + e_r * om_oo_z) / k_rr / k_rr;
      Hv(2, 0) = 1.0 / k_rr;
      Ho(2, 2) = e_r / k_rr;

      // v_fl measurement
      const double vflDenom = k_fl * cos(phi_L) *
        ((v_oo_x - e_f * om_oo_z) * (v_oo_x - e_f * om_oo_z) +
        (L * om_oo_z) * (L * om_oo_z));
      Ht(3, 0) = L * om_oo_z * om_oo_z * (v_oo_x - e_f * om_oo_z) / vflDenom;
      Ht(3, 2) = -om_oo_z * (v_oo_x - e_f * om_oo_z) *
        (v_oo_x - e_f * om_oo_z) / vflDenom;
      Ht(3, 9) = -(v_oo_x - e_f * om_oo_z) / k_fl / k_fl / cos(phi_L);
      Hv(3, 0) = (v_oo_x - e_f * om_oo_z) * (v_oo_x - e_f * om_oo_z) / vflDenom;
      Ho(3, 2) = (L * L * v_oo_x * om_oo_z - e_f * ((v_oo_x - e_f * om_oo_z) *
        (v_oo_x - e_f * om_oo_z) + (L * om_oo_z) * (L * om_oo_z))) / vflDenom;

      // v_fr measurement
      const double vfrDenom = k_fl * cos(phi_R) *
        ((v_oo_x + e_f * om_oo_z) * (v_oo_x + e_f * om_oo_z) +
        (L * om_oo_z) * (L * om_oo_z));
      Ht(4, 0) = L * om_oo_z * om_oo_z * (v_oo_x + e_f * om_oo_z) / vfrDenom;
      Ht(4, 2) = om_oo_z * (v_oo_x + e_f * om_oo_z) * (v_oo_x + e_f * om_oo_z) /
        vfrDenom;
      Ht(4, 10) = -(v_oo_x + e_f * om_oo_z) / k_fr / k_fr / cos(phi_R);
      Hv(4, 0) = (v_oo_x + e_f * om_oo_z) * (v_oo_x + e_f * om_oo_z) / vfrDenom;
      Ho(4, 2) = (L * L * v_oo_x * om_oo_z + e_f * ((v_oo_x + e_f * om_oo_z) *
        (v_oo_x + e_f * om_oo_z) + (L * om_oo_z) * (L * om_oo_z))) / vfrDenom;

      // pass the Jacobians with the chain rule
      _v_oo.evaluateJacobians(jacobians, -Hv);
      _om_oo.evaluateJacobians(jacobians, -Ho);
      jacobians.add(_params, -Ht);
    }

  }
}
