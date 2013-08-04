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

#include "aslam/calibration/2dlrf/ErrorTermMotion.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermMotion::ErrorTermMotion(VectorDesignVariable<3>* xkm1,
        VectorDesignVariable<3>* xk, double T, const Input& uk,
        const Covariance& Q) :
        _xkm1(xkm1),
        _xk(xk),
        _T(T),
        _uk(uk),
        _Q(Q) {
      setInvR(_Q.inverse());
      setDesignVariables(xkm1, xk);
    }

    ErrorTermMotion::ErrorTermMotion(const ErrorTermMotion& other) :
        ErrorTermFs<3>(other),
        _xkm1(other._xkm1),
        _xk(other._xk),
        _T(other._T),
        _uk(other._uk),
        _Q(other._Q) {
    }

    ErrorTermMotion& ErrorTermMotion::operator =
        (const ErrorTermMotion& other) {
      if (this != &other) {
        ErrorTermFs<3>::operator=(other);
       _xkm1 = other._xkm1;
       _xk = other._xk;
       _T = other._T;
       _uk = other._uk;
       _Q = other._Q;
      }
      return *this;
    }

    ErrorTermMotion::~ErrorTermMotion() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double ErrorTermMotion::getTimestep() const {
      return _T;
    }

    void ErrorTermMotion::setTimestep(double T) {
      _T = T;
    }

    const ErrorTermMotion::Input& ErrorTermMotion::getInput() const {
      return _uk;
    }

    ErrorTermMotion::Input& ErrorTermMotion::getInput() {
      return _uk;
    }

    void ErrorTermMotion::setInput(const Input& uk) {
      _uk = uk;
    }

    const ErrorTermMotion::Covariance& ErrorTermMotion::getCovariance() const {
      return _Q;
    }

    ErrorTermMotion::Covariance& ErrorTermMotion::getCovariance() {
      return _Q;
    }

    void ErrorTermMotion::setCovariance(const Covariance& Q) {
      _Q = Q;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermMotion::evaluateErrorImplementation() {
      Eigen::Matrix<double, 3, 3> B = Eigen::Matrix<double, 3, 3>::Identity();
      B(0, 0) = cos((_xkm1->getValue())(2));
      B(0, 1) = sin((_xkm1->getValue())(2));
      B(1, 0) = -sin((_xkm1->getValue())(2));
      B(1, 1) = cos((_xkm1->getValue())(2));
      error_t error = _uk -
        (1 / _T * B * (_xk->getValue() - _xkm1->getValue()));
      error(2) = sm::kinematics::angleMod(error(2));
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermMotion::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {
      Eigen::Matrix<double, 3, 3> Hxk = Eigen::Matrix<double, 3, 3>::Zero();
      Hxk(0, 0) = cos((_xkm1->getValue())(2));
      Hxk(0, 1) = sin((_xkm1->getValue())(2));
      Hxk(1, 0) = -sin((_xkm1->getValue())(2));
      Hxk(1, 1) = cos((_xkm1->getValue())(2));
      Hxk(2, 2) = 1;
      Eigen::Matrix<double, 3, 3> Hxkm1 =
        Eigen::Matrix<double, 3, 3>::Zero();
      Hxkm1(0, 0) = -cos((_xkm1->getValue())(2));
      Hxkm1(0, 1) = -sin((_xkm1->getValue())(2));
      Hxkm1(0, 2) = -sin((_xkm1->getValue())(2)) *
        ((_xk->getValue())(0) - (_xkm1->getValue())(0)) +
        cos((_xkm1->getValue())(2)) *
        ((_xk->getValue())(1) - (_xkm1->getValue())(1));
      Hxkm1(1, 0) = sin((_xkm1->getValue())(2));
      Hxkm1(1, 1) = -cos((_xkm1->getValue())(2));
      Hxkm1(1, 2) = -cos((_xkm1->getValue())(2)) *
        ((_xk->getValue())(0) - (_xkm1->getValue())(0)) -
        sin((_xkm1->getValue())(2)) *
        ((_xk->getValue())(1) - (_xkm1->getValue())(1));
      Hxkm1(2, 2) = -1;
      jacobians.add(_xkm1, -Hxkm1 / _T);
      jacobians.add(_xk, -Hxk / _T);
    }

  }
}
