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

#include "aslam/calibration/2dlrf/ErrorTermObservation.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermObservation::ErrorTermObservation(VectorDesignVariable<3>* xk,
        VectorDesignVariable<2>* xl, VectorDesignVariable<3>* Theta, double r,
        double b, const Covariance& R) :
        _xk(xk),
        _xl(xl),
        _Theta(Theta),
        _r(r),
        _b(b),
        _R(R) {
      setInvR(_R.inverse());
      setDesignVariables(xk, xl, Theta);
    }

    ErrorTermObservation::ErrorTermObservation(
        const ErrorTermObservation& other) :
        ErrorTermFs<2>(other),
        _xk(other._xk),
        _xl(other._xl),
        _Theta(other._Theta),
        _r(other._r),
        _b(other._b),
        _R(other._R) {
    }

    ErrorTermObservation& ErrorTermObservation::operator =
        (const ErrorTermObservation& other) {
      if (this != &other) {
        ErrorTermFs<2>::operator=(other);
       _xk = other._xk;
       _xl = other._xl;
       _Theta = other._Theta;
       _r = other._r;
       _b = other._b;
       _R = other._R;
      }
      return *this;
    }

    ErrorTermObservation::~ErrorTermObservation() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double ErrorTermObservation::getRange() const {
      return _r;
    }

    void ErrorTermObservation::setRange(double r) {
      _r = r;
    }

    double ErrorTermObservation::getBearing() const {
      return _b;
    }

    void ErrorTermObservation::setBearing(double b) {
      _b = b;
    }

    const ErrorTermObservation::Covariance&
        ErrorTermObservation::getCovariance() const {
      return _R;
    }

    ErrorTermObservation::Covariance& ErrorTermObservation::getCovariance() {
      return _R;
    }

    void ErrorTermObservation::setCovariance(const Covariance& R) {
      _R = R;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermObservation::evaluateErrorImplementation() {
      const double ct = cos((_xk->getValue())(2));
      const double st = sin((_xk->getValue())(2));
      const double dxct = (_Theta->getValue())(0) * ct;
      const double dxst = (_Theta->getValue())(0) * st;
      const double dyct = (_Theta->getValue())(1) * ct;
      const double dyst = (_Theta->getValue())(1) * st;
      const double aa = (_xl->getValue())(0) - (_xk->getValue())(0) - dxct
        + dyst;
      const double bb = (_xl->getValue())(1) - (_xk->getValue())(1) - dxst
        - dyct;
      const double temp1 = aa * aa + bb * bb;
      const double temp2 = sqrt(temp1);
      error_t error;
      error(0) = _r - temp2;
      error(1) = sm::kinematics::angleMod(_b - (atan2(bb, aa) -
        (_xk->getValue())(2) - (_Theta->getValue())(2)));
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermObservation::evaluateJacobiansImplementation() {
      const double ct = cos((_xk->getValue())(2));
      const double st = sin((_xk->getValue())(2));
      const double dxct = (_Theta->getValue())(0) * ct;
      const double dxst = (_Theta->getValue())(0) * st;
      const double dyct = (_Theta->getValue())(1) * ct;
      const double dyst = (_Theta->getValue())(1) * st;
      const double aa = (_xl->getValue())(0) - (_xk->getValue())(0) - dxct
        + dyst;
      const double bb = (_xl->getValue())(1) - (_xk->getValue())(1) - dxst
        - dyct;
      const double temp1 = aa * aa + bb * bb;
      const double temp2 = sqrt(temp1);
      Eigen::Matrix<double, 2, 3> Gxk = Eigen::Matrix<double, 2, 3>::Zero();
      Gxk(0, 0) = -aa / temp2;
      Gxk(0, 1) = -bb / temp2;
      Gxk(0, 2) = (aa * (dxst + dyct) + bb * (-dxct + dyst)) / temp2;
      Gxk(1, 0) = bb / temp1;
      Gxk(1, 1) = -aa / temp1;
      Gxk(1, 2) = (aa * (-dxct + dyst) - bb * (dxst + dyct)) / temp1 - 1;
      Eigen::Matrix<double, 2, 2> Glk = Eigen::Matrix<double, 2, 2>::Zero();
      Glk(0, 0) = aa / temp2;
      Glk(0, 1) = bb / temp2;
      Glk(1, 0) = -bb / temp1;
      Glk(1, 1) = aa / temp1;
      Eigen::Matrix<double, 2, 3> Gtk = Eigen::Matrix<double, 2, 3>::Zero();
      Gtk(0, 0) = -(aa * ct + bb * st) / temp2;
      Gtk(0, 1) = (aa * st - bb * ct) / temp2;
      Gtk(1, 0) = (-aa * st + bb * ct) / temp1;
      Gtk(1, 1) = -(aa * ct + bb * st) / temp1;
      Gtk(1, 2) = -1;
      _jacobians.add(_xk, -Gxk);
      _jacobians.add(_xl, -Glk);
      _jacobians.add(_Theta, -Gtk);
    }

  }
}
