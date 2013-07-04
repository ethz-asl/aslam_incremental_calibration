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

/** \file ErrorTermOdometryTest.cpp
    \brief This file tests the ErrorTermOdometry class.
  */

#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>

#include "aslam/calibration/car/ErrorTermOdometry.h"
#include "aslam/calibration/car/ErrorTermFws.h"
#include "aslam/calibration/car/ErrorTermRws.h"
#include "aslam/calibration/car/ErrorTermSteering.h"
#include "aslam/calibration/car/ErrorTermDMI.h"

TEST(AslamCalibrationTestSuite, testErrorTermOdometry) {

  // calibration parameters
  const double L = 1.81;
  const double e_r = 0.6575;
  const double e_f = 0.6425;
  const double a0 = 0.0;
  const double a1 = 1.0;
  const double a2 = 0.0;
  const double a3 = 0.0;
  const double k_rl = 1.0;
  const double k_rr = 1.0;
  const double k_fl = 1.0;
  const double k_fr = 1.0;
  aslam::calibration::VectorDesignVariable<11> Theta(
    (aslam::calibration::VectorDesignVariable<11>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr).finished());

  // linear velocity
  const Eigen::Matrix<double, 3, 1> v(1.5, 0, 0);
  auto v_en = boost::make_shared<aslam::backend::EuclideanPoint>(v);
  aslam::backend::EuclideanExpression v_e(v_en);

  // angular velocity
  const Eigen::Matrix<double, 3, 1> om(0, 0, M_PI / 8.0);
  auto om_en = boost::make_shared<aslam::backend::EuclideanPoint>(om);
  aslam::backend::EuclideanExpression om_e(om_en);

  // odometry sensor measurement without noise
  const double v_x = v(0);
  const double om_z = om(2);
  const double phi_L = atan(L * om_z / (v_x - e_f * om_z));
  const double phi_R = atan(L * om_z / (v_x + e_f * om_z));
  const double phi = atan(L * om_z / v_x);
  aslam::calibration::ErrorTermOdometry::Input odo;
  odo(0) = a0 + a1 * phi + a2 * phi * phi + a3 * phi * phi * phi;
  odo(1) = (v_x - e_r * om_z) / k_rl;
  odo(2) = (v_x + e_r * om_z) / k_rr;
  odo(3) = (v_x - e_f * om_z) / cos(phi_L) / k_fl;
  odo(4) = (v_x + e_f * om_z) / cos(phi_R) / k_fr;

  // covariance matrix of the measurement
  aslam::calibration::ErrorTermOdometry::Covariance Q =
    aslam::calibration::ErrorTermOdometry::Covariance::Zero();
  Q(0, 0) = 1e-4;
  Q(1, 1) = 1e-4;
  Q(2, 2) = 1e-4;
  Q(3, 3) = 4e-4;
  Q(4, 4) = 4e-4;

  // odometry sensor measurement with noise
  aslam::calibration::ErrorTermOdometry::Input odo_n(
    aslam::calibration::NormalDistribution<5>(odo, Q).getSample());

  // build the error term
  aslam::calibration::ErrorTermOdometry e1(v_e, om_e, &Theta, odo_n, Q);

  // test the error term
  try {
    aslam::backend::ErrorTermTestHarness<5> harness(&e1);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // accessors test
  e1.setInput(odo_n);
  e1.setCovariance(Q);
  ASSERT_EQ(e1.getInput(), odo_n);
  ASSERT_EQ(e1.getCovariance(), Q);

  // separate odometry error terms
  try {
    Eigen::Matrix2d R_fws;
    R_fws << 1e-4, 0, 0, 1e-4;
    Eigen::Vector2d meas;
    meas << odo_n(3), odo_n(4);
    aslam::calibration::ErrorTermFws eFws(v_e, om_e, &Theta, meas, R_fws);
    aslam::backend::ErrorTermTestHarness<2> harness(&eFws);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
  try {
    Eigen::Matrix2d R_rws;
    R_rws << 1e-4, 0, 0, 1e-4;
    Eigen::Vector2d meas;
    meas << odo_n(1), odo_n(2);
    aslam::calibration::ErrorTermRws eRws(v_e, om_e, &Theta, meas, R_rws);
    aslam::backend::ErrorTermTestHarness<2> harness(&eRws);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
  try {
    Eigen::Matrix<double, 1, 1> R_st;
    R_st << 1e-4;
    Eigen::Matrix<double, 1, 1> meas;
    meas << odo_n(0);
    aslam::calibration::ErrorTermSteering eSt(v_e, om_e, &Theta, meas, R_st);
    aslam::backend::ErrorTermTestHarness<1> harness(&eSt);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
  try {
    Eigen::Matrix<double, 1, 1> R_dmi;
    R_dmi << 1e-4;
    Eigen::Matrix<double, 1, 1> meas;
    meas << odo_n(1);
    aslam::calibration::VectorDesignVariable<1> ThetaDmi(
      (aslam::calibration::VectorDesignVariable<1>::Container() <<
      e_r).finished());
    aslam::calibration::ErrorTermDMI eDmi(v_e, om_e, &ThetaDmi, meas, R_dmi);
    aslam::backend::ErrorTermTestHarness<1> harness(&eDmi);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
}