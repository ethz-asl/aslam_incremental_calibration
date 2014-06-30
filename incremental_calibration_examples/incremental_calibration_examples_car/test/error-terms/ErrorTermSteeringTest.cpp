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

/** \file ErrorTermWheelTest.cpp
    \brief This file tests the ErrorTermWheel class.
  */

#include <Eigen/Core>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

#include "aslam/calibration/car/error-terms/ErrorTermSteering.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testErrorTermSteering) {

  // linear velocity
  const Eigen::Matrix<double, 3, 1> v(1.5, 1.6, 2.6);
  auto v_dv = boost::make_shared<EuclideanPoint>(v);
  EuclideanExpression v_exp(v_dv);

  // steering coefficients
  VectorDesignVariable<4> a((VectorDesignVariable<4>::Container() << 0.1, 1.5,
    0.2, 0.5).finished());

  // error term steering
  ErrorTermSteering e_st(v_exp, 1.5, 1, &a);

  // test the error terms
  try {
    ErrorTermTestHarness<1> harness(&e_st);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // test accessors
  e_st.setVariance(10);
  ASSERT_EQ(e_st.getVariance(), 10);
  e_st.setMeasurement(10);
  ASSERT_EQ(e_st.getMeasurement(), 10);
}
