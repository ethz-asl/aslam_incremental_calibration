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

/** \file AlgorithmsTest.cpp
    \brief This file tests the algorithms.
  */

#include <vector>

#include <gtest/gtest.h>

#include "aslam/calibration/algorithms/permute.h"
#include "aslam/calibration/statistics/UniformDistribution.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testAlgorithms) {
  UniformDistribution<double> dist(0, 100);
  const std::vector<double> input = {dist.getSample(), dist.getSample(),
    dist.getSample(), dist.getSample(), dist.getSample(), dist.getSample()};
  const std::vector<size_t> p = {1, 3, 0, 2, 5, 4};
  auto pinput = input;
  permute(pinput, p);
  ASSERT_EQ(pinput, std::vector<double>({input[p[0]], input[p[1]], input[p[2]],
    input[p[3]], input[p[4]], input[p[5]]}));
  ASSERT_THROW(permute(pinput, {1, 3, 0}), OutOfBoundException<size_t>);
  ASSERT_THROW(permute(pinput, {10, 3, 0, 2, 5, 4}),
    OutOfBoundException<size_t>);
}
