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

/** \file OptimizationProblemTest.cpp
    \brief This file tests the OptimizationProblem class.
  */

#include <gtest/gtest.h>

#include "aslam/calibration/core/OptimizationProblem.h"
#include "aslam/calibration/data-structures/VectorDesignVariable.h"

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testOptimizationProblem) {
  OptimizationProblem problem;
  OptimizationProblem::DesignVariableSP dv1(new VectorDesignVariable<2>());
  OptimizationProblem::DesignVariableSP dv2(new VectorDesignVariable<3>());
  OptimizationProblem::DesignVariableSP dv3(new VectorDesignVariable<4>());
  OptimizationProblem::DesignVariableSP dv4(new VectorDesignVariable<4>());
  problem.addDesignVariable(dv1, 0);
  problem.addDesignVariable(dv2, 1);
  problem.addDesignVariable(dv3, 1);
  ASSERT_TRUE(problem.isDesignVariableInProblem(dv1.get()));
  ASSERT_TRUE(problem.isDesignVariableInProblem(dv2.get()));
  ASSERT_TRUE(problem.isDesignVariableInProblem(dv3.get()));
  ASSERT_FALSE(problem.isDesignVariableInProblem(dv4.get()));
  try {
    problem.addDesignVariable(dv1);
  }
  catch (...) {
  }
  ASSERT_EQ(problem.getNumGroups(), 2);
  ASSERT_EQ(problem.getDesignVariablesGroup(1),
    OptimizationProblem::DesignVariablesSP({dv2, dv3}));
  ASSERT_EQ(problem.getDesignVariablesGroup(0),
    OptimizationProblem::DesignVariablesSP({dv1}));
  try {
    problem.getDesignVariablesGroup(2);
  }
  catch (...) {
  }
  problem.setGroupsOrdering(std::vector<size_t>({1, 0}));
  ASSERT_EQ(problem.getGroupsOrdering(), std::vector<size_t>({1, 0}));
  try {
    problem.setGroupsOrdering(std::vector<size_t>({2, 0}));
  }
  catch (...) {
  }
  try {
    problem.setGroupsOrdering(std::vector<size_t>({2, 0, 3}));
  }
  catch (...) {
  }
  try {
    problem.setGroupsOrdering(std::vector<size_t>({0, 0}));
  }
  catch (...) {
  }
  ASSERT_EQ(problem.getGroupId(dv1.get()), 0);
  ASSERT_EQ(problem.getGroupId(dv2.get()), 1);
  ASSERT_EQ(problem.getGroupId(dv3.get()), 1);
  try {
    problem.getGroupId(dv4.get());
  }
  catch (...) {
  }
  ASSERT_EQ(problem.numDesignVariables(), 3);
  ASSERT_EQ(problem.designVariable(0), dv2.get());
  ASSERT_EQ(problem.designVariable(1), dv3.get());
  ASSERT_EQ(problem.designVariable(2), dv1.get());
  try {
    problem.designVariable(3);
  }
  catch (...) {
  }
  ASSERT_EQ(problem.numErrorTerms(), 0);
  problem.permuteDesignVariables(std::vector<size_t>({1, 0}), 1);
  ASSERT_EQ(problem.designVariable(0), dv3.get());
  try {
    problem.permuteDesignVariables(std::vector<size_t>({1, 0}), 2);
  }
  catch (...) {
  }
  ASSERT_EQ(problem.getGroupDim(0), 2);
  ASSERT_EQ(problem.getGroupDim(1), 7);
  ASSERT_TRUE(problem.isGroupInProblem(0));
  ASSERT_TRUE(problem.isGroupInProblem(1));
  ASSERT_FALSE(problem.isGroupInProblem(2));
}
