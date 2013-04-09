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

/** \file IncrementalOptimizationProblemTest.cpp
    \brief This file tests the IncrementalOptimizationProblem class.
  */

#include <boost/shared_ptr.hpp>

#include <gtest/gtest.h>

#include "aslam/calibration/core/IncrementalOptimizationProblem.h"
#include "aslam/calibration/core/OptimizationProblem.h"
#include "aslam/calibration/data-structures/VectorDesignVariable.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testIncrementalOptimizationProblem) {
  boost::shared_ptr<OptimizationProblem> problem1(new OptimizationProblem);
  OptimizationProblem::DesignVariableSP dv1(new VectorDesignVariable<2>());
  OptimizationProblem::DesignVariableSP dv2(new VectorDesignVariable<3>());
  OptimizationProblem::DesignVariableSP dv3(new VectorDesignVariable<4>());
  problem1->addDesignVariable(dv1, 0);
  problem1->addDesignVariable(dv2, 0);
  problem1->addDesignVariable(dv3, 1);
  boost::shared_ptr<OptimizationProblem> problem2(new OptimizationProblem);
  OptimizationProblem::DesignVariableSP dv4(new VectorDesignVariable<2>());
  OptimizationProblem::DesignVariableSP dv5(new VectorDesignVariable<3>());
  problem2->addDesignVariable(dv4, 0);
  problem2->addDesignVariable(dv5, 0);
  problem2->addDesignVariable(dv3, 1);
  boost::shared_ptr<OptimizationProblem> problem3(new OptimizationProblem);
  OptimizationProblem::DesignVariableSP dv6(new VectorDesignVariable<6>());
  OptimizationProblem::DesignVariableSP dv7(new VectorDesignVariable<6>());
  problem3->addDesignVariable(dv4, 0);
  problem3->addDesignVariable(dv6, 0);
  problem3->addDesignVariable(dv3, 1);
  IncrementalOptimizationProblem incProblem;
  incProblem.add(problem1);
  incProblem.add(problem2);
  incProblem.add(problem3);
  ASSERT_EQ(incProblem.getNumOptimizationProblems(), 3);
  ASSERT_EQ(incProblem.getOptimizationProblem(0), problem1.get());
  ASSERT_EQ(incProblem.getOptimizationProblem(1), problem2.get());
  ASSERT_EQ(incProblem.getOptimizationProblem(2), problem3.get());
  ASSERT_THROW(incProblem.getOptimizationProblem(3),
    OutOfBoundException<size_t>);
  const IncrementalOptimizationProblem::OptimizationProblemsSP& problems =
    incProblem.getOptimizationProblems();
  ASSERT_EQ(problems.size(), 3);
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv1.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv2.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv3.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv4.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv5.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv6.get()));
  ASSERT_FALSE(incProblem.isDesignVariableInProblem(dv7.get()));
  const IncrementalOptimizationProblem::DesignVariablePGroups& groups =
    incProblem.getDesignVariablesGroups();
  ASSERT_EQ(groups.size(), 2);
  const IncrementalOptimizationProblem::DesignVariablesP& dvs0 =
    incProblem.getDesignVariablesGroup(0);
  ASSERT_EQ(dvs0.size(), 5);
  const IncrementalOptimizationProblem::DesignVariablesP& dvs1 =
    incProblem.getDesignVariablesGroup(1);
  ASSERT_EQ(dvs1.size(), 1);
  ASSERT_THROW(incProblem.getDesignVariablesGroup(2),
    OutOfBoundException<size_t>);
  const IncrementalOptimizationProblem::ErrorTermsP& et =
    incProblem.getErrorTerms();
  ASSERT_EQ(et.size(), 0);
  ASSERT_EQ(incProblem.getNumGroups(), 2);
  ASSERT_EQ(incProblem.getGroupId(dv1.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv2.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv3.get()), 1);
  ASSERT_EQ(incProblem.getGroupId(dv4.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv5.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv6.get()), 0);
  ASSERT_THROW(incProblem.getGroupId(dv7.get()), InvalidOperationException);
  ASSERT_EQ(incProblem.getGroupDim(0), 16);
  ASSERT_EQ(incProblem.getGroupDim(1), 4);
  ASSERT_THROW(incProblem.getGroupDim(2), OutOfBoundException<size_t>);
  ASSERT_TRUE(incProblem.isGroupInProblem(0));
  ASSERT_TRUE(incProblem.isGroupInProblem(1));
  ASSERT_FALSE(incProblem.isGroupInProblem(2));
  ASSERT_EQ(incProblem.getGroupsOrdering(), std::vector<size_t>({0, 1}));
  ASSERT_EQ(incProblem.numDesignVariables(), 6);
  ASSERT_EQ(incProblem.designVariable(0), dv1.get());
  ASSERT_EQ(incProblem.designVariable(1), dv2.get());
  ASSERT_EQ(incProblem.designVariable(2), dv4.get());
  ASSERT_EQ(incProblem.designVariable(3), dv5.get());
  ASSERT_EQ(incProblem.designVariable(4), dv6.get());
  ASSERT_EQ(incProblem.designVariable(5), dv3.get());
  ASSERT_THROW(incProblem.designVariable(7), OutOfBoundException<size_t>);
  incProblem.setGroupsOrdering({1, 0});
  ASSERT_EQ(incProblem.getGroupsOrdering(), std::vector<size_t>({1, 0}));
  ASSERT_EQ(incProblem.designVariable(0), dv3.get());
  ASSERT_THROW(incProblem.setGroupsOrdering({1, 0, 2}),
    OutOfBoundException<size_t>);
  ASSERT_THROW(incProblem.setGroupsOrdering({1, 2}),
    OutOfBoundException<size_t>);
  ASSERT_THROW(incProblem.setGroupsOrdering({0, 0}),
    OutOfBoundException<size_t>);
  ASSERT_EQ(incProblem.numErrorTerms(), 0);
  ASSERT_THROW(incProblem.errorTerm(2), OutOfBoundException<size_t>);
  incProblem.permuteDesignVariables({1, 0, 2, 3, 4}, 0);
  ASSERT_EQ(incProblem.designVariable(1), dv2.get());
  incProblem.remove(2);
  ASSERT_EQ(incProblem.getNumOptimizationProblems(), 2);
  ASSERT_FALSE(incProblem.isDesignVariableInProblem(dv6.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv3.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv4.get()));
  const IncrementalOptimizationProblem::DesignVariablesP& dvs0update =
    incProblem.getDesignVariablesGroup(0);
  ASSERT_EQ(dvs0update.size(), 4);
  ASSERT_THROW(incProblem.remove(4), OutOfBoundException<size_t>);
}
