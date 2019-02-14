/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file VectorDesignVariableTest.cpp
    \brief This file tests the VectorDesignVariable class.
  */

#include <gtest/gtest.h>

#include "aslam/calibration/data-structures/VectorDesignVariable.h"

TEST(AslamCalibrationTestSuite, testVectorDesignVariable) {
  // Default constructor using static size
  aslam::calibration::VectorDesignVariable<3> dv1;
  ASSERT_EQ(dv1.getValue(),
    aslam::calibration::VectorDesignVariable<3>::Container::Zero());
  dv1.setValue(aslam::calibration::VectorDesignVariable<3>::Container::Ones());
  ASSERT_EQ(dv1.getValue(),
    aslam::calibration::VectorDesignVariable<3>::Container::Ones());

  // Constructor with initial value using static size
  aslam::calibration::VectorDesignVariable<3> dv2(
    aslam::calibration::VectorDesignVariable<3>::Container::Ones());

  // Constructor with initial value using dynamic size
  aslam::calibration::VectorDesignVariable<Eigen::Dynamic> dv3(
    aslam::calibration::VectorDesignVariable<Eigen::Dynamic>::
    Container::Random(5, 1));
  dv3.setValue(aslam::calibration::VectorDesignVariable<Eigen::Dynamic>::
    Container::Ones(5, 1));
  ASSERT_EQ(dv3.getValue(),
    aslam::calibration::VectorDesignVariable<Eigen::Dynamic>::
    Container::Ones(5, 1));

  // Copy constructor
  aslam::calibration::VectorDesignVariable<3> dv4(dv1);
  ASSERT_EQ(dv1.getValue(), dv4.getValue());

  // Assignment constructor
  aslam::calibration::VectorDesignVariable<3> dv5 = dv1;
  ASSERT_EQ(dv1.getValue(), dv5.getValue());

  // Setter/getter
  dv1.setParameters(Eigen::Vector3d::Ones());
  Eigen::MatrixXd dv1Param;
  dv1.getParameters(dv1Param);
  ASSERT_EQ(Eigen::Vector3d::Ones(), dv1Param);
  ASSERT_THROW(dv1.setParameters(Eigen::Vector2d::Ones()),
    aslam::calibration::OutOfBoundException<int>);
}
