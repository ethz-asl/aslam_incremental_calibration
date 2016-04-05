#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "aslam/calibration/algorithms/permute.h"
#include "aslam/calibration/algorithms/matrixOperations.h"
#include "aslam/calibration/statistics/UniformDistribution.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"

TEST(AslamCalibrationTestSuite, testPermute) {
  using namespace aslam::calibration;

  UniformDistribution<double> dist(0, 100);
    const std::vector<double> input = {dist.getSample(), dist.getSample(),
      dist.getSample(), dist.getSample(), dist.getSample(), dist.getSample()};
  const std::vector<size_t> p = {1, 3, 0, 2, 5, 4};
  auto pinput = input;

  permute(pinput, p);

  ASSERT_EQ(pinput, std::vector<double>({input[p[0]], input[p[1]], input[p[2]],
    input[p[3]], input[p[4]], input[p[5]]}));
  ASSERT_THROW(permute(pinput, std::vector<size_t>({1, 3, 0})),
    OutOfBoundException<size_t>);
  ASSERT_THROW(permute(pinput, std::vector<size_t>({10, 3, 0, 2, 5, 4})),
    OutOfBoundException<size_t>);
}

TEST(AslamCalibrationTestSuite, testComputeSumLogDiagR) {
  using namespace aslam::calibration;

  Eigen::Matrix3d A;
  A << 1,0,0, 10,50,0, 0,20,3.3;

  EXPECT_NEAR(computeSumLogDiagR(A,0,2), std::log2(std::fabs(A(0,0))) +
              std::log2(std::fabs(A(1,1))) + std::log2(std::fabs(A(2,2))), 1e-8);
  EXPECT_NEAR(computeSumLogDiagR(A,0,1), std::log2(std::fabs(A(0,0))) +
              std::log2(std::fabs(A(1,1))), 1e-8);
  EXPECT_NEAR(computeSumLogDiagR(A,1,2), std::log2(std::fabs(A(1,1))) +
              std::log2(std::fabs(A(2,2))), 1e-8);
}
