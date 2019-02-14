/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/statistics/EstimatorML.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    EstimatorML<NormalDistribution<1> >::EstimatorML() :
        mNumPoints(0),
        mValid(false),
        mValuesSum(0),
        mSquaredValuesSum(0) {
    }

    EstimatorML<NormalDistribution<1> >::EstimatorML(const EstimatorML& other) :
        mDistribution(other.mDistribution),
        mNumPoints(other.mNumPoints),
        mValid(other.mValid),
        mValuesSum(other.mValuesSum),
        mSquaredValuesSum(other.mSquaredValuesSum) {
    }

    EstimatorML<NormalDistribution<1> >&
        EstimatorML<NormalDistribution<1> >::operator =
        (const EstimatorML& other) {
      if (this != &other) {
        mDistribution = other.mDistribution;
        mNumPoints = other.mNumPoints;
        mValid = other.mValid;
        mValuesSum = other.mValuesSum;
        mSquaredValuesSum = other.mSquaredValuesSum;
      }
      return *this;
    }

    EstimatorML<NormalDistribution<1> >::~EstimatorML() {
    }

/******************************************************************************/
/* Streaming operations                                                       */
/******************************************************************************/

    void EstimatorML<NormalDistribution<1> >::read(std::istream& stream) {
    }

    void EstimatorML<NormalDistribution<1> >::write(std::ostream& stream)
        const {
      stream << "distribution: " << std::endl << mDistribution << std::endl
        << "number of points: " << mNumPoints << std::endl
        << "valid: " << mValid;
    }

    void EstimatorML<NormalDistribution<1> >::read(std::ifstream& stream) {
    }

    void EstimatorML<NormalDistribution<1> >::write(std::ofstream& stream)
        const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    size_t EstimatorML<NormalDistribution<1> >::getNumPoints() const {
      return mNumPoints;
    }

    bool EstimatorML<NormalDistribution<1> >::getValid() const {
      return mValid;
    }

    const NormalDistribution<1>&
        EstimatorML<NormalDistribution<1> >::getDistribution() const {
      return mDistribution;
    }

    void EstimatorML<NormalDistribution<1> >::reset() {
      mNumPoints = 0;
      mValid = false;
      mValuesSum = 0;
      mSquaredValuesSum = 0;
    }

    void EstimatorML<NormalDistribution<1> >::addPoint(const Point& point) {
      mNumPoints++;
      mValuesSum += point;
      mSquaredValuesSum += point * point;
      try {
        mValid = true;
        const double mean = mValuesSum / mNumPoints;
        mDistribution.setMean(mean);
        mDistribution.setVariance(mSquaredValuesSum / mNumPoints -
          mValuesSum * mValuesSum * 2 / (mNumPoints * mNumPoints) +
          mean * mean);
      }
      catch (...) {
        mValid = false;
      }
    }

    void EstimatorML<NormalDistribution<1> >::addPoints(const
        ConstPointIterator& itStart, const ConstPointIterator& itEnd) {
      for (auto it = itStart; it != itEnd; ++it)
        addPoint(*it);
    }

    void EstimatorML<NormalDistribution<1> >::addPoints(const Container&
        points) {
      addPoints(points.begin(), points.end());
    }

    void EstimatorML<NormalDistribution<1> >::addPoints(const
        ConstPointIterator& itStart, const ConstPointIterator& itEnd, const
        Eigen::Matrix<double, Eigen::Dynamic, 1>& responsibilities) {
      if (responsibilities.size() != itEnd - itStart)
        return;
      double mean = 0;
      for (auto it = itStart; it != itEnd; ++it)
        mean += responsibilities(it - itStart) * (*it);
      double numPoints = responsibilities.sum();
      mean /= numPoints;
      double variance = 0;
      for (auto it = itStart; it != itEnd; ++it)
        variance += responsibilities(it - itStart) * (*it - mean) *
          (*it - mean);
      variance /= numPoints;
      try {
        mValid = true;
        mDistribution.setMean(mean);
        mDistribution.setVariance(variance);
      }
      catch (...) {
        mValid = false;
      }
    }

  }
}
