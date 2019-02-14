/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include <limits>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename T>
    Histogram<T, 1>::Histogram(const T& min, const T& max, const T& binSize) :
        Grid<T, double, 1>((Coordinate() << min).finished(),
          (Coordinate() << max).finished(), (Coordinate()
          << binSize).finished()) {
    }

    template <typename T>
    Histogram<T, 1>::Histogram(const Histogram& other) :
      Grid<T, double, 1>(other) {
    }

    template <typename T>
    Histogram<T, 1>& Histogram<T, 1>::operator = (const Histogram& other) {
      if (this != &other) {
        Grid<T, double, 1>::operator=(other);
      }
      return *this;
    }

    template <typename T>
    Histogram<T, 1>::~Histogram() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename T>
    double Histogram<T, 1>::getMean() const {
      double mean = 0;
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i))
        mean += this->getCoordinates(i)(0) * this->getCell(i);
      return mean / getSum();
    }

    template <typename T>
    double Histogram<T, 1>::getMedian() const {
      const double sum = getSum();
      double currentSum = 0;
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i)) {
        currentSum += this->getCell(i);
        if (currentSum >= sum * 0.5)
          return this->getCoordinates(i)(0);
      }
      return this->getMinimum()(0);
    }

    template <typename T>
    double Histogram<T, 1>::getMode() const {
      double max = -std::numeric_limits<double>::infinity();
      Index modeIdx;
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i)) {
        if (this->getCell(i) > max) {
          max = this->getCell(i);
          modeIdx = i;
        }
      }
      return this->getCoordinates(modeIdx)(0);
    }

    template <typename T>
    double Histogram<T, 1>::getVariance() const {
      double variance = 0;
      const double mean = getMean();
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i))
        variance += (this->getCoordinates(i)(0) - mean) *
          (this->getCoordinates(i)(0) - mean) * this->getCell(i);
      return variance / (getSum() - 1);
    }

    template <typename T>
    double Histogram<T, 1>::getSum() const {
      double sum = 0;
      for (auto it = this->getCellBegin(); it != this->getCellEnd(); ++it)
        sum += *it;
      return sum;
    }

    template <typename T>
    void Histogram<T, 1>::addSample(const T& sample) {
      if (this->isInRange((Coordinate() << sample).finished()))
        this->getCell(this->getIndex((Coordinate() << sample).finished()))++;
    }

    template <typename T>
    void Histogram<T, 1>::addSamples(const std::vector<T>& samples) {
      for (size_t i = 0; i < samples.size(); ++i)
        addSample(samples[i]);
    }

    template <typename T>
    Histogram<T, 1> Histogram<T, 1>::getNormalized() const {
      const double sum = getSum();
      auto histCopy = *this;
      for (auto it = histCopy.getCellBegin(); it != histCopy.getCellEnd(); ++it)
        *it /= sum;
      return histCopy;
    }

  }
}
