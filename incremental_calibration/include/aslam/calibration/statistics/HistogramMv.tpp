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

    template <typename T, int M>
    Histogram<T, M>::Histogram(const Coordinate& min, const Coordinate& max,
        const Coordinate& binSize) :
        Grid<T, double, M>(min, max, binSize) {
    }

    template <typename T, int M>
    Histogram<T, M>::Histogram(const Histogram& other) :
      Grid<T, double, M>(other) {
    }

    template <typename T, int M>
    Histogram<T, M>& Histogram<T, M>::operator = (const Histogram& other) {
      if (this != &other) {
        Grid<T, double, M>::operator=(other);
      }
      return *this;
    }

    template <typename T, int M>
    Histogram<T, M>::~Histogram() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename T, int M>
    typename Histogram<T, M>::Mean Histogram<T, M>::getMean() const {
      Mean mean = Mean::Zero(this->mNumCells.size());
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i))
        mean += this->getCoordinates(i).template cast<double>() *
          this->getCell(i);
      return mean / getSum();
    }

    template <typename T, int M>
    typename Histogram<T, M>::Mode Histogram<T, M>::getMode() const {
      double max = -std::numeric_limits<double>::infinity();
      Index modeIdx;
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i)) {
        if (this->getCell(i) > max) {
          max = this->getCell(i);
          modeIdx = i;
        }
      }
      return this->getCoordinates(modeIdx);
    }

    template <typename T, int M>
    typename Histogram<T, M>::Covariance Histogram<T, M>::getCovariance()
        const {
      Covariance covariance = Covariance::Zero(this->mNumCells.size(),
        this->mNumCells.size());
      const Mean mean = getMean();
      for (Index i = Index::Zero(this->mNumCells.size()); i != this->mNumCells;
          incrementIndex(i))
        covariance += (this->getCoordinates(i).template cast<double>() - mean) *
          (this->getCoordinates(i).template cast<double>() - mean).transpose() *
          this->getCell(i);
      return covariance / (getSum() - 1);
    }

    template <typename T, int M>
    double Histogram<T, M>::getSum() const {
      double sum = 0;
      for (auto it = this->getCellBegin(); it != this->getCellEnd(); ++it)
        sum += *it;
      return sum;
    }

    template <typename T, int M>
    void Histogram<T, M>::addSample(const Coordinate& sample) {
      if (this->isInRange(sample))
        this->getCell(this->getIndex(sample))++;
    }

    template <typename T, int M>
    void Histogram<T, M>::addSamples(const std::vector<Coordinate>& samples) {
      for (size_t i = 0; i < samples.size(); ++i)
        addSample(samples[i]);
    }

    template <typename T, int M>
    Histogram<T, M> Histogram<T, M>::getNormalized() const {
      const double sum = getSum();
      auto histCopy = *this;
      for (auto it = histCopy.getCellBegin(); it != histCopy.getCellEnd(); ++it)
        *it /= sum;
      return histCopy;
    }

  }
}
