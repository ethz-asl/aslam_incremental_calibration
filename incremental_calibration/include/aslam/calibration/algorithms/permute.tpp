/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/exceptions/OutOfBoundException.h"

#include <utility>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <typename T, typename I>
    void permute(std::vector<T>& container, const std::vector<I>& permutation) {
      if (container.size() != permutation.size())
        throw OutOfBoundException<I>(permutation.size(), container.size(),
          "permutation vector must match container size",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      for (I i = 0; i < container.size(); ++i) {
        if (permutation[i] >= static_cast<I>(container.size()))
          throw OutOfBoundException<I>(permutation[i], static_cast<I>(
            container.size()), "permutation vector index out of bound",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
        I k = permutation[i];
        while (k < i)
          k = permutation[k];
        if (k > i)
          std::swap<T>(container[i], container[k]);
      }
    }

  }
}
