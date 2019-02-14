/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file permute.h
    \brief This file defines the permute function, which permutes containers.
  */

#ifndef ASLAM_CALIBRATION_ALGORITHMS_PERMUTE_H
#define ASLAM_CALIBRATION_ALGORITHMS_PERMUTE_H

#include <vector>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /** 
     * This function permutes an STL vector.
     * \brief STL vector permutation
     * 
     * \param[in, out] container vector to be permuted
     * \param[in] permutation permutation vector
     */
    template <typename T, typename I> void permute(std::vector<T>& container,
      const std::vector<I>& permutation);
    /** @}
      */

  }
}

#include "aslam/calibration/algorithms/permute.tpp"

#endif // ASLAM_CALIBRATION_ALGORITHMS_PERMUTE_H
