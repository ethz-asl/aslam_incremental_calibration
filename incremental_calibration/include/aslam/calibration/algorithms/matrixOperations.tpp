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

#include "aslam/calibration/exceptions/OutOfBoundException.h"

#include <cmath>
#include <cstddef>

#include <limits>

namespace aslam {
  namespace calibration {

    template <typename T>
    void checkColumnIndices(const T& R, size_t colBegin, size_t colEnd) {
      if (colBegin >= colEnd)
        throw OutOfBoundException<size_t>(colBegin,
          "checkColumnIndices(): "
          "column index begin must be smaller that column end index",
          __FILE__, __LINE__);
       if (colEnd >= static_cast<size_t>(R.cols()))
         throw OutOfBoundException<size_t>(colEnd,
           "checkColumnIndices(): "
           "column end index must be smaller than the columns of R",
           __FILE__, __LINE__);
    }

    template <typename T>
    double computeSumLogDiagR(const T& R, size_t colBegin, size_t colEnd) {
      checkColumnIndices(R, colBegin, colEnd);
      // NOTE: What about checking the form of R? Upper triangular matrix
      double sumLogDiagR = 0;
      for (size_t i = colBegin; i <= colEnd; ++i) {
        const double value = R(i, i);
        if (std::fabs(value) > std::numeric_limits<double>::epsilon())
          sumLogDiagR += std::log2(std::fabs(value));
      }
      return sumLogDiagR;
    }
  }
}
