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

#include "aslam/calibration/core/IncrementalEstimator.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalEstimator::IncrementalEstimator() {
    }

    IncrementalEstimator::IncrementalEstimator(
        const IncrementalEstimator& other) {
    }

    IncrementalEstimator& IncrementalEstimator::operator =
        (const IncrementalEstimator& other) {
      if (this != &other) {
      }
      return *this;
    }

    IncrementalEstimator::~IncrementalEstimator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    bool IncrementalEstimator::addMeasurementBatch(const std::vector<
        boost::shared_ptr<aslam::backend::ErrorTerm> >& errorTermsNew) {

      // add the new measurements into the informative set
      _errorTermsInfo.reserve(_errorTermsInfo.size() + errorTermsNew.size());
      _errorTermsInfo.insert(_errorTermsInfo.end(), errorTermsNew.begin(),
        errorTermsNew.end());
      return true;
    }

  }
}
