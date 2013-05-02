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

#include "aslam/calibration/car/utils.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    Eigen::Vector3d rotVectorNoFlipping(const Eigen::Vector3d& prv,
        const Eigen::Vector3d& crv) {
      // current angle
      const double angle = crv.norm();
      // current axis
      const Eigen::Vector3d axis = crv / angle;
      // best rotation vector
      Eigen::Vector3d best_rv = crv;
      // best distance
      double best_dist = (best_rv - prv).norm();
      // find best vector
      for (int s = -3; s <= 4; ++s) {
        const Eigen::Vector3d aa = axis * (angle + M_PI * 2.0 * s);
        const double dist = (aa - prv).norm();
        if (dist < best_dist) {
          best_rv = aa;
          best_dist = dist;
        }
      }
      return best_rv;
    }

  }
}
