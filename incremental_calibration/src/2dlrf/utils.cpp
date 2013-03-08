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

#include "aslam/calibration/2dlrf/utils.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void genSineWavePath(std::vector<Eigen::Matrix<double, 3, 1> >& u,
        size_t steps, double amplitude, double frequency, double T) {
      u.clear();
      u.reserve(steps);
      u.push_back(Eigen::Matrix<double, 3, 1>::Zero());
      std::vector<double> cost;
      for (double t = 0; t < steps * T; t += T)
        cost.push_back(amplitude * cos(2 * M_PI * frequency * t));
      double theta = 0;
      for (size_t i = 1; i < cost.size(); ++i) {
        const double theta_new = atan(cost[i]);
        const double v = 2 * M_PI * frequency *
          (sqrt(1 + cost[i] * cost[i]) +
          sqrt(1 + cost[i - 1] * cost[i - 1])) / 2;
        const double om = (theta_new - theta) / T;
        theta = theta_new;
        u.push_back(Eigen::Matrix<double, 3, 1>(v, 0, om));
      }
    }

    void initLandmarks(std::vector<Eigen::Matrix<double, 2, 1> >& x_l_hat,
        const std::vector<Eigen::Matrix<double, 3, 1> >& x_odom,
        const Eigen::Matrix<double, 3, 1>& Theta_hat,
        const std::vector<std::vector<double> >& r,
        const std::vector<std::vector<double> >& b) {
      if (r.size() < 1)
        return;
      const size_t nl = r[0].size();
      x_l_hat.clear();
      x_l_hat.assign(nl, Eigen::Matrix<double, 2, 1>::Zero());
      std::vector<bool> l_found(nl, false);
      for (size_t i = 0; i < x_odom.size(); ++i) {
        for (size_t j = 0; j < nl; ++j) {
          if (!l_found[j] && r[i][j] > 0) {
            Eigen::Matrix<double, 2, 1> l;
            x_l_hat[j](0) = x_odom[i](0) + Theta_hat(0) * cos(x_odom[i](2)) -
              Theta_hat(1) * sin(x_odom[i](2)) + r[i][j] *
              cos(b[i][j] + Theta_hat(2) + x_odom[i](2));
            x_l_hat[j](1) = x_odom[i](1) + Theta_hat(0) * sin(x_odom[i](2)) +
              Theta_hat(1) * cos(x_odom[i](2)) + r[i][j] * sin(b[i][j] +
              Theta_hat(2) + x_odom[i](2));
            l_found[j] = true;
          }
        }
      }
    }

  }
}
