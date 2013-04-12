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

#include "aslam/calibration/car/Odometry.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Odometry::Odometry(const VehicleParameters& parameters,
        const Eigen::Vector3d& initialPose) :
      _parameters(parameters),
      _poses(1, initialPose) {
    }

    Odometry::~Odometry() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const Odometry::VehicleParameters& Odometry::getVehicleParameters() const {
      return _parameters;
    }

    Odometry::VehicleParameters& Odometry::getVehicleParameters() {
      return _parameters;
    }

    const Eigen::Vector3d& Odometry::getPose() const {
      return _poses.back();
    }

    const std::vector<Eigen::Vector3d>& Odometry::getPoseHistory() const {
      return _poses;
    }

    void Odometry::reset(const Eigen::Vector3d& initialPose) {
      _poses.assign(1, initialPose);
    }

    void Odometry::insertPose(const Eigen::Vector3d& pose) {
      _poses.push_back(pose);
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void Odometry::updateCOGDisplacement(double dTrans, double dRot) {
      const Eigen::Vector3d& pose_km1 = _poses.back();
      _poses.push_back(Eigen::Vector3d(
        pose_km1(0) + dTrans * cos(pose_km1(2) + dRot * 0.5),
        pose_km1(1) + dTrans * sin(pose_km1(2) + dRot * 0.5),
        pose_km1(2) + dRot));
    }

    void Odometry::updateCOGVelocity(double vTrans, double vRot, double dT) {
      updateCOGDisplacement(vTrans * dT, vRot * dT);
    }

  }
}
