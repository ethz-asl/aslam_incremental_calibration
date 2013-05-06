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

#include "aslam/calibration/car/AckermanOdometry.h"

#include <Eigen/Dense>

#include <cmath>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    AckermanOdometry::AckermanOdometry(const Odometry::VehicleParameters&
        parameters, const Eigen::Vector3d& initialPose) :
        Odometry(parameters, initialPose) {
    }

    AckermanOdometry::~AckermanOdometry() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void AckermanOdometry::updateWheelTranslationalVelocitiesSteering(
        double vRearLeftWheel, double vRearRightWheel, double vFrontLeftWheel,
        double vFrontRightWheel, double steering, double dT) {
      updateWheelDisplacementsSteering(dT * vRearLeftWheel,
        dT * vRearRightWheel, dT * vFrontLeftWheel, dT * vFrontRightWheel,
        steering);
    }

    void AckermanOdometry::updateWheelRotationalVelocitiesSteering(
        double wRearLeftWheel, double wRearRightWheel, double wFrontLeftWheel,
        double wFrontRightWheel, double steering, double dT) {
      const double RL = _parameters._rearLeftWheelRadius;
      const double RR = _parameters._rearRightWheelRadius;
      const double FL = _parameters._frontLeftWheelRadius;
      const double FR = _parameters._frontRightWheelRadius;
      updateWheelDisplacementsSteering(dT * RL * wRearLeftWheel,
        dT * RR * wRearRightWheel, dT * FL * wFrontLeftWheel,
        dT * FR * wFrontRightWheel, steering);
    }

    void AckermanOdometry::updateWheelDisplacementsSteering(
        double dRearLeftWheel, double dRearRightWheel, double dFrontLeftWheel,
        double dFrontRightWheel, double steering) {
      const double L = _parameters._wheelBase;
      const double e = _parameters._wheelTrack * 0.5;
      const double tanPhi = tan(steering);
      const double phiL = atan(tanPhi * L / (L - e * tanPhi));
      const double phiR = atan(tanPhi * L / (L + e * tanPhi));
      Eigen::MatrixXd A = Eigen::MatrixXd(4, 2);
      A << 1, -e, 1, e, 1, -e, 1, e;
      Eigen::Vector4d b;
      b << dRearLeftWheel, dRearRightWheel, dFrontLeftWheel * cos(phiL),
        dFrontRightWheel * cos(phiR);
      Eigen::Vector2d x = A.jacobiSvd(Eigen::ComputeThinU |
        Eigen::ComputeThinV).solve(b);
      updateCOGDisplacement(x(0), x(1));
    }

  }
}
