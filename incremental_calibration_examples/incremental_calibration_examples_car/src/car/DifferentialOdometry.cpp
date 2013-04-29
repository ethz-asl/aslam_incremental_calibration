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

#include "aslam/calibration/car/DifferentialOdometry.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    DifferentialOdometry::DifferentialOdometry(
        const Odometry::VehicleParameters& parameters,
        const Eigen::Vector3d& initialPose) :
        Odometry(parameters, initialPose) {
    }

    DifferentialOdometry::~DifferentialOdometry() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void DifferentialOdometry::updateWheelTranslationalVelocities(
        double vLeftWheel, double vRightWheel, double dT) {
      updateWheelDisplacements(vLeftWheel * dT, vRightWheel * dT);
    }

    void DifferentialOdometry::updateWheelRotationalVelocities(
        double wLeftWheel, double wRightWheel, double dT) {
      updateWheelDisplacements(
        _parameters._rearLeftWheelRadius * wLeftWheel * dT,
        _parameters._rearLeftWheelRadius * wRightWheel * dT);
    }

    void DifferentialOdometry::updateWheelDisplacements(
        double dLeftWheel, double dRightWheel) {
      updateCOGDisplacement(0.5 * (dRightWheel + dLeftWheel),
        _parameters._wheelTrack * (dRightWheel - dLeftWheel));
    }

  }
}
