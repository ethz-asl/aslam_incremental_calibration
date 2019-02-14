/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file geodetic.h
    \brief This file contains utilities for geodetic datum.
  */

#ifndef ASLAM_CALIBRATION_CAR_GEODETIC_H
#define ASLAM_CALIBRATION_CAR_GEODETIC_H

#include <sm/kinematics/Transformation.hpp>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /// Returns the transformation from ECEF to ENU
    sm::kinematics::Transformation ecef2enu(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the transformation from ENU to ECEF
    sm::kinematics::Transformation enu2ecef(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the transformation from ECEF to NED
    sm::kinematics::Transformation ecef2ned(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the transformation from NED to ECEF
    sm::kinematics::Transformation ned2ecef(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the ECEF coordinates from WGS84
    void wgs84ToEcef(double latitude, double longitude, double altitude,
      double& x, double& y, double& z);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_CAR_GEODETIC_H
