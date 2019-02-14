/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/car/geo/geodetic.h"

#include <Eigen/Core>

#include <cmath>

#include <sm/kinematics/quaternion_algebra.hpp>

using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    sm::kinematics::Transformation ecef2enu(double x, double y, double z, double
        latitude, double longitude) {
      return enu2ecef(x, y, z, latitude, longitude).inverse();
    }

    sm::kinematics::Transformation enu2ecef(double x, double y, double z, double
        latitude, double longitude) {

      const double slat = std::sin(latitude);
      const double clat = std::cos(latitude);
      const double slong = std::sin(longitude);
      const double clong = std::cos(longitude);

      Eigen::Matrix3d enu_R_ecef;
      enu_R_ecef << -slong, clong, 0,
        -slat * clong, -slat * slong, clat,
        clat * clong, clat * slong, slat;

      return Transformation(
        r2quat(enu_R_ecef.transpose()), Eigen::Vector3d(x, y, z));
    }

    sm::kinematics::Transformation ecef2ned(double x, double y, double z, double
        latitude, double longitude) {
      return ned2ecef(x, y, z, latitude, longitude).inverse();
    }

    sm::kinematics::Transformation ned2ecef(double x, double y, double z, double
        latitude, double longitude) {

      const double slat = std::sin(latitude);
      const double clat = std::cos(latitude);
      const double slong = std::sin(longitude);
      const double clong = std::cos(longitude);

      Eigen::Matrix3d ned_R_ecef;
      ned_R_ecef << -slat * clong, -slat * slong, clat,
        -slong, clong, 0,
        -clat * clong, -clat * slong, -slat;

      return Transformation(
        r2quat(ned_R_ecef.transpose()), Eigen::Vector3d(x, y, z));
    }

    void wgs84ToEcef(double latitude, double longitude, double altitude,
        double& x, double& y, double& z) {
      const double slat = std::sin(latitude);
      const double clat = std::cos(latitude);
      const double slong = std::sin(longitude);
      const double clong = std::cos(longitude);
      const double a = 6378137;
      const double e2 = 0.006694380004260827;
      const double R = a / std::sqrt(1 - e2 * slat * slat);
      x = (R + altitude) * clat * clong;
      y = (R + altitude) * clat * slong;
      z = (R * (1 - e2) + altitude) * slat;
    }

  }
}
