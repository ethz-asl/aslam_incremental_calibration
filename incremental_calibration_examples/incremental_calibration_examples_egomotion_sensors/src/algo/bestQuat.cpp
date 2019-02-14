/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/egomotion/algo/bestQuat.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    Eigen::Vector4d bestQuat(const Eigen::Vector4d& pquat, const
        Eigen::Vector4d& cquat) {
      if ((pquat + cquat).norm() < (pquat - cquat).norm())
        return -cquat;
      else
        return cquat;
    }

  }
}
