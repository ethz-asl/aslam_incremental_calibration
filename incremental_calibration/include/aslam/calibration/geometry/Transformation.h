/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Transformation.h
    \brief This file is an interface to the 2d/3d transformations.
  */

#ifndef ASLAM_CALIBRATION_GEOMETRY_TRANSFORMATION_H
#define ASLAM_CALIBRATION_GEOMETRY_TRANSFORMATION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

template <typename T = double, int M = 2> class Transformation;

  }
}

#include "aslam/calibration/geometry/Transformation2d.h"
#include "aslam/calibration/geometry/Transformation3d.h"

#endif // ASLAM_CALIBRATION_GEOMETRY_TRANSFORMATION_H
