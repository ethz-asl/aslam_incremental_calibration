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

/** \file IncrementalCalibration.h
    \brief This file defines the IncrementalCalibration class, which implements
           the core algorithm for incremental calibration.
  */

#ifndef ASLAM_CALIBRATION_CORE_INCREMENTAL_CALIBRATION_H
#define ASLAM_CALIBRATION_CORE_INCREMENTAL_CALIBRATION_H

namespace aslam {
  namespace calibration {

    /** The class IncrementalCalibration implements the core algorithm for
        incremental calibration.
        \brief Incremental calibration
      */
    class IncrementalCalibration {
    public:
      /** \name Types definitions
        @{
        */
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      IncrementalCalibration();
      /// Copy constructor
      IncrementalCalibration(const IncrementalCalibration& other);
      /// Assignment operator
      IncrementalCalibration& operator = (const IncrementalCalibration& other);
      /// Destructor
      virtual ~IncrementalCalibration();
      /** @}
        */

      /** \name Methods
        @{
        */
      /** @}
        */

      /** \name Accessors
        @{
        */
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /** @}
        */

      /** \name Protected members
        @{
        */
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_INCREMENTAL_CALIBRATION_H
