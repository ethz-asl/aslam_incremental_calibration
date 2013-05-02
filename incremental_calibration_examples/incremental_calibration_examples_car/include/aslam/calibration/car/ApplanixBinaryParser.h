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

/** \file ApplanixBinaryParser.h
    \brief This file defines the ApplanixBinaryParser class which implements
           a parser for Applanix binary data.
  */

#ifndef ASLAM_CALIBRATION_APPLANIX_BINARY_PARSER_H
#define ASLAM_CALIBRATION_APPLANIX_BINARY_PARSER_H

#include <string>
#include <map>

namespace aslam {
  namespace calibration {

    /** The class ApplanixBinaryParser implements a parser for Applanix binary
        data.
        \brief Applanix binary parser
      */
    class ApplanixBinaryParser {
    public:
      /** \name Types definitions
        @{
        */
      /// Vehicle navigation solution in local ENU system
      struct NavigationSolution {
        /// x pose [m]
        double x;
        /// y pose [m]
        double y;
        /// z pose [m]
        double z;
        /// roll [rad]
        double roll;
        /// pitch [rad]
        double pitch;
        /// yaw [rad]
        double yaw;
        /// body linear velocity in x in world frame [m/s]
        double v_x;
        /// body linear velocity in y in world frame [m/s]
        double v_y;
        /// body linear velocity in z in world frame [m/s]
        double v_z;
        /// body angular velocity in x in body frame [rad/s]
        double om_x;
        /// body angular velocity in y in body frame [rad/s]
        double om_y;
        /// body angular velocity in z in body frame [rad/s]
        double om_z;
        /// body linear acceleration in x in body frame [m/s^2]
        double a_x;
        /// body linear acceleration in y in body frame [m/s^2]
        double a_y;
        /// body linear acceleration in z in body frame [m/s^2]
        double a_z;
        /// linear velocity [m/s]
        double v;
        /// x pose sigma^2
        double x_sigma2;
        /// y pose sigma^2
        double y_sigma2;
        /// z pose sigma^2
        double z_sigma2;
        /// roll sigma^2
        double roll_sigma2;
        /// pitch sigma^2
        double pitch_sigma2;
        /// yaw sigma^2
        double yaw_sigma2;
        /// linear velocity in x sigma^2
        double v_x_sigma2;
        /// linear velocity in y sigma^2
        double v_y_sigma2;
        /// linear velocity in z sigma^2
        double v_z_sigma2;
      };
      /// Message container
      typedef std::map<double, NavigationSolution> Container;
      /// Message container iterator
      typedef Container::iterator ContainerIt;
      /// Message container constant iterator
      typedef Container::const_iterator ContainerCIt;
      /// Self type
      typedef ApplanixBinaryParser Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      ApplanixBinaryParser(const std::string& filename);
      /// Copy constructor
      ApplanixBinaryParser(const Self& other) = delete;
      /// Copy assignment operator
      ApplanixBinaryParser& operator = (const Self& other) = delete;
      /// Move constructor
      ApplanixBinaryParser(Self&& other) = delete;
      /// Move assignment operator
      ApplanixBinaryParser& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~ApplanixBinaryParser();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the filename of the log file
      const std::string& getFilename() const;
      /// Sets the filename of the log file
      void setFilename(const std::string& filename);
      /// Returns the number of navigation solution messages
      size_t getNumNavigationSolution() const;
      /// Returns iterator to start
      ContainerCIt cbegin() const;
      /// Returns iterator to end
      ContainerCIt cend() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Parses the log file and fills data structures
      void parse();
      /// Writes to MATLAB compatible format
      void writeMATLAB(std::ostream& stream) const;
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// File name of the log file
      std::string _filename;
      /// Vehicle navigation solution messages
      Container _navigationSolution;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_APPLANIX_BINARY_PARSER_H
