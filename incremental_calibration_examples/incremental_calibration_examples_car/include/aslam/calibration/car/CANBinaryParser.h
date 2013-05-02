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

/** \file CANBinaryParser.h
    \brief This file defines the CANBinaryParser class which implements
           a parser for CAN binary data.
  */

#ifndef ASLAM_CALIBRATION_CAN_BINARY_PARSER_H
#define ASLAM_CALIBRATION_CAN_BINARY_PARSER_H

#include <string>
#include <map>

namespace aslam {
  namespace calibration {

    /** The class CANBinaryParser implements a parser for CAN binary data.
        \brief CAN binary parser
      */
    class CANBinaryParser {
    public:
      /** \name Types definitions
        @{
        */
      /// Container for wheel speeds
      typedef std::map<double, std::pair<double, double> > SpeedContainer;
      /// Wheel speed container iterator
      typedef SpeedContainer::iterator SpeedContainerIt;
      /// Wheel speed container const iterator
      typedef SpeedContainer::const_iterator SpeedContainerCIt;
      /// Container for steering
      typedef std::map<double, double> SteeringContainer;
      /// Steering container iterator
      typedef SteeringContainer::iterator SteeringContainerIt;
      /// Steering container const iterator
      typedef SteeringContainer::const_iterator SteeringContainerCIt;
      /// Self type
      typedef CANBinaryParser Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      CANBinaryParser(const std::string& filename);
      /// Copy constructor
      CANBinaryParser(const Self& other) = delete;
      /// Copy assignment operator
      CANBinaryParser& operator = (const Self& other) = delete;
      /// Move constructor
      CANBinaryParser(Self&& other) = delete;
      /// Move assignment operator
      CANBinaryParser& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~CANBinaryParser();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the filename of the log file
      const std::string& getFilename() const;
      /// Sets the filename of the log file
      void setFilename(const std::string& filename);
      /// Returns the number of front wheel messages
      size_t getNumFrontWheels() const;
      /// Returns the number of rear wheel messages
      size_t getNumRearWheels() const;
      /// Returns the number of steering messages
      size_t getNumSteering() const;
      /// Returns iterator to start of front wheel messages
      SpeedContainerCIt cbeginFw() const;
      /// Returns iterator to end of front wheel messages
      SpeedContainerCIt cendFw() const;
      /// Returns iterator to start of rear wheel messages
      SpeedContainerCIt cbeginRw() const;
      /// Returns iterator to end of rear wheel messages
      SpeedContainerCIt cendRw() const;
      /// Returns iterator to start of steering messages
      SteeringContainerCIt cbeginSt() const;
      /// Returns iterator to end of steering messages
      SteeringContainerCIt cendSt() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Parses the log file and fills data structures
      void parse();
      /// Writes front wheel speeds to MATLAB compatible format
      void writeFwMATLAB(std::ostream& stream) const;
      /// Writes rear wheel speeds to MATLAB compatible format
      void writeRwMATLAB(std::ostream& stream) const;
      /// Writes steering to MATLAB compatible format
      void writeStMATLAB(std::ostream& stream) const;
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// File name of the log file
      std::string _filename;
      /// Container for front wheel speeds
      SpeedContainer _fws;
      /// Container for rear wheel speeds
      SpeedContainer _rws;
      /// Container for steering
      SteeringContainer _st;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAN_BINARY_PARSER_H
