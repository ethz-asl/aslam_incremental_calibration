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
      typedef std::map<double, std::pair<double, double> > WheelSpeedContainer;
      /// Wheel speed container iterator
      typedef WheelSpeedContainer::iterator WSpeedContainerIt;
      /// Wheel speed container const iterator
      typedef WheelSpeedContainer::const_iterator WSpeedContainerCIt;
      /// Container for steering
      typedef std::map<double, double> SteeringContainer;
      /// Steering container iterator
      typedef SteeringContainer::iterator SteeringContainerIt;
      /// Steering container const iterator
      typedef SteeringContainer::const_iterator SteeringContainerCIt;
      /// Container for acceleration
      typedef WheelSpeedContainer AccContainer;
      /// Wheel speed container iterator
      typedef WheelSpeedContainer::iterator AccContainerIt;
      /// Wheel speed container const iterator
      typedef WheelSpeedContainer::const_iterator AccContainerCIt;
      /// Container for speed
      typedef std::map<double, double> SpeedContainer;
      /// Speed container iterator
      typedef SpeedContainer::iterator SpeedContainerIt;
      /// Speed container const iterator
      typedef SpeedContainer::const_iterator SpeedContainerCIt;
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
      /// Returns the number of steering1 messages
      size_t getNumSteering1() const;
      /// Returns the number of steering2 messages
      size_t getNumSteering2() const;
      /// Returns the number of acceleration1 messages
      size_t getNumAcceleration1() const;
      /// Returns the number of acceleration2 messages
      size_t getNumAcceleration2() const;
      /// Returns iterator to start of front wheel messages
      WSpeedContainerCIt cbeginFw() const;
      /// Returns iterator to end of front wheel messages
      WSpeedContainerCIt cendFw() const;
      /// Returns iterator to start of rear wheel messages
      WSpeedContainerCIt cbeginRw() const;
      /// Returns iterator to end of rear wheel messages
      WSpeedContainerCIt cendRw() const;
      /// Returns iterator to start of steering1 messages
      SteeringContainerCIt cbeginSt1() const;
      /// Returns iterator to end of steering1 messages
      SteeringContainerCIt cendSt1() const;
      /// Returns iterator to start of steering2 messages
      SteeringContainerCIt cbeginSt2() const;
      /// Returns iterator to end of steering2 messages
      SteeringContainerCIt cendSt2() const;
      /// Returns iterator to start of acceleration1 messages
      AccContainerCIt cbeginAcc1() const;
      /// Returns iterator to end of acceleration1 messages
      AccContainerCIt cendAcc1() const;
      /// Returns iterator to start of acceleration2 messages
      AccContainerCIt cbeginAcc2() const;
      /// Returns iterator to end of acceleration2 messages
      AccContainerCIt cendAcc2() const;
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
      /// Writes steering1 to MATLAB compatible format
      void writeSt1MATLAB(std::ostream& stream) const;
      /// Writes steering2 to MATLAB compatible format
      void writeSt2MATLAB(std::ostream& stream) const;
      /// Writes acceleration1 to MATLAB compatible format
      void writeAcc1MATLAB(std::ostream& stream) const;
      /// Writes acceleration2 to MATLAB compatible format
      void writeAcc2MATLAB(std::ostream& stream) const;
      /// Writes speed1 to MATLAB compatible format
      void writeSp1MATLAB(std::ostream& stream) const;
      /// Writes speed2 to MATLAB compatible format
      void writeSp2MATLAB(std::ostream& stream) const;
      /// Writes speed3 to MATLAB compatible format
      void writeSp3MATLAB(std::ostream& stream) const;
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// File name of the log file
      std::string _filename;
      /// Container for front wheel speeds
      WheelSpeedContainer _fws;
      /// Container for rear wheel speeds
      WheelSpeedContainer _rws;
      /// Container for steering 1
      SteeringContainer _st1;
      /// Container for steering 2
      SteeringContainer _st2;
      /// Container for acceleration 1
      AccContainer _acc1;
      /// Container for acceleration 2
      AccContainer _acc2;
      /// Container for speed 1
      SpeedContainer _sp1;
      /// Container for speed 2
      SpeedContainer _sp2;
      /// Container for speed 3
      SpeedContainer _sp3;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAN_BINARY_PARSER_H
