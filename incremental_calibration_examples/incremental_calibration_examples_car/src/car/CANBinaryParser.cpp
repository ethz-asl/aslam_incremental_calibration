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

#include "aslam/calibration/car/CANBinaryParser.h"

#include <fstream>
#include <memory>
#include <iomanip>

#include <libcan-prius/exceptions/IOException.h>
#include <libcan-prius/base/BinaryStreamReader.h>
#include <libcan-prius/types/PRIUSMessage.h>
#include <libcan-prius/types/FrontWheelsSpeed.h>
#include <libcan-prius/types/RearWheelsSpeed.h>
#include <libcan-prius/types/Steering1.h>
#include <libcan-prius/base/Factory.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CANBinaryParser::CANBinaryParser(const std::string& filename) :
        _filename(filename) {
    }

    CANBinaryParser::~CANBinaryParser() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const std::string& CANBinaryParser::getFilename() const {
      return _filename;
    }

    void CANBinaryParser::setFilename(const std::string& filename) {
      _filename = filename;
    }

    size_t CANBinaryParser::getNumFrontWheels() const {
      return _fws.size();
    }

    size_t CANBinaryParser::getNumRearWheels() const {
      return _rws.size();
    }

    size_t CANBinaryParser::getNumSteering() const {
      return _st.size();
    }

    CANBinaryParser::SpeedContainerCIt CANBinaryParser::cbeginFw() const {
      return _fws.cbegin();
    }

    CANBinaryParser::SpeedContainerCIt CANBinaryParser::cendFw() const {
      return _fws.cend();
    }

    CANBinaryParser::SpeedContainerCIt CANBinaryParser::cbeginRw() const {
      return _rws.cbegin();
    }

    CANBinaryParser::SpeedContainerCIt CANBinaryParser::cendRw() const {
      return _rws.cend();
    }

    CANBinaryParser::SteeringContainerCIt CANBinaryParser::cbeginSt() const {
      return _st.cbegin();
    }

    CANBinaryParser::SteeringContainerCIt CANBinaryParser::cendSt() const {
      return _st.cend();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CANBinaryParser::parse() {
      _fws.clear();
      _rws.clear();
      _st.clear();
      std::ifstream binaryLogFile(_filename);
      if (!binaryLogFile.is_open())
        throw IOException("CANBinaryParser::parse(): file opening failed");
      BinaryStreamReader<std::ifstream> logReader(binaryLogFile);
      binaryLogFile.seekg (0, std::ios::end);
      const int length = binaryLogFile.tellg();
      binaryLogFile.seekg (0, std::ios::beg);
      while (binaryLogFile.tellg() != length) {
        double timestamp;
        logReader >> timestamp;
        int typeID;
        logReader >> typeID;
        std::shared_ptr<PRIUSMessage>
          priusMessage(Factory<int, PRIUSMessage>::getInstance().create(
          typeID));
        logReader >> *priusMessage;
        if (priusMessage->instanceOf<FrontWheelsSpeed>()) {
          const FrontWheelsSpeed& fws =
            priusMessage->typeCast<FrontWheelsSpeed>();
          _fws[timestamp] = std::make_pair(fws.mRight, fws.mLeft);
        }
        else if (priusMessage->instanceOf<RearWheelsSpeed>()) {
          const RearWheelsSpeed& rws =
            priusMessage->typeCast<RearWheelsSpeed>();
          _rws[timestamp] = std::make_pair(rws.mRight, rws.mLeft);
        }
        else if (priusMessage->instanceOf<Steering1>()) {
          const Steering1& st = priusMessage->typeCast<Steering1>();
          _st[timestamp] = st.mValue;
        }
      }
      // kick out the last measurement, problem with parsing!
      if (_fws.size() > 0) {
        auto it = _fws.end();
        --it;
        _fws.erase(it);
      }
      if (_rws.size() > 0) {
        auto it = _rws.end();
        --it;
        _rws.erase(it);
      }
      if (_st.size() > 0) {
        auto it = _st.end();
        --it;
        _st.erase(it);
      }
    }

    void CANBinaryParser::writeFwMATLAB(std::ostream& stream) const {
      for (auto it = _fws.cbegin(); it != _fws.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second.first << " "
          << it->second.second << std::endl;
    }

    void CANBinaryParser::writeRwMATLAB(std::ostream& stream) const {
      for (auto it = _rws.cbegin(); it != _rws.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second.first << " "
          << it->second.second << std::endl;
    }

    void CANBinaryParser::writeStMATLAB(std::ostream& stream) const {
      for (auto it = _st.cbegin(); it != _st.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second << std::endl;
    }

  }
}
