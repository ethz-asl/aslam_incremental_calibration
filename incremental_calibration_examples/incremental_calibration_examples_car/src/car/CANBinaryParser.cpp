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
#include <libcan-prius/types/Steering2.h>
#include <libcan-prius/types/Acceleration1.h>
#include <libcan-prius/types/Acceleration2.h>
#include <libcan-prius/types/Speed1.h>
#include <libcan-prius/types/Speed2.h>
#include <libcan-prius/types/Speed3.h>
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

    size_t CANBinaryParser::getNumSteering1() const {
      return _st1.size();
    }

    size_t CANBinaryParser::getNumSteering2() const {
      return _st2.size();
    }

    size_t CANBinaryParser::getNumAcceleration1() const {
      return _acc1.size();
    }

    size_t CANBinaryParser::getNumAcceleration2() const {
      return _acc2.size();
    }

    CANBinaryParser::WSpeedContainerCIt CANBinaryParser::cbeginFw() const {
      return _fws.cbegin();
    }

    CANBinaryParser::WSpeedContainerCIt CANBinaryParser::cendFw() const {
      return _fws.cend();
    }

    CANBinaryParser::WSpeedContainerCIt CANBinaryParser::cbeginRw() const {
      return _rws.cbegin();
    }

    CANBinaryParser::WSpeedContainerCIt CANBinaryParser::cendRw() const {
      return _rws.cend();
    }

    CANBinaryParser::SteeringContainerCIt CANBinaryParser::cbeginSt1() const {
      return _st1.cbegin();
    }

    CANBinaryParser::SteeringContainerCIt CANBinaryParser::cendSt1() const {
      return _st1.cend();
    }

    CANBinaryParser::SteeringContainerCIt CANBinaryParser::cbeginSt2() const {
      return _st2.cbegin();
    }

    CANBinaryParser::SteeringContainerCIt CANBinaryParser::cendSt2() const {
      return _st2.cend();
    }

    CANBinaryParser::AccContainerCIt CANBinaryParser::cbeginAcc1() const {
      return _acc1.cbegin();
    }

    CANBinaryParser::AccContainerCIt CANBinaryParser::cendAcc1() const {
      return _acc1.cend();
    }

    CANBinaryParser::AccContainerCIt CANBinaryParser::cbeginAcc2() const {
      return _acc2.cbegin();
    }

    CANBinaryParser::AccContainerCIt CANBinaryParser::cendAcc2() const {
      return _acc2.cend();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CANBinaryParser::parse() {
      _fws.clear();
      _rws.clear();
      _st1.clear();
      _st2.clear();
      _acc1.clear();
      _acc2.clear();
      _sp1.clear();
      _sp2.clear();
      _sp3.clear();
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
          _st1[timestamp] = st.mValue;
        }
        else if (priusMessage->instanceOf<Steering2>()) {
          const Steering2& st = priusMessage->typeCast<Steering2>();
          _st2[timestamp] = st.mValue;
        }
        else if (priusMessage->instanceOf<Acceleration1>()) {
          const Acceleration1& acc = priusMessage->typeCast<Acceleration1>();
          _acc1[timestamp] = std::make_pair(acc.mValue1, acc.mValue2);
        }
        else if (priusMessage->instanceOf<Acceleration2>()) {
          const Acceleration2& acc = priusMessage->typeCast<Acceleration2>();
          _acc2[timestamp] = std::make_pair(acc.mValue1, acc.mValue2);
        }
        else if (priusMessage->instanceOf<Speed1>()) {
          const Speed1& sp = priusMessage->typeCast<Speed1>();
          _sp1[timestamp] = sp.mValue;
        }
        else if (priusMessage->instanceOf<Speed2>()) {
          const Speed2& sp = priusMessage->typeCast<Speed2>();
          _sp2[timestamp] = sp.mValue;
        }
        else if (priusMessage->instanceOf<Speed3>()) {
          const Speed3& sp = priusMessage->typeCast<Speed3>();
          _sp3[timestamp] = sp.mSpeed;
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
      if (_st1.size() > 0) {
        auto it = _st1.end();
        --it;
        _st1.erase(it);
      }
      if (_st2.size() > 0) {
        auto it = _st2.end();
        --it;
        _st2.erase(it);
      }
      if (_acc1.size() > 0) {
        auto it = _acc1.end();
        --it;
        _acc1.erase(it);
      }
      if (_acc2.size() > 0) {
        auto it = _acc2.end();
        --it;
        _acc2.erase(it);
      }
      if (_sp1.size() > 0) {
        auto it = _sp1.end();
        --it;
        _sp1.erase(it);
      }
      if (_sp2.size() > 0) {
        auto it = _sp2.end();
        --it;
        _sp2.erase(it);
      }
      if (_sp3.size() > 0) {
        auto it = _sp3.end();
        --it;
        _sp3.erase(it);
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

    void CANBinaryParser::writeSt1MATLAB(std::ostream& stream) const {
      for (auto it = _st1.cbegin(); it != _st1.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second << std::endl;
    }

    void CANBinaryParser::writeSt2MATLAB(std::ostream& stream) const {
      for (auto it = _st2.cbegin(); it != _st2.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second << std::endl;
    }

    void CANBinaryParser::writeAcc1MATLAB(std::ostream& stream) const {
      for (auto it = _acc1.cbegin(); it != _acc1.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second.first << " "
          << it->second.second << std::endl;
    }

    void CANBinaryParser::writeAcc2MATLAB(std::ostream& stream) const {
      for (auto it = _acc2.cbegin(); it != _acc2.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second.first << " "
          << it->second.second << std::endl;
    }

    void CANBinaryParser::writeSp1MATLAB(std::ostream& stream) const {
      for (auto it = _sp1.cbegin(); it != _sp1.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second << std::endl;
    }

    void CANBinaryParser::writeSp2MATLAB(std::ostream& stream) const {
      for (auto it = _sp2.cbegin(); it != _sp2.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second << std::endl;
    }

    void CANBinaryParser::writeSp3MATLAB(std::ostream& stream) const {
      for (auto it = _sp3.cbegin(); it != _sp3.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second << std::endl;
    }

  }
}
