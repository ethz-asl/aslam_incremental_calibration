/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Serializable.h
    \brief This file defines the Serializable class, which is an interface to
           serializable types
  */

#ifndef ASLAM_CALIBRATION_BASE_SERIALIZABLE_H
#define ASLAM_CALIBRATION_BASE_SERIALIZABLE_H

#include <iostream>
#include <fstream>

namespace aslam {
  namespace calibration {

    /** The class Serializable is an interface to serializable types.
        \brief Serializable class
      */
    class Serializable {
      /** \name Stream methods
        @{
        */
      /// Writes object to standard output
      friend std::ostream& operator << (std::ostream& stream,
        const Serializable& obj);
      /// Reads object from standard input
      friend std::istream& operator >> (std::istream& stream,
        Serializable& obj);
      /// Writes object to file
      friend std::ofstream& operator << (std::ofstream& stream,
        const Serializable& obj);
      /// Reads object from file
      friend std::ifstream& operator >> (std::ifstream& stream,
        Serializable& obj);
      /** @}
        */

    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~Serializable();
      /** @}
        */

    protected:
      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream) = 0;
      /// Writes to standard output
      virtual void write(std::ostream& stream) const = 0;
      /// Reads from a file
      virtual void read(std::ifstream& stream) = 0;
      /// Writes to a file
      virtual void write(std::ofstream& stream) const = 0;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_SERIALIZABLE_H
