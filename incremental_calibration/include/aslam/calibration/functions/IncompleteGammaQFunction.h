/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file IncompleteGammaQFunction.h
    \brief This file defines the IncompleteGammaQFunction class, which
           represents the normalized incomplete gamma function
  */

#ifndef ASLAM_CALIBRATION_FUNCTIONS_INCOMPLETEGAMMAQFUNCTION_H
#define ASLAM_CALIBRATION_FUNCTIONS_INCOMPLETEGAMMAQFUNCTION_H

#include "aslam/calibration/functions/ContinuousFunction.h"
#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The IncompleteGammaQFunction class represents the normalized incomplete
        gamma function.
        \brief Normalized incomplete gamma function
      */
    class IncompleteGammaQFunction :
      public ContinuousFunction<double, double>,
      public virtual Serializable {
    public:
      /** \name Types
        @{
        */
      /// Variable type
      typedef ContinuousFunction<double, double>::Domain VariableType;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs function with parameters
      IncompleteGammaQFunction(double alpha);
      /// Copy constructor
      IncompleteGammaQFunction(const IncompleteGammaQFunction& other);
      /// Assignment operator
      IncompleteGammaQFunction& operator =
        (const IncompleteGammaQFunction& other);
      /// Destructor
      virtual ~IncompleteGammaQFunction();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the alpha parameter
      double getAlpha() const;
      /// Sets the alpha parameter
      void setAlpha(double alpha);
      /// Access the function value for the given argument
      virtual double getValue(const VariableType& argument) const;
      /** @}
        */

    protected:
      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Alpha parameter
      double mAlpha;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_FUNCTIONS_INCOMPLETEGAMMAQFUNCTION_H
