/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/exceptions/OutOfBoundException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <int M>
    VectorDesignVariable<M>::VectorDesignVariable(const Container& initValue) :
        _value(initValue),
        _oldValue(initValue) {
    }

    template <int M>
    VectorDesignVariable<M>::VectorDesignVariable(const VectorDesignVariable&
        other) :
        DesignVariable(other),
        _value(other._value),
        _oldValue(other._oldValue) {
    }

    template <int M>
    VectorDesignVariable<M>& VectorDesignVariable<M>::operator =
        (const VectorDesignVariable& other) {
      if (this != &other) {
        DesignVariable::operator=(other);
        _value = other._value;
        _oldValue = other._oldValue;
      }
      return *this;
    }

    template <int M>
    VectorDesignVariable<M>::~VectorDesignVariable() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <int M>
    void VectorDesignVariable<M>::read(std::istream& /*stream*/) {
    }

    template <int M>
    void VectorDesignVariable<M>::write(std::ostream& stream) const {
      stream << _value.transpose();
    }

    template <int M>
    void VectorDesignVariable<M>::read(std::ifstream& /*stream*/) {
    }

    template <int M>
    void VectorDesignVariable<M>::write(std::ofstream& stream) const {
      stream << _value.transpose();
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <int M>
    const typename VectorDesignVariable<M>::Container&
        VectorDesignVariable<M>::getValue() const {
      return _value;
    }

    template <int M>
    void VectorDesignVariable<M>::setValue(const Container& value) {
      _oldValue = _value;
      _value = value;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <int M>
    int VectorDesignVariable<M>::minimalDimensionsImplementation() const {
      return _value.rows();
    }

    template <int M>
    void VectorDesignVariable<M>::updateImplementation(const double* dp,
        int size) {
      Eigen::Map<const Container> update(dp, size);
      _oldValue = _value;
      _value += update;
    }

    template <int M>
    void VectorDesignVariable<M>::revertUpdateImplementation() {
      _value = _oldValue;
    }

    template<int M>
    void VectorDesignVariable<M>::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      value = _value;
    }

    template<int M>
    void VectorDesignVariable<M>::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      if (value.cols() != _value.cols() || value.rows() != _value.rows())
        throw OutOfBoundException<int>(value.cols(), _value.cols(),
          "dimensions must match", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      _oldValue = _value;
      _value = value;
    }

  }
}
