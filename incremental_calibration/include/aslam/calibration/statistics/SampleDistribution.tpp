/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors/Destructor                                                    */
/******************************************************************************/

    template <typename X>
    SampleDistribution<X>::~SampleDistribution() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X>
    void SampleDistribution<X>::getSamples(std::vector<X>& samples, size_t
        numSamples) const {
      samples.clear();
      samples.reserve(numSamples);
      for (size_t i = 0; i < numSamples; ++i)
        samples.push_back(getSample());
    }

  }
}
