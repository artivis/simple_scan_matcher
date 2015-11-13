#ifndef SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H
#define SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H

#include <vector>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{

  class ScanMatcher
  {
   public:

    ScanMatcher() { }

    ~ScanMatcher() { }

    void computeCorrespondences(const Scan &source, const Scan &target, Correspondences& correspondences);

    Correspondences filterCorrespondences(const Scan &source, const Scan &target,
                                          const Correspondences& correspondences,
                                          const Similitude::ComplexVec& sim);

    Scan transformScan(const Scan& source, const Similitude& sim);

    Similitude computeTransform(const Scan& source, const Scan& target,
                                Correspondences& correspondences);

    Similitude computeTransform(const Scan& source, const Scan& target);

  private:

  };

} //namespace simple_scan_matcher

#endif //SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H
