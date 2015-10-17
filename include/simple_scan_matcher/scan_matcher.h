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

    virtual void computeCorrespondences(const Scan &source, const Scan &target, Correspondences& correspondences);

    Scan transformScan(const Scan& source, const Similitude& sim);

    Similitude computeTransform(const Scan& source, const Scan& target);

  private:

  };

} //namespace simple_scan_matcher

#endif //SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H
