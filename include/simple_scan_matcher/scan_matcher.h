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

    void computeTransform(const Scan& source, const Scan& taget);

  private:

  };

} //namespace simple_scan_matcher

#endif //SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H
