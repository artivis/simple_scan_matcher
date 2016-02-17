#ifndef SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H
#define SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H

#include <vector>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{

  class ScanMatcher
  {
   public:

    ScanMatcher() : _match_min_dist(0.2), _weight_lsq(false) { }

    ~ScanMatcher() { }

    void computeCorrespondences(const Scan &source, const Scan &target,
                                Correspondences& correspondences,
                                std::vector<double>& distances);

    Correspondences filterCorrespondences(const Scan &source, const Scan &target,
                                          const Correspondences& correspondences,
                                          const Similitude::ComplexVector2& simvec);

    Scan transformScan(const Scan& source, const Similitude& sim);

    Similitude computeTransform(const Scan& source, const Scan& target,
                                Correspondences& correspondences);

    Similitude computeTransform(const Scan& source, const Scan& target,
                                Correspondences& correspondences,
                                std::vector<double>& distances);

    Similitude computeTransform(const Scan& source, const Scan& target);

  protected:

    bool _weight_lsq;

    double _match_min_dist;

  };

} //namespace simple_scan_matcher

#endif //SIMPLE_SCAN_MATCHER_SCAN_MATCHER_H
