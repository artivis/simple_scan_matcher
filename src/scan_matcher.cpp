#include "simple_scan_matcher/scan_matcher.h"

namespace simple_scan_matcher
{

  void ScanMatcher::computeCorrespondences(const Scan &source, const Scan &target,
                                           Correspondences& correspondences)
  {
    correspondences.clear();

    for (size_t i=0; i<source.size(); ++i)
    {
      double prev_dist = 99999.9;

      Correspondence correspondence;

      for (size_t j=0; j<target.size(); ++j)
      {
        double dist = ((source[i].getX() - target[j].getX()) * (source[i].getX() - target[j].getX())) +
                      ((source[i].getY() - target[j].getY()) * (source[i].getY() - target[j].getY()));

        if (dist < prev_dist)
        {
          prev_dist = dist;
          correspondence = std::make_pair(i, j);
        }
      }

      correspondences.push_back(correspondence);
    }

    /*
    std::cout << "Computed " << correspondences.size() << " correspondences." << std::endl;
    for (int i=0; i<correspondences.size(); ++i)
      std::cout << " " << correspondences[i].first << "-" << correspondences[i].second;
    std::cout << "\n" << std::endl;
    */
  }

  Scan ScanMatcher::transformScan(const Scan& source, const Similitude& sim)
  {
    Scan out(source.size());

    double theta = sim.getTheta();

    Similitude::SimMat sim_mat; //homogenous transform
    sim_mat << std::cos(theta), -std::sin(theta), sim.getScale() * sim.getX(),
               std::sin(theta),  std::cos(theta), sim.getScale() * sim.getY(),
               0, 0, 1;

    for (int i=0; i<source.size(); ++i)
      out[i] = source[i].transform(sim_mat);

    return out;
  }

  Similitude ScanMatcher::computeTransform(const Scan& source, const Scan& target)
  {
    if (source.empty() || target.empty()) return Similitude();

    Correspondences correspondences;
    computeCorrespondences(source, target, correspondences);

    return Similitude::computeSimilitude(source, target, correspondences);
  }

} //namespace simple_scan_matcher
