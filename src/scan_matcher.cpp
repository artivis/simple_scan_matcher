#include "simple_scan_matcher/scan_matcher.h"

namespace simple_scan_matcher
{

typedef std::pair<uint, uint>   IndexPair;
typedef std::vector<IndexPair>  IndexPairs;

  void ScanMatcher::computeCorrespondences(const Scan &source, const Scan &target,
                                           Correspondences& correspondences)
  {
    correspondences.clear();

    uint last_index = 0;

    bool match = false;

    for (size_t i=0; i<source.size(); ++i)
    {
      double prev_dist = 0.2;

      Correspondence correspondence;

      match = false;

      std::vector<bool> is_matched(target.size(), false);

      for (size_t j=last_index; j<target.size(); ++j)
      {
        double xdiff = (source[i].getX() - target[j].getX());
        double ydiff = (source[i].getY() - target[j].getY());

        double dist = (xdiff*xdiff) + (ydiff*ydiff);

        if (dist < prev_dist && !is_matched[j])
        {
          prev_dist  = dist;
          correspondence = std::make_pair(i, j);

          is_matched[j] = true;
          match = true;
        }
      }

      if (match)
        correspondences.push_back(correspondence);
    }

    //std::cout << "source size : " << source.size() << std::endl;
    //std::cout << "target size : " << target.size() << std::endl;
    //std::cout << "Computed " << correspondences.size() << " correspondences." << std::endl;
    //for (int i=0; i<correspondences.size(); ++i)
    //  std::cout << " " << correspondences[i].first << "-" << correspondences[i].second;
    //std::cout << "\n" << std::endl;
  }

  Correspondences ScanMatcher::filterCorrespondences(const Scan &source, const Scan &target,
                                                     const Correspondences& correspondences,
                                                     const Similitude::ComplexVec& sim)
  {
    // Similarity (geometry) computed from
    // 2 pairs of corresponding points
    // compute all possible combinations of those pairs
    IndexPairs pairs_correspondence;
    for (uint i=0; i<correspondences.size()-1; ++i)
//      for (uint j=i+1; j<correspondences.size(); ++j)
        pairs_correspondence.push_back( std::make_pair(i, i+1) );

    double pairs_count = pairs_correspondence.size();

    std::vector<double> distances(pairs_count, 0);

    // compute Similitude for all pairs of correspondences
    for (size_t i=0; i<pairs_count; ++i)
    {
      Correspondences corr_tmp;

      corr_tmp.push_back( correspondences[ pairs_correspondence[i].first  ] );
      corr_tmp.push_back( correspondences[ pairs_correspondence[i].second ] );

      Similitude::Complex a(source[ corr_tmp[0].first  ].getX(), source[ corr_tmp[0].first  ].getY());
      Similitude::Complex b(source[ corr_tmp[1].first  ].getX(), source[ corr_tmp[1].first  ].getY());

      Similitude::Complex az(target[ corr_tmp[0].second ].getX(), target[ corr_tmp[0].second ].getY());
      Similitude::Complex bz(target[ corr_tmp[1].second ].getX(), target[ corr_tmp[1].second ].getY());

      Similitude::ComplexVec ab_tmp;
      Similitude::computeSimilitude(a, b, az, bz, ab_tmp);

      distances[i] = ( (sim[0].real()-ab_tmp[0].real())*(sim[1].real()-ab_tmp[1].real()) -
                                (sim[0].imag()-ab_tmp[0].imag())*(sim[1].imag()-ab_tmp[1].imag()) );
    }

    double sum  = std::accumulate(distances.begin(), distances.end(), 0.0);
    double mean = sum / pairs_count;

    double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
    double stdev  = std::sqrt(sq_sum / pairs_count - mean * mean);

//    std::cout << "distances: sum  : " << sum << " pairs_count: " << pairs_count << std::endl;
//    std::cout << "distances: mean : " << mean << " std_dev   : " << stdev << std::endl;

    // keep at least 75% of the population
    double stdev2 = stdev + stdev;

    // first validate coarsly by pair of correspondences
    std::vector<bool> valid_pairs(pairs_count, false);

    std::set<Correspondence> asserted_set;

    for (size_t i=0; i<pairs_count; ++i)
    {
      if (distances[i] <= stdev2)
      {
        //valid_pairs[i] = true;

        asserted_set.insert(correspondences[ pairs_correspondence[i].first ] );
        asserted_set.insert(correspondences[ pairs_correspondence[i].second] );
      }
    }

    Correspondences asserted_corr(asserted_set.begin(), asserted_set.end());

    return asserted_corr;
  }

  Scan ScanMatcher::transformScan(const Scan& source, const Similitude& sim)
  {
    Scan out(source.size());

    Similitude::SimMat sim_mat = sim.getTransformationMat();

    for (int i=0; i<source.size(); ++i)
      out[i] = source[i].transform(sim_mat);

    return out;
  }

  Similitude ScanMatcher::computeTransform(const Scan& source, const Scan& target,
                                           Correspondences& correspondences)
  {
    correspondences.clear();

    if (source.empty() || target.empty()) return Similitude();

    Scan source_tmp = source;

    Similitude sim;

    for (size_t i=0; i<100; ++i)
    {
      computeCorrespondences(source_tmp, target, correspondences);

      Similitude::ComplexVec ab_tmp;
      Similitude::computeSimilitude(source_tmp, target, correspondences, ab_tmp);

      Similitude sim_tmp(ab_tmp[0], ab_tmp[1]);

//      if(std::abs(1. - sim_tmp.getScale()) < 0.1)
        sim *= sim_tmp;

      if (std::fabs(sim_tmp.getTheta()) < 1e-7 ||
         (sim_tmp.getX()*sim_tmp.getX() + sim_tmp.getY()*sim_tmp.getY()) < 1e-7)
      {
        //std::cout << "Break at i : " << i << std::endl;
        break;
      }

      source_tmp = transformScan(source, sim);
    }

    return sim;
  }

  Similitude ScanMatcher::computeTransform(const Scan& source, const Scan& target)
  {
    Correspondences correspondences;

    return computeTransform(source, target, correspondences);
  }

} //namespace simple_scan_matcher
