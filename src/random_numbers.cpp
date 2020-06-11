#include "random_numbers.h"

#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/scoped_ptr.hpp>

/// This file is taken from here
/// http://docs.ros.org/groovy/api/random_numbers/html/random__numbers_8cpp_source.html

// RandomNumberGenerator is used in RANSAC outlier removal in Feature Tracking

// TODO: if possible, remove dependency of boost library

namespace msckf_vio
{

static boost::uint32_t first_seed_ = 0;

static boost::uint32_t firstSeed(void)
{
    boost::scoped_ptr<int> mem(new int());
    first_seed_ = (boost::uint32_t)((boost::posix_time::microsec_clock::universal_time() -
                                     boost::posix_time::ptime(boost::date_time::min_date_time))
                                        .total_microseconds() +
                                    (unsigned long long)(mem.get()));
    return first_seed_;
}

static boost::uint32_t nextSeed(void)
{
    static boost::mutex rngMutex;
    boost::mutex::scoped_lock slock(rngMutex);
    static boost::lagged_fibonacci607 sGen(firstSeed());
    static boost::uniform_int<> sDist(1, 1000000000);
    static boost::variate_generator<boost::lagged_fibonacci607 &, boost::uniform_int<>> s(sGen, sDist);
    boost::uint32_t v = s();
    return v;
}

RandomNumberGenerator::RandomNumberGenerator(void) : generator_(nextSeed()),
                                                     uniDist_(0, 1),
                                                     normalDist_(0, 1),
                                                     uni_(generator_, uniDist_),
                                                     normal_(generator_, normalDist_)
{
}

RandomNumberGenerator::RandomNumberGenerator(boost::uint32_t seed)
    : generator_(seed),
      uniDist_(0, 1),
      normalDist_(0, 1),
      uni_(generator_, uniDist_),
      normal_(generator_, normalDist_)
{
    // Because we manually specified a seed, we need to save it ourselves
    first_seed_ = seed;
}

// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
//       pg. 124-132
void RandomNumberGenerator::quaternion(double value[4])
{
    double x0 = uni_();
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * boost::math::constants::pi<double>() * uni_(), t2 = 2.0 * boost::math::constants::pi<double>() * uni_();
    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    value[0] = s1 * r1;
    value[1] = c1 * r1;
    value[2] = s2 * r2;
    value[3] = c2 * r2;
}

boost::uint32_t RandomNumberGenerator::getFirstSeed()
{
    return first_seed_;
}

} // namespace msckf_vio
