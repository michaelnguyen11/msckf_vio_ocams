#ifndef RANDOM_NUMBER_GENERATOR_H_
#define RANDOM_NUMBER_GENERATOR_H_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

/// This file is taken from here
/// http://docs.ros.org/groovy/api/random_numbers/html/random__numbers_8h_source.html

namespace msckf_vio
{

class RandomNumberGenerator
{
public:
    RandomNumberGenerator(void);

    explicit RandomNumberGenerator(boost::uint32_t seed);

    double uniform01(void)
    {
        return uni_();
    }

    double uniformReal(double lower_bound, double upper_bound)
    {
        return (upper_bound - lower_bound) * uni_() + lower_bound;
    }

    double gaussian01(void)
    {
        return normal_();
    }

    double gaussian(double mean, double stddev)
    {
        return normal_() * stddev + mean;
    }

    void quaternion(double value[4]);

    int uniformInteger(int min, int max)
    {
        boost::uniform_int<> dis(min, max);
        return dis(generator_);
    }

    boost::uint32_t getFirstSeed();

private:
    boost::mt19937 generator_;
    boost::uniform_real<> uniDist_;
    boost::normal_distribution<> normalDist_;
    boost::variate_generator<boost::mt19937 &, boost::uniform_real<>> uni_;
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<>> normal_;
};

} // namespace msckf_vio

#endif