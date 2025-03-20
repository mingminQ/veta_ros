#include "veta/math/random/GaussianRandomizer.h"
#include "veta/math/random/UniformRandomizer.h"
#include "veta/util/LogHandler.h"

#include "veta_noetic/util/Timer.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RandomizerDemo");
    ros::NodeHandle nh;

    // global seed test


    veta::random::GaussianRandomizer grand;
    VETA_INFO("Global seed from gaussian randomizer >> %u", grand.getGlobalSeed());

    veta::random::UniformRandomizer urand;
    VETA_INFO("Global seed from uniform randomizer >> %u", urand.getGlobalSeed());
    std::cout << "---" << std::endl;

    // Gaussian randomizer test
    VETA_INFO("[Gaussian Randomizer]");
    VETA_INFO("real         >> %lf", grand.gaussianReal(0.0, 1.0));
    VETA_INFO("folded int   >> %d", grand.foldedGaussianInt(-1.0, 1.0, 3.0));
    VETA_INFO("folded real  >> %lf", grand.foldedGaussianReal(-1.0, 1.0, 2.0));
    std::cout << "---" << std::endl;

    // Uniform randomizer test


    return 0;
}