#include "veta/util/LogHandler.h"
#include "veta/math/Angle.h"
#include "veta/space/StateSpaceSE2.h"

#include "veta_noetic/util/Timer.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "StateSpaceDemo");
    ros::NodeHandle nh;

    using namespace veta;
    using namespace veta::space;
    StateSpaceSE2 spaceSE2;

    TIMER_START;

    auto *s1 = spaceSE2.newState()->state_cast<StateSE2>();
    auto *s2 = spaceSE2.newState()->state_cast<StateSE2>();

    s1->state_cast<StateR2>(StateSE2::POSITION)->m_elements[StateR2::X] = 1.0;
    s1->state_cast<StateR2>(StateSE2::POSITION)->m_elements[StateR2::Y] = 1.0;
    s1->state_cast<StateSO2>(StateSE2::ROTATION)->m_element = deg2rad(0.0);

    s2->state_cast<StateR2>(StateSE2::POSITION)->m_elements[StateR2::X] = 2.0;
    s2->state_cast<StateR2>(StateSE2::POSITION)->m_elements[StateR2::Y] = 2.0;
    s2->state_cast<StateSO2>(StateSE2::ROTATION)->m_element = deg2rad(45.0);

    double dist = spaceSE2.distance(s1, s2);
    State *s3 = spaceSE2.interpolate(s1, s2, 0.5);

    VETA_INFO("Distance >> %lf", dist);
    spaceSE2.printState(s3);

    spaceSE2.deleteState(s1);
    spaceSE2.deleteState(s2);
    spaceSE2.deleteState(s3);

    TIMER_END;

    return 0;
}

