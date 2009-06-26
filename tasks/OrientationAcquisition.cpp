#include "OrientationAcquisition.hpp"

#include <rtt/FileDescriptorActivity.hpp>


using namespace imu;


RTT::FileDescriptorActivity* OrientationAcquisition::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


OrientationAcquisition::OrientationAcquisition(std::string const& name)
    : OrientationAcquisitionBase(name)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationAcquisition.hpp for more detailed
// documentation about them.

// bool OrientationAcquisition::configureHook()
// {
//     return true;
// }
// bool OrientationAcquisition::startHook()
// {
//     return true;
// }

// void OrientationAcquisition::updateHook()
// {
// }

// void OrientationAcquisition::errorHook()
// {
// }
// void OrientationAcquisition::stopHook()
// {
// }
// void OrientationAcquisition::cleanupHook()
// {
// }

