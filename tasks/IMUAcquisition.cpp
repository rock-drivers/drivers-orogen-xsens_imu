#include "IMUAcquisition.hpp"

#include <rtt/FileDescriptorActivity.hpp>


using namespace imu;


RTT::FileDescriptorActivity* IMUAcquisition::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


IMUAcquisition::IMUAcquisition(std::string const& name)
    : IMUAcquisitionBase(name), m_driver(NULL)
{
}

IMUAcquisition::~IMUAcquisition()
{
    delete m_driver;
}
 

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IMUAcquisition.hpp for more detailed
// documentation about them.

bool IMUAcquisition::configureHook()
{
    std::auto_ptr<xsens_imu::XsensDriver> driver(new xsens_imu::XsensDriver());
    if( !driver->open( _port.value() ) ) {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    if( !driver->setReadingMode( xsens_imu::CAL_AND_ORI_DATA ) ) {
        std::cerr << "Error changing reading mode to CAL_AND_ORI_DATA";
        return false;
    }

    m_driver = driver.release();
    return true;
}

// bool IMUAcquisition::startHook()
// {
//     return true;
// }

void IMUAcquisition::updateHook()
{
    DFKI::IMUReading reading;
    xsens_imu::errorCodes retval;
    retval = m_driver->getReading();

    if( retval == xsens_imu::NO_ERROR ) {
        reading.orientation = m_driver->getOrientation();
        reading.accel = m_driver->getCalibratedAccData();
        reading.gyro = m_driver->getCalibratedGyroData();
        reading.mag = m_driver->getCalibratedMagData();
    } 

    if( retval == xsens_imu::ERROR_TIMEOUT ) {
        // a time out is not yet an error per se
        // TODO: add something like a timeout count which then would be an
        // error
        std::cerr << "IMU driver timout." << std::endl;
    }

    if( retval == xsens_imu::ERROR_OTHER ) {
        std::cerr << "IMU driver error" << std::endl;
        return error();
    }

    _imu_readings.write( reading );
}

// void IMUAcquisition::errorHook()
// {
// }
// void IMUAcquisition::stopHook()
// {
// }
// void IMUAcquisition::cleanupHook()
// {
// }

