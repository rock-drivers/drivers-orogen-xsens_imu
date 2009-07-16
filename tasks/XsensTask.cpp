#include "XsensTask.hpp"

#include <rtt/FileDescriptorActivity.hpp>

using namespace imu;


RTT::FileDescriptorActivity* XsensTask::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


XsensTask::XsensTask(std::string const& name)
    : XsensTaskBase(name), m_driver(NULL), timeout_counter(0)
{
    _mode = xsens_imu::CAL_AND_ORI_DATA;
    _max_timeouts = 5;
}

XsensTask::~XsensTask()
{
    delete m_driver;
}
 

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See XsensTask.hpp for more detailed
// documentation about them.

bool XsensTask::configureHook()
{
    std::auto_ptr<xsens_imu::XsensDriver> driver(new xsens_imu::XsensDriver());
    if( !driver->open( _port.value() ) ) {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    if( !driver->setReadingMode( _mode ) ) {
        std::cerr << "Error changing reading mode to " << _mode;
        return false;
    }
    
    getFileDescriptorActivity()->watch(driver->getFileHandle());

    m_driver = driver.release();
    return true;
}

// bool IMUAcquisition::startHook()
// {
//     return true;
// }

void XsensTask::updateHook()
{
    // since this task is fd driven, we assume that the reception of data on
    // the port is the closest we can get to the actual acquisition time. This
    // can later be fixed to using a hardware sync-out from the IMU.
    DFKI::Time ts = DFKI::Time::now();

    xsens_imu::errorCodes retval;
    retval = m_driver->getReading();

    if( retval == xsens_imu::NO_ERROR ) {
        timeout_counter = 0;

        if( _mode == xsens_imu::CAL_AND_ORI_DATA || _mode == xsens_imu::ONLY_ORI_DATA ) {
            DFKI::OrientationReading reading;
            reading.stamp = ts;
            reading.value = m_driver->getOrientation();
            _orientation_readings.write( reading );
        }
        
        if( _mode == xsens_imu::CAL_AND_ORI_DATA || _mode == xsens_imu::ONLY_CAL_DATA ) {
            {
                DFKI::AccelerometerReading reading;
                reading.stamp = ts;
                reading.value = m_driver->getCalibratedAccData();
                _acc_readings.write( reading );
            }

            {
                DFKI::AngularRateReading reading;
                reading.stamp = ts;
                reading.value = m_driver->getCalibratedGyroData();
                _gyro_readings.write( reading );
            }

            {
                DFKI::MagnetometerReading reading;
                reading.stamp = ts;
                reading.value = m_driver->getCalibratedGyroData();
                _mag_readings.write( reading );
            }
        }
   } 

    if( retval == xsens_imu::ERROR_TIMEOUT ) {
        timeout_counter++;

        if( timeout_counter >= _max_timeouts ) {
            std::cerr << "IMU driver timout." << std::endl;
            return error();
        }
   }

    if( retval == xsens_imu::ERROR_OTHER ) {
        std::cerr << "IMU driver error" << std::endl;
        return error();
    }
}

// void XsensTask::errorHook()
// {
// }

// void XsensTask::stopHook()
// {
// }
void XsensTask::cleanupHook()
{
    m_driver->close();
}

